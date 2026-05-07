import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import time
import threading
import sys
import tty
import termios

# States
IDLE     = 'IDLE'
SEARCH   = 'SEARCH'
ALIGN    = 'ALIGN'
APPROACH = 'APPROACH'
DOCKED   = 'DOCKED'
UNDOCK   = 'UNDOCK'

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Instellingen
        self.center_threshold = 30
        self.forward_speed    = 0.15
        self.max_turn         = 0.4
        self.search_turn      = 0.3
        self.undock_speed     = -0.15
        self.marker_size_dock = 200
        self.undock_duration  = 3.0

        # Failsafe instellingen
        self.search_timeout        = 10.0  # sec zoeken voor opgeven
        self.approach_timeout      = 30.0  # sec vooruit voor opgeven
        self.min_battery_voltage   = 9.5   # onder 9.5V = kritiek
        self.max_lost_marker_count = 5     # marker X keer kwijt = terug SEARCH

        # State
        self.state             = IDLE
        self.last_cmd_time     = 0.0
        self.cmd_interval      = 2.0
        self.undock_start      = None
        self.search_start      = None
        self.approach_start    = None
        self.lost_marker_count = 0
        self.running           = True

        # Batterij
        self.battery_voltage = 0.0
        self.prev_voltage    = 0.0
        self.charging_status = 'ONBEKEND'

        self.get_logger().info('=== Docking Controller gestart! ===')
        self.get_logger().info('Druk D om te starten | Druk U om te undocken | CTRL+C om te stoppen')

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def battery_callback(self, msg):
        self.prev_voltage    = self.battery_voltage
        self.battery_voltage = msg.voltage

        if self.prev_voltage > 0.0:
            diff = self.battery_voltage - self.prev_voltage
            if diff > 0.05:
                self.charging_status = 'OPLADEN ⚡'
            elif diff < -0.05:
                self.charging_status = 'ONTLADEN 🔋'
            else:
                self.charging_status = 'STABIEL'

        # FAILSAFE — batterij kritiek laag
        if self.battery_voltage > 0.0 and self.battery_voltage < self.min_battery_voltage:
            if self.state not in [IDLE, DOCKED]:
                self.get_logger().error(
                    f'⚠️ FAILSAFE: Batterij kritiek laag! {self.battery_voltage:.2f}V — STOP!')
                self.stop_robot()
                self.set_state(IDLE)

        self.get_logger().info(
            f'Batterij: {self.battery_voltage:.2f}V | {self.charging_status}')

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = sys.stdin.read(1)
                if key == '\x03':
                    self.get_logger().info('CTRL+C — Stoppen!')
                    self.running = False
                    self.stop_robot()
                    rclpy.shutdown()
                    break
                elif key.lower() == 'd':
                    if self.state == IDLE:
                        self.get_logger().info('D ingedrukt — START DOCKING!')
                        self.set_state(SEARCH)
                    else:
                        self.get_logger().info(f'Al bezig: {self.state}')
                elif key.lower() == 'u':
                    if self.state == DOCKED:
                        self.get_logger().info('U ingedrukt — UNDOCK!')
                        self.set_state(UNDOCK)
                    else:
                        self.get_logger().info(f'Kan niet undocken vanuit {self.state}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def set_state(self, new_state):
        self.state = new_state
        # Reset timers bij state wissel
        self.search_start   = None
        self.approach_start = None
        self.lost_marker_count = 0
        self.get_logger().info(f'=== STATE: {new_state} ===')

    def stop_robot(self):
        twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(twist)

    def publish_cmd(self, linear, angular):
        now = time.time()
        if now - self.last_cmd_time < self.cmd_interval:
            return False
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)
        self.last_cmd_time = now
        return True

    def get_marker_size(self, corners):
        c      = corners[0][0]
        width  = c[1][0] - c[0][0]
        height = c[2][1] - c[0][1]
        return abs(width * height) ** 0.5

    def get_direction(self, error):
        if error < -self.center_threshold:
            return 'LINKS ←'
        elif error > self.center_threshold:
            return 'RECHTS →'
        else:
            return 'MIDDEN ✓'

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector   = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        marker_found = ids is not None

        if self.state == IDLE:
            return

        # SEARCH
        elif self.state == SEARCH:
            if self.search_start is None:
                self.search_start = time.time()
            if marker_found:
                self.set_state(ALIGN)
            else:
                # FAILSAFE — search timeout
                if time.time() - self.search_start > self.search_timeout:
                    self.get_logger().warn('⚠️ FAILSAFE: Marker niet gevonden na 10sec — IDLE')
                    self.stop_robot()
                    self.set_state(IDLE)
                else:
                    remaining = self.search_timeout - (time.time() - self.search_start)
                    self.publish_cmd(0.0, self.search_turn)
                    self.get_logger().info(f'SEARCH | Zoeken... nog {remaining:.0f}sec')

        # ALIGN
        elif self.state == ALIGN:
            if not marker_found:
                # FAILSAFE — marker kwijt tijdens align
                self.lost_marker_count += 1
                self.get_logger().warn(f'⚠️ Marker kwijt! ({self.lost_marker_count}/{self.max_lost_marker_count})')
                if self.lost_marker_count >= self.max_lost_marker_count:
                    self.get_logger().warn('⚠️ FAILSAFE: Marker te vaak kwijt — SEARCH')
                    self.stop_robot()
                    self.set_state(SEARCH)
                return
            self.lost_marker_count = 0
            c  = corners[0][0]
            cx = int(c[:, 0].mean())
            img_center  = frame.shape[1] // 2
            error       = cx - img_center
            turn        = max(min(-float(error) / 200.0, self.max_turn), -self.max_turn)
            marker_size = self.get_marker_size(corners)
            direction   = self.get_direction(error)

            if abs(error) <= self.center_threshold:
                self.set_state(APPROACH)
            else:
                if self.publish_cmd(0.0, turn):
                    self.get_logger().info(
                        f'[ALIGN] {direction} | Error: {error}px | '
                        f'Marker: {marker_size:.0f}px | '
                        f'Batterij: {self.battery_voltage:.2f}V')

        # APPROACH
        elif self.state == APPROACH:
            if self.approach_start is None:
                self.approach_start = time.time()

            if not marker_found:
                # FAILSAFE — marker kwijt tijdens approach
                self.lost_marker_count += 1
                self.get_logger().warn(f'⚠️ Marker kwijt tijdens APPROACH! ({self.lost_marker_count}/{self.max_lost_marker_count})')
                if self.lost_marker_count >= self.max_lost_marker_count:
                    self.get_logger().warn('⚠️ FAILSAFE: Marker kwijt — STOP en SEARCH')
                    self.stop_robot()
                    self.set_state(SEARCH)
                return

            self.lost_marker_count = 0
            c  = corners[0][0]
            cx = int(c[:, 0].mean())
            img_center  = frame.shape[1] // 2
            error       = cx - img_center
            marker_size = self.get_marker_size(corners)
            direction   = self.get_direction(error)

            # FAILSAFE — approach timeout
            if time.time() - self.approach_start > self.approach_timeout:
                self.get_logger().warn('⚠️ FAILSAFE: Approach timeout — STOP')
                self.stop_robot()
                self.set_state(IDLE)
                return

            if abs(error) > self.center_threshold:
                self.set_state(ALIGN)
            elif marker_size >= self.marker_size_dock:
                self.stop_robot()
                self.set_state(DOCKED)
            else:
                if self.publish_cmd(self.forward_speed, 0.0):
                    self.get_logger().info(
                        f'[APPROACH] {direction} | '
                        f'Afstand: {marker_size:.0f}px | '
                        f'Batterij: {self.battery_voltage:.2f}V')

        # DOCKED
        elif self.state == DOCKED:
            self.stop_robot()
            self.get_logger().info(
                f'*** DOCKED *** | Batterij: {self.battery_voltage:.2f}V | '
                f'Status: {self.charging_status} | Druk U om te undocken!')

        # UNDOCK
        elif self.state == UNDOCK:
            if self.undock_start is None:
                self.undock_start = time.time()
            if time.time() - self.undock_start >= self.undock_duration:
                self.stop_robot()
                self.undock_start = None
                self.set_state(IDLE)
            else:
                if self.publish_cmd(self.undock_speed, 0.0):
                    self.get_logger().info(
                        f'UNDOCK | Achteruit... | '
                        f'Batterij: {self.battery_voltage:.2f}V')

    def destroy_node(self):
        self.running = False
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
