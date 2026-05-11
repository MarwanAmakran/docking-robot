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
        self.center_threshold = 10
        self.forward_speed    = 0.28
        self.max_turn         = 0.1
        self.search_turn      = 0.2
        self.undock_speed     = -0.15

        # Timing
        self.cmd_interval    = 1.0
        self.last_cmd_time   = 0.0
        self.last_turn_time  = 0.0
        self.wait_after_turn = 1.0

        # Failsafes
        self.search_timeout   = 15.0
        self.approach_timeout = 60.0
        self.max_lost_marker  = 10
        self.lost_marker_count = 0
        self.min_battery      = 9.5

        # Batterij
        self.battery_voltage = 0.0
        self.prev_voltage    = 0.0
        self.charging_status = 'ONBEKEND'

        # State
        self.state          = IDLE
        self.search_start   = None
        self.approach_start = None
        self.undock_start   = None
        self.running        = True

        self.get_logger().info('=== Docking Controller gestart! ===')
        self.get_logger().info('D = start | U = undock | CTRL+C = stop')

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def battery_callback(self, msg):
        self.prev_voltage    = self.battery_voltage
        self.battery_voltage = msg.voltage
        if self.prev_voltage > 0.0:
            diff = self.battery_voltage - self.prev_voltage
            if diff > 0.05:
                self.charging_status = 'OPLADEN'
                if self.state == APPROACH:
                    self.get_logger().info('Spanning stijgt — DOCKED!')
                    self.publish_stop()
                    self.set_state(DOCKED)
            elif diff < -0.05:
                self.charging_status = 'ONTLADEN'
            else:
                self.charging_status = 'STABIEL'

        if self.battery_voltage > 0.0 and self.battery_voltage < self.min_battery:
            if self.state not in [IDLE, DOCKED]:
                self.get_logger().error('FAILSAFE: Batterij kritiek!')
                self.publish_stop()
                self.set_state(IDLE)

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = sys.stdin.read(1)
                if key == '\x03':
                    self.running = False
                    self.publish_stop()
                    break
                elif key.lower() == 'd':
                    if self.state == IDLE:
                        self.get_logger().info('START DOCKING!')
                        self.set_state(SEARCH)
                    else:
                        self.get_logger().info(f'Al bezig: {self.state}')
                elif key.lower() == 'u':
                    if self.state == DOCKED:
                        self.set_state(UNDOCK)
                    else:
                        self.get_logger().info('Niet in DOCKED state')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def set_state(self, new_state):
        self.state             = new_state
        self.search_start      = None
        self.approach_start    = None
        self.lost_marker_count = 0
        self.last_cmd_time     = 0.0
        self.last_turn_time    = 0.0
        self.get_logger().info(f'=== STATE: {new_state} ===')

    def publish_stop(self):
        try:
            twist = Twist()
            self.cmd_pub.publish(twist)
        except Exception:
            pass

    def send_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)
        self.last_cmd_time  = time.time()
        if angular != 0.0:
            self.last_turn_time = time.time()

    def get_aruco_error(self, frame):
        """Geeft error in pixels terug, of None als geen marker."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector   = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None:
            return None
        c  = corners[0][0]
        cx = int(c[:, 0].mean())
        return cx - (frame.shape[1] // 2)

    def image_callback(self, msg):
        if self.state in [IDLE, DOCKED]:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if self.state == UNDOCK:
            if self.undock_start is None:
                self.undock_start = time.time()
            if time.time() - self.undock_start >= 3.0:
                self.publish_stop()
                self.undock_start = None
                self.set_state(IDLE)
            else:
                now = time.time()
                if now - self.last_cmd_time >= self.cmd_interval:
                    self.send_cmd(self.undock_speed, 0.0)
                    self.get_logger().info('UNDOCK | Achteruit...')
            return

        if self.state == APPROACH:
            if self.approach_start is None:
                self.approach_start = time.time()
            elapsed = time.time() - self.approach_start
            if elapsed > self.approach_timeout:
                self.get_logger().warn('FAILSAFE: timeout — STOP')
                self.publish_stop()
                self.set_state(IDLE)
                return

            now = time.time()
            if now - self.last_cmd_time < self.cmd_interval:
                return

            # Check ArUco tijdens approach
            error = self.get_aruco_error(frame)
            if error is not None:
                if abs(error) > self.center_threshold:
                    # Afwijking — stuur bij met vooruit + klein draaien
                    turn = max(min(-float(error) / 200.0, self.max_turn), -self.max_turn)
                    self.send_cmd(self.forward_speed, turn)
                    self.get_logger().info(
                        f'APPROACH | Bijsturen | Error: {error}px | '
                        f'Bat: {self.battery_voltage:.2f}V')
                else:
                    # Gecentreerd — recht vooruit
                    self.send_cmd(self.forward_speed, 0.0)
                    self.get_logger().info(
                        f'APPROACH | Recht vooruit | {elapsed:.0f}sec | '
                        f'Bat: {self.battery_voltage:.2f}V')
            else:
                # Geen marker — recht vooruit
                self.send_cmd(self.forward_speed, 0.0)
                self.get_logger().info(
                    f'APPROACH | {elapsed:.0f}sec | '
                    f'Bat: {self.battery_voltage:.2f}V | {self.charging_status}')
            return

        # SEARCH en ALIGN
        error = self.get_aruco_error(frame)
        marker_found = error is not None

        if self.state == SEARCH:
            if self.search_start is None:
                self.search_start = time.time()
            if marker_found:
                self.set_state(ALIGN)
            else:
                if time.time() - self.search_start > self.search_timeout:
                    self.get_logger().warn('Marker niet gevonden — IDLE')
                    self.publish_stop()
                    self.set_state(IDLE)
                else:
                    now = time.time()
                    if now - self.last_cmd_time >= self.cmd_interval:
                        self.send_cmd(0.0, self.search_turn)
                        self.get_logger().info('SEARCH | Zoeken...')

        elif self.state == ALIGN:
            if not marker_found:
                self.lost_marker_count += 1
                if self.lost_marker_count >= self.max_lost_marker:
                    self.get_logger().warn('Marker kwijt — SEARCH')
                    self.publish_stop()
                    self.set_state(SEARCH)
                return

            self.lost_marker_count = 0
            now = time.time()

            # Wacht 1 sec na laatste draai
            if now - self.last_turn_time < self.wait_after_turn:
                return

            if abs(error) <= self.center_threshold:
                self.get_logger().info(f'GECENTREERD! Error: {error}px — APPROACH!')
                self.publish_stop()
                self.set_state(APPROACH)
            else:
                if now - self.last_cmd_time >= self.cmd_interval:
                    turn = max(min(-float(error) / 200.0, self.max_turn), -self.max_turn)
                    self.send_cmd(0.0, turn)
                    self.get_logger().info(f'ALIGN | Error: {error}px | Draait...')

    def destroy_node(self):
        self.running = False
        self.publish_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        node.publish_stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
