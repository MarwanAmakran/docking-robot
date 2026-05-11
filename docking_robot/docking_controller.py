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

        # Centreer instellingen
        self.center_threshold    = 20
        self.align_stable_needed = 8
        self.align_stable_count  = 0

        # Snelheden — rustiger draaien!
        self.forward_speed = 0.25
        self.max_turn      = 0.3   # was 0.5, nu rustiger
        self.search_turn   = 0.25
        self.undock_speed  = -0.2

        # Afstand
        self.marker_real_size     = 100.0
        self.focal_length         = 600.0
        self.measured_distance_mm = 0.0
        self.approach_start_time  = None
        self.approach_duration    = 0.0

        # Intervals
        self.align_cmd_time    = 0.0
        self.align_interval    = 0.5   # stuur commando elke 0.5 sec tijdens align
        self.approach_cmd_time = 0.0
        self.approach_interval = 1.0
        self.search_cmd_time   = 0.0
        self.search_interval   = 1.0
        self.log_counter       = 0

        # Failsafes
        self.search_timeout   = 15.0
        self.approach_timeout = 30.0
        self.max_lost_marker  = 15
        self.min_battery      = 9.5

        # State
        self.state             = IDLE
        self.undock_start      = None
        self.search_start      = None
        self.lost_marker_count = 0
        self.running           = True

        # Batterij
        self.battery_voltage = 0.0
        self.prev_voltage    = 0.0
        self.charging_status = 'ONBEKEND'

        self.get_logger().info('=== Docking Controller gestart! ===')
        self.get_logger().info('D = start docking | U = undock | CTRL+C = stop')

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
            elif diff < -0.05:
                self.charging_status = 'ONTLADEN'
            else:
                self.charging_status = 'STABIEL'
        if self.battery_voltage > 0.0 and self.battery_voltage < self.min_battery:
            if self.state not in [IDLE, DOCKED]:
                self.get_logger().error(f'FAILSAFE: Batterij kritiek! {self.battery_voltage:.2f}V')
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
        self.state               = new_state
        self.search_start        = None
        self.approach_start_time = None
        self.lost_marker_count   = 0
        self.align_stable_count  = 0
        self.log_counter         = 0
        self.get_logger().info(f'=== STATE: {new_state} ===')

    def publish_stop(self):
        try:
            twist = Twist()
            self.cmd_pub.publish(twist)
        except Exception:
            pass

    def send_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    def get_distance_mm(self, corners):
        c         = corners[0][0]
        width     = abs(c[1][0] - c[0][0])
        height    = abs(c[2][1] - c[0][1])
        marker_px = (width + height) / 2.0
        if marker_px <= 0:
            return 9999
        return (self.marker_real_size * self.focal_length) / marker_px

    def get_direction(self, error):
        if error < -self.center_threshold:
            return 'LINKS'
        elif error > self.center_threshold:
            return 'RECHTS'
        else:
            return 'MIDDEN'

    def image_callback(self, msg):
        if self.state in [IDLE, DOCKED]:
            return

        self.log_counter += 1

        if self.state == UNDOCK:
            if self.undock_start is None:
                self.undock_start = time.time()
            if time.time() - self.undock_start >= 3.0:
                self.publish_stop()
                self.undock_start = None
                self.set_state(IDLE)
            else:
                now = time.time()
                if now - self.approach_cmd_time >= self.approach_interval:
                    self.send_twist(self.undock_speed, 0.0)
                    self.approach_cmd_time = now
                    self.get_logger().info('UNDOCK | Achteruit...')
            return

        if self.state == APPROACH:
            if self.approach_start_time is None:
                self.approach_start_time = time.time()
            elapsed   = time.time() - self.approach_start_time
            remaining = self.approach_duration - elapsed
            if elapsed > self.approach_timeout:
                self.get_logger().warn('FAILSAFE: Approach timeout — STOP')
                self.publish_stop()
                self.set_state(IDLE)
                return
            if elapsed >= self.approach_duration:
                self.publish_stop()
                self.get_logger().info(f'Gereden: {self.measured_distance_mm:.0f}mm — DOCKED!')
                self.set_state(DOCKED)
            else:
                now = time.time()
                if now - self.approach_cmd_time >= self.approach_interval:
                    self.send_twist(self.forward_speed, 0.0)
                    self.approach_cmd_time = now
                    self.get_logger().info(f'APPROACH | Rechtdoor... nog {remaining:.1f}sec')
            return

        # Camera verwerking alleen voor SEARCH en ALIGN
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector   = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        marker_found = ids is not None

        if self.state == SEARCH:
            if self.search_start is None:
                self.search_start = time.time()
            if marker_found:
                self.set_state(ALIGN)
            else:
                if time.time() - self.search_start > self.search_timeout:
                    self.get_logger().warn('FAILSAFE: Marker niet gevonden — IDLE')
                    self.publish_stop()
                    self.set_state(IDLE)
                else:
                    now = time.time()
                    if now - self.search_cmd_time >= self.search_interval:
                        self.send_twist(0.0, self.search_turn)
                        self.search_cmd_time = now
                        remaining = self.search_timeout - (time.time() - self.search_start)
                        self.get_logger().info(f'SEARCH | Zoeken... {remaining:.0f}sec')

        elif self.state == ALIGN:
            if not marker_found:
                self.lost_marker_count += 1
                if self.lost_marker_count >= self.max_lost_marker:
                    self.get_logger().warn('FAILSAFE: Marker kwijt — SEARCH')
                    self.publish_stop()
                    self.set_state(SEARCH)
                return

            self.lost_marker_count  = 0
            c          = corners[0][0]
            cx         = int(c[:, 0].mean())
            img_center = frame.shape[1] // 2
            error      = cx - img_center
            direction  = self.get_direction(error)
            # Proportioneel maar rustiger
            turn        = max(min(-float(error) / 200.0, self.max_turn), -self.max_turn)
            distance_mm = self.get_distance_mm(corners)

            now = time.time()
            if abs(error) <= self.center_threshold:
                # Gecentreerd — stuur stop en tel frames
                self.send_twist(0.0, 0.0)
                self.align_stable_count += 1
                # Log elke 3 frames
                if self.log_counter % 3 == 0:
                    self.get_logger().info(
                        f'Centreren... {self.align_stable_count}/{self.align_stable_needed} | '
                        f'Afstand: {distance_mm:.0f}mm')
                if self.align_stable_count >= self.align_stable_needed:
                    self.measured_distance_mm = distance_mm
                    self.approach_duration    = (distance_mm / 1000.0) / self.forward_speed
                    self.get_logger().info(
                        f'ALIGN klaar! Afstand: {distance_mm:.0f}mm | '
                        f'Rijd {self.approach_duration:.1f}sec rechtdoor')
                    self.set_state(APPROACH)
            else:
                # Niet gecentreerd — bijsturen met interval
                self.align_stable_count = 0
                if now - self.align_cmd_time >= self.align_interval:
                    self.send_twist(0.0, turn)
                    self.align_cmd_time = now
                    self.get_logger().info(
                        f'ALIGN | {direction} | Error: {error}px | '
                        f'Afstand: {distance_mm:.0f}mm')

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
