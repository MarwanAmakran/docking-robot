import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
import serial
import time

class CmdVelSerialBridge(Node):
    def __init__(self):
        super().__init__('cmdvel_serial_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter("max_pwm", 30)
        self.declare_parameter('linear_gain', 50.0)
        self.declare_parameter('angular_gain', 50.0)
        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.max_pwm      = int(self.get_parameter('max_pwm').value)
        self.linear_gain  = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)

        self.ser = serial.Serial(port, baud, timeout=2)
        self.get_logger().info(f"Connected to {port} @ {baud}")

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_timer(5.0, self.read_battery)

    def cmd_callback(self, msg):
        linear  = float(msg.linear.x)
        angular = float(msg.angular.z)
        left    = int(self.linear_gain * linear - self.angular_gain * angular)
        right   = int(self.linear_gain * linear + self.angular_gain * angular)
        left    = max(min(left,  self.max_pwm), -self.max_pwm)
        right   = max(min(right, self.max_pwm), -self.max_pwm)
        command = f"D {left} {right} 1\n"
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent: {command.strip()}")

    def read_battery(self):
        try:
            self.ser.write(b'battery\n')
            time.sleep(0.5)
            response = self.ser.readline().decode('utf-8').strip()
            if 'BatteryVoltage:' in response:
                voltage = float(response.replace('BatteryVoltage:', '').replace('V', '').strip())
                msg = BatteryState()
                msg.voltage = voltage
                self.battery_pub.publish(msg)
                self.get_logger().info(f'Batterij: {voltage:.2f}V')
        except Exception as e:
            self.get_logger().error(f'Batterij fout: {e}')

    def destroy_node(self):
        try:
            self.ser.write(b"D 0 0 1\n")
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
