import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import serial
import time

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(BatteryState, '/battery_state', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=2)
        self.get_logger().info('Battery Publisher gestart!')
        # Vraag elke 5 seconden de voltage op
        self.create_timer(5.0, self.read_battery)

    def read_battery(self):
        try:
            self.ser.write(b'battery\n')
            time.sleep(0.5)
            response = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f'Response: {response}')

            if 'BatteryVoltage:' in response:
                voltage = float(response.replace('BatteryVoltage:', '').replace('V', '').strip())
                msg = BatteryState()
                msg.voltage = voltage
                self.publisher_.publish(msg)
                self.get_logger().info(f'Batterij: {voltage:.2f}V')
        except Exception as e:
            self.get_logger().error(f'Fout: {e}')

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
