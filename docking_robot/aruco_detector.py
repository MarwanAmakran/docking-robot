import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.get_logger().info('ArUco Detector gestart!')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            c = corners[0][0]
            cx = int(c[:, 0].mean())
            img_center = frame.shape[1] // 2
            error = cx - img_center

            # Marker grootte berekenen
            width  = c[1][0] - c[0][0]
            height = c[2][1] - c[0][1]
            size   = abs(width * height) ** 0.5

            self.get_logger().info(
                f'Marker ID: {ids[0][0]} | '
                f'Midden: {cx} | '
                f'Error: {error}px | '
                f'Grootte: {size:.0f}px')
        else:
            self.get_logger().info('Geen marker gevonden...')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
