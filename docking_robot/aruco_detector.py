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
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.get_logger().info('ArUco Detector gestart!')

    def image_callback(self, msg):
        # Zet ROS image om naar OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ArUco detectie
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            # Marker gevonden!
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Bereken middelpunt van marker
            c = corners[0][0]
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())
            
            # Beeldmidden
            img_center = frame.shape[1] // 2
            error = cx - img_center
            
            self.get_logger().info(f'Marker ID: {ids[0][0]} | Midden: {cx} | Error: {error}px')
        else:
            self.get_logger().info('Geen marker gevonden...')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
