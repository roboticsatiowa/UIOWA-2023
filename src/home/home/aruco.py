import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco
import numpy as np

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        self.bridge = CvBridge()

        # Subscriptions
        self.subscription1 = self.create_subscription(Image, 'hand_camera', self.image_callback1, 10)
        self.subscription2 = self.create_subscription(Image, 'chest_camera', self.image_callback2, 10)
        self.subscription3 = self.create_subscription(Image, 'sky_camera', self.image_callback3, 10)

        # Publishers
        self.publisher1 = self.create_publisher(Image, 'hand_aruco', 10)
        self.publisher2 = self.create_publisher(Image, 'chest_aruco', 10)
        self.publisher3 = self.create_publisher(Image, 'sky_aruco', 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback1(self, msg):
        self.process_image(msg, "camera1")

    def image_callback2(self, msg):
        self.process_image(msg, "camera2")

    def image_callback3(self, msg):
        self.process_image(msg, "camera3")

    def process_image(self, msg, camera_name):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        frame = cv2.aruco.drawDetectedMarkers(frame, corners)

        if ids is not None:
            for i in range(len(ids)):
                frame = cv2.putText(frame, str(ids[i][0]), tuple(corners[i][0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 2)

        modified_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        modified_image_msg.header.frame_id = camera_name
        modified_image_msg.header.stamp = self.get_clock().now().to_msg()

        if camera_name == "camera1":
            self.publisher1.publish(modified_image_msg)
        elif camera_name == "camera2":
            self.publisher2.publish(modified_image_msg)
        elif camera_name == "camera3":
            self.publisher3.publish(modified_image_msg)

def main(args=None):
    rclpy.init(args=args)

    aruco_node = ArucoNode()

    try:
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        pass

    aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
