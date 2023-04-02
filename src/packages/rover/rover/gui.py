import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebcamReceiver(Node):
    def __init__(self):
        super().__init__('webcam_receiver')
        self.subscription1 = self.create_subscription(Image, 'hand_camera', self.callback1, 10)
        self.subscription2 = self.create_subscription(Image, 'chest_camera', self.callback2, 10)
        self.subscription3 = self.create_subscription(Image, 'sky_camera', self.callback3, 10)
        self.bridge = CvBridge()

    def callback1(self, msg):
        self.display_image(msg, 'Camera Feed 1')

    def callback2(self, msg):
        self.display_image(msg, 'Camera Feed 2')

    def callback3(self, msg):
        self.display_image(msg, 'Camera Feed 3')

    def display_image(self, msg, window_name):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(window_name, image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamReceiver()
    rclpy.spin(node)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
