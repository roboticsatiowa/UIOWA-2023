import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebcamStreamer(Node):
    def __init__(self):
        super().__init__('webcam_streamer')
        self.publisher1 = self.create_publisher(Image, 'hand_camera', 10)
        self.publisher2 = self.create_publisher(Image, 'chest_camera', 10)
        self.publisher3 = self.create_publisher(Image, 'sky_camera', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)  # Approx 30 FPS
        self.cap_hand = cv2.VideoCapture(2)
        self.cap_chest = cv2.VideoCapture(0)
        self.cap_sky = cv2.VideoCapture(4)	#Change these values according to the indexes of the cameras.
        self.bridge = CvBridge()

    def timer_callback(self):
        hand_ret, hand_frame = self.cap_hand.read()
        chest_ret, chest_frame = self.cap_chest.read()
        sky_ret, sky_frame = self.cap_sky.read()
        if hand_ret and chest_ret and sky_ret:
            hand_image_msg = self.bridge.cv2_to_imgmsg(hand_frame, encoding='bgr8')
            chest_image_msg = self.bridge.cv2_to_imgmsg(chest_frame, encoding='bgr8')
            sky_image_msg = self.bridge.cv2_to_imgmsg(sky_frame, encoding='bgr8')
            self.publisher1.publish(hand_image_msg)
            self.publisher2.publish(chest_image_msg)
            self.publisher3.publish(sky_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamStreamer()
    rclpy.spin(node)

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
