import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QPushButton
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self, topic, node_name):
        super().__init__(node_name)
        self.subscription = self.create_subscription(Image, topic, self.image_callback, 10)
        self.bridge = CvBridge()
        self.cv_image = None

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

class TimerModule:
    def __init__(self):
        self.timer = QTimer()
        self.time_elapsed = 0
        self.timer.timeout.connect(self.update_time)

    def start(self):
        self.timer.start(1000)

    def pause(self):
        self.timer.stop()

    def reset(self):
        self.time_elapsed = 0

    def update_time(self):
        self.time_elapsed += 1

class RoverGUI(QMainWindow):
    def __init__(self, camera_subs):
        super().__init__()

        self.camera_subs = camera_subs
        self.setWindowTitle("The University of Iowa Robotics 2023")
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout()
        self.central_widget.setLayout(self.main_layout)

        # Set up camera feeds
        self.camera_layout = QHBoxLayout()
        self.main_layout.addLayout(self.camera_layout)
        self.camera_labels = [QLabel() for _ in range(3)]
        for label in self.camera_labels:
            self.camera_layout.addWidget(label)

        # Set up timer module
        self.timer_module = TimerModule()
        self.timer_label = QLabel("00:00")
        self.timer_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.timer_label)

        # Set up timer control buttons
        self.timer_control_layout = QHBoxLayout()
        self.main_layout.addLayout(self.timer_control_layout)
        self.start_button = QPushButton("Start")
        self.pause_button = QPushButton("Pause")
        self.reset_button = QPushButton("Reset")
        self.timer_control_layout.addWidget(self.start_button)
        self.timer_control_layout.addWidget(self.pause_button)
        self.timer_control_layout.addWidget(self.reset_button)

        # Connect timer control buttons to actions
        self.start_button.clicked.connect(self.timer_module.start)
        self.pause_button.clicked.connect(self.timer_module.pause)
        self.reset_button.clicked.connect(self.timer_module.reset)

        # Set up update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_gui)
        self.update_timer.start(30)

    def update_gui(self):
        for i, camera_sub in enumerate(self.camera_subs):
            if camera_sub.cv_image is not None:
                height, width, _ = camera_sub.cv_image.shape
                bytes_per_line = 3 * width
                q_image = QImage(camera_sub.cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(q_image.rgbSwapped())
                self.camera_labels[i].setPixmap(pixmap.scaled(self.camera_labels[i].size(), Qt.KeepAspectRatio))

        minutes, seconds = divmod(self.timer_module.time_elapsed, 60)
        self.timer_label.setText(f"{minutes:02d}:{seconds:02d}")

def main(args=None):
    rclpy.init(args=args)

    camera_topics = ['hand_aruco', 'chest_aruco', 'sky_aruco']
    camera_subs = [CameraSubscriber(topic, f'camera_subscriber_{i}') for i, topic in enumerate(camera_topics)]

    app = QApplication(sys.argv)
    main_window = RoverGUI(camera_subs)
    main_window.show()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(camera_subs[0])
    executor.add_node(camera_subs[1])
    executor.add_node(camera_subs[2])

    def rclpy_spin():
        while rclpy.ok():
            executor.spin_once()

    from threading import Thread
    spin_thread = Thread(target=rclpy_spin, daemon=True)
    spin_thread.start()

    sys.exit(app.exec_())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
