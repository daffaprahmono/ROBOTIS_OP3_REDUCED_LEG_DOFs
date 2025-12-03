#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

class HaarOP3Detector(Node):
    def __init__(self):
        super().__init__('haar_op3_detector_node')

        # ==== Parameter ====
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('cascade_path', '')
        self.declare_parameter('scale_factor', 1.05)
        self.declare_parameter('min_neighbors', 8)
        self.declare_parameter('min_size', 80)

        image_topic  = self.get_parameter('image_topic').value
        cascade_path = self.get_parameter('cascade_path').value
        self.scale_factor  = float(self.get_parameter('scale_factor').value)
        self.min_neighbors = int(self.get_parameter('min_neighbors').value)
        self.min_size      = int(self.get_parameter('min_size').value)

        # ==== Load cascade ====
        if cascade_path == '' or not os.path.exists(cascade_path):
            raise RuntimeError(f'cascade_path tidak valid: {cascade_path}')

        self.robot_cascade = cv2.CascadeClassifier(cascade_path)
        if self.robot_cascade.empty():
            raise RuntimeError('Gagal load cascade.xml')

        # ==== Subscriber ====
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        self.get_logger().info(f"Subscribe image dari: {image_topic}")
        self.get_logger().info(f"Model cascade: {cascade_path}")

        # FPS
        self.start_time = time.time()
        self.frame_count = 0

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # (Optional) Preprocessing ringan agar mirip Windows
        # gray = cv2.equalizeHist(gray)

        robots = self.robot_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(self.min_size, self.min_size)
        )

        # FPS
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0

        # Draw hasil
        if len(robots) > 0:
            status_text = "Robot OP3 Terdeteksi"
            color = (0, 255, 0)
            for (x, y, w, h) in robots:
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
        else:
            status_text = "Robot Tidak Terdeteksi"
            color = (0, 0, 255)

        cv2.putText(frame, status_text, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Deteksi OP3 ROS2", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HaarOP3Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
