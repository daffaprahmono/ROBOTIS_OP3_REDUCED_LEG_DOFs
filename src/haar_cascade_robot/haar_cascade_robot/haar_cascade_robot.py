import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
import os
import numpy as np


class HaarRobotDetector(Node):
    def __init__(self):
        super().__init__('haar_robot_detector')

        self.get_logger().info("=== Node Deteksi Robot OP3 (Haar Cascade + CLAHE + ROI + Gamma) ===")

        # === Path model Haar Cascade ===
        cascade_path = os.path.expanduser("~/robotis_ws/src/haar_cascade_robot/haar_cascade_robot/cascade.xml")
        self.robot_cascade = cv2.CascadeClassifier(cascade_path)
        if self.robot_cascade.empty():
            self.get_logger().error(f"Gagal memuat model Haar Cascade di: {cascade_path}")
            raise SystemExit

        # === Bridge untuk konversi ROS Image <-> OpenCV ===
        self.bridge = CvBridge()

        # === Subscribe ke topic kamera (misal dari usb_cam) ===
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # === Publisher status deteksi ===
        self.pub_status = self.create_publisher(String, '/robot_detection', 10)

        # === Variabel FPS ===
        self.start_time = time.time()
        self.frame_count = 0

        # === Inisialisasi CLAHE ===
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        self.get_logger().info("Node aktif dan menunggu feed dari /image_raw ...")

    def preprocess_image(self, frame):
        """Konversi, CLAHE, dan gamma correction"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # CLAHE untuk menyesuaikan kontras
        gray = self.clahe.apply(gray)

        # Gamma correction agar objek gelap terlihat lebih jelas
        gamma = 1.3
        gray = np.power(gray / 255.0, 1.0 / gamma)
        gray = np.uint8(gray * 255)

        return gray

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocessing
        gray = self.preprocess_image(frame)

        # === Gunakan ROI di bagian bawah frame (tempat robot berada) ===
        height, width = gray.shape
        roi_y_start = int(height * 0.4)
        roi = gray[roi_y_start:height, 0:width]

        # === Deteksi Haar Cascade ===
        robots = self.robot_cascade.detectMultiScale(
            roi,
            scaleFactor=1.05,   # deteksi lebih halus
            minNeighbors=8,     # kompromi antara recall dan precision
            minSize=(60, 60)    # bisa mendeteksi robot kecil
        )

        # FPS
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0.0

        status = "Robot Tidak Terdeteksi"
        color = (0, 0, 255)

        # === Gambarkan hasil deteksi ===
        if len(robots) > 0:
            status = "Robot OP3 Terdeteksi"
            color = (0, 255, 0)
            for (x, y, w, h) in robots:
                # Offset karena ROI dimulai dari tengah ke bawah
                y += roi_y_start
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

        # === Overlay teks ===
        cv2.putText(frame, status, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # === Tampilkan frame hasil deteksi ===
        cv2.imshow("Deteksi Robot Humanoid OP3", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Keluar dari tampilan video.")
            rclpy.shutdown()

        # === Publish status ke ROS topic ===
        msg_out = String()
        msg_out.data = status
        self.pub_status.publish(msg_out)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HaarRobotDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node dihentikan oleh pengguna.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
