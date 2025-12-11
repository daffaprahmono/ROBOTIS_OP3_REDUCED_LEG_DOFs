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

        # ==== Runtime FPS ====
        self.start_time = time.time()
        self.frame_count = 0

        # ==== Pengujian Waktu Komputasi (dalam detik) ====
        self.test_frames = 20           # jumlah frame uji
        self.time_list = []             # menyimpan waktu tiap frame (detik)
        self.measured_frames = 0
        self.measurement_done = False

        # Nama file output
        self.csv_filename = "waktu_komputasi_haar.csv"

        self.get_logger().info(
            f"Pengujian waktu komputasi (detik) aktif, frame uji = {self.test_frames}"
        )
        self.get_logger().info(
            f"Hasil pengujian akan disimpan ke: {self.csv_filename}"
        )

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # =========== PENGUKURAN WAKTU KOMPUTASI (DETIK) ==========
        start_time = time.perf_counter()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        robots = self.robot_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(self.min_size, self.min_size)
        )

        end_time = time.perf_counter()
        detection_time_s = (end_time - start_time)   # satuan detik

        # Simpan waktu detik untuk 20 frame pertama
        if self.measured_frames < self.test_frames:
            self.measured_frames += 1
            self.time_list.append(detection_time_s)
            self.get_logger().info(
                f"[Waktu Komputasi] Frame {self.measured_frames}: "
                f"{detection_time_s:.6f} detik"
            )

        # Setelah selesai 20 frame → hitung statistik
        if (self.measured_frames == self.test_frames) and (not self.measurement_done):

            avg_time = sum(self.time_list) / len(self.time_list)
            min_time = min(self.time_list)
            max_time = max(self.time_list)

            variance = sum((t - avg_time) ** 2 for t in self.time_list) / len(self.time_list)
            std_time = variance ** 0.5

            fps_from_time = 1.0 / avg_time if avg_time > 0 else 0.0

            self.get_logger().info("===== HASIL PENGUJIAN WAKTU KOMPUTASI (DETIK) =====")
            self.get_logger().info(f"Jumlah frame uji      : {self.test_frames}")
            self.get_logger().info(f"Rata-rata waktu (s)   : {avg_time:.6f}")
            self.get_logger().info(f"Standar deviasi (s)   : {std_time:.6f}")
            self.get_logger().info(f"Waktu minimum (s)     : {min_time:.6f}")
            self.get_logger().info(f"Waktu maksimum (s)    : {max_time:.6f}")
            self.get_logger().info(f"FPS estimasi          : {fps_from_time:.2f} FPS")
            self.get_logger().info("=======================================================")

            # ============ SIMPAN KE CSV (DETIK) ============
            try:
                with open(self.csv_filename, "w") as f:
                    f.write("frame,waktu_detik\n")
                    for i, t in enumerate(self.time_list, start=1):
                        f.write(f"{i},{t:.6f}\n")

                    f.write("\n")
                    f.write(f"summary_avg_s,{avg_time:.6f}\n")
                    f.write(f"summary_std_s,{std_time:.6f}\n")
                    f.write(f"summary_min_s,{min_time:.6f}\n")
                    f.write(f"summary_max_s,{max_time:.6f}\n")
                    f.write(f"summary_fps,{fps_from_time:.6f}\n")

                self.get_logger().info(
                    f"Hasil pengujian berhasil disimpan ke {self.csv_filename}"
                )

            except Exception as e:
                self.get_logger().error(
                    f"Gagal menyimpan CSV {self.csv_filename}: {e}"
                )

            self.measurement_done = True

        # =========== HITUNG FPS RUNTIME (UNTUK DITAMPILKAN DI FRAME) ==========
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps_runtime = self.frame_count / elapsed if elapsed > 0 else 0.0

        # ====== Gambar hasil deteksi ======
        if len(robots) > 0:
            status_text = "Robot OP3 Terdeteksi"
            color = (0, 255, 0)
            for (x, y, w, h) in robots:
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        else:
            status_text = "Robot Tidak Terdeteksi"
            color = (0, 0, 255)

        cv2.putText(frame, status_text, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        cv2.putText(frame, f"FPS: {fps_runtime:.2f}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)

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
