#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from robotis_controller_msgs.srv import SetModule

import time
import threading


class ActionSequencer(Node):
    def __init__(self):
        super().__init__('action_sequencer')

        self.publisher_ = self.create_publisher(Int32, '/robotis/action/page_num', 10)
        self.set_module_client = self.create_client(SetModule, '/robotis/set_present_ctrl_modules')

        self.declare_parameter('walk_duration', 10.0)
        self.declare_parameter('step_interval', 0.05)   # interval kecil
        self.declare_parameter('start_page', 90)
        self.declare_parameter('stop_page', 91)
        self.declare_parameter('warmup_sec', 2.0)
        self.declare_parameter('post_module_wait_sec', 2.0)
        self.declare_parameter('stop_repeat', 3)
        self.declare_parameter('stop_repeat_interval', 0.2)

        self._done = False
        self._failed = False

        # timer untuk shutdown rapi (dipanggil di main thread)
        self.create_timer(0.2, self._shutdown_if_done)

        self.get_logger().info("ActionSequencer siap. Menjalankan sekuens...")

        th = threading.Thread(target=self.run_sequence, daemon=True)
        th.start()

    def _shutdown_if_done(self):
        if self._done:
            self.get_logger().info("ActionSequencer selesai. Shutdown rapi.")
            # ✅ shutdown hanya sekali, dari main thread
            if rclpy.ok():
                rclpy.shutdown()

    def set_module(self, module_name: str) -> bool:
        if not self.set_module_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /robotis/set_present_ctrl_modules tidak tersedia.")
            return False

        req = SetModule.Request()
        req.module_name = module_name

        future = self.set_module_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error("Gagal set module (timeout / no response).")
            return False

        self.get_logger().info(f"Module aktif: {module_name}")
        return True

    def run_sequence(self):
        warmup = float(self.get_parameter('warmup_sec').value)
        post_wait = float(self.get_parameter('post_module_wait_sec').value)

        duration = float(self.get_parameter('walk_duration').value)
        step_interval = float(self.get_parameter('step_interval').value)

        start_page = int(self.get_parameter('start_page').value)
        stop_page = int(self.get_parameter('stop_page').value)

        stop_repeat = int(self.get_parameter('stop_repeat').value)
        stop_repeat_interval = float(self.get_parameter('stop_repeat_interval').value)

        time.sleep(warmup)

        ok = self.set_module('action_module')
        if not ok:
            self._failed = True
            self._done = True
            return

        time.sleep(post_wait)

        self.get_logger().info(
            f"Mulai jalan: page={start_page}, durasi={duration:.2f}s, interval={step_interval:.3f}s"
        )

        start_t = time.time()
        msg = Int32()
        msg.data = start_page

        while rclpy.ok() and (time.time() - start_t) < duration:
            self.publisher_.publish(msg)
            if step_interval > 0:
                time.sleep(step_interval)

        self.get_logger().info(f"Selesai durasi. Kirim stop page={stop_page} (repeat={stop_repeat}x).")

        stop_msg = Int32()
        stop_msg.data = stop_page
        for _ in range(max(stop_repeat, 1)):
            self.publisher_.publish(stop_msg)
            time.sleep(stop_repeat_interval)

        self._done = True


def main(args=None):
    rclpy.init(args=args)
    node = ActionSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
