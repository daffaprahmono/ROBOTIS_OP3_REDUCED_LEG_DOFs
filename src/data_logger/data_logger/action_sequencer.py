import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from robotis_controller_msgs.srv import SetModule # <--- Import Service Message
import time
import threading
import sys

class ActionSequencer(Node):
    def __init__(self):
        super().__init__('action_sequencer')
        
        # Publisher untuk perintah Page (Jalan/Berdiri)
        self.publisher_ = self.create_publisher(Int32, '/robotis/action/page_num', 10)
        
        # Client untuk mengaktifkan Action Module
        self.set_module_client = self.create_client(SetModule, '/robotis/set_present_ctrl_modules')
        
        # Parameter durasi
        self.declare_parameter('walk_duration', 10.0) 
        self.declare_parameter('step_interval', 0.32) # Default sesuai hitungan kita tadi
        
        self.get_logger().info('Sequencer siap. Menunggu setup...')
        
        # Jalankan di thread terpisah
        thread = threading.Thread(target=self.run_sequence)
        thread.start()

    def set_module(self, module_name):
        """Fungsi helper buat ganti modul (base_module -> action_module)"""
        if not self.set_module_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service /robotis/set_present_ctrl_modules tidak tersedia!')
            return False
            
        request = SetModule.Request()
        request.module_name = module_name
        
        future = self.set_module_client.call_async(request)
        # Kita tunggu sebentar tapi manual biar ga ngeblock spin
        time.sleep(0.5) 
        self.get_logger().info(f'Mengaktifkan modul: {module_name}')
        return True

    def run_sequence(self):
        # 1. Warm Up & Enable Action Module
        time.sleep(2.0)
        
        # --- BAGIAN PENTING: AKTIFKAN ACTION MODULE ---
        self.set_module('action_module')
        time.sleep(2.0) # Kasih waktu robot "siap-siap" (torque on adjust)
        
        # Ambil parameter
        duration = self.get_parameter('walk_duration').value
        step_delay = self.get_parameter('step_interval').value
        
        self.get_logger().info(f'MULAI JALAN! (Looping {duration} detik)')
        start_time = time.time()
        
        # 2. Loop Jalan (Page 90)
        while rclpy.ok() and (time.time() - start_time < duration):
            msg = Int32()
            msg.data = 90
            self.publisher_.publish(msg)
            
            # Info sisa waktu
            sisa = int(duration - (time.time() - start_time))
            self.get_logger().info(f'Step... (Sisa: {sisa}s)')
            # time.sleep(step_delay)

        # 3. Stop / Berdiri (Page 91)
        self.get_logger().info('WAKTU HABIS! Mengirim perintah BERDIRI (Page 91).')
        stop_msg = Int32()
        stop_msg.data = 91
        
        # Kirim beberapa kali biar yakin masuk
        for _ in range(3):
            self.publisher_.publish(stop_msg)
            time.sleep(0.5)
            
        self.get_logger().info('Selesai. Mematikan node...')
        
        # Opsional: Kembalikan ke 'none' atau biarkan saja
        # self.set_module('none') 
        
        # Matikan node
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = ActionSequencer()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('sequencer').info('Node berhenti.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()