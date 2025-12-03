import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32  # masih boleh dipakai kalau mau log battery_voltage global
from my_msg.msg import Arus, Volt  # <-- import custom message
import csv
import os

LEG_JOINTS = [
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_ank_pitch", "r_ank_roll",
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_ank_pitch", "l_ank_roll"
]

# Mapping nama joint -> field di my_msg/Arus dan my_msg/Volt
JOINT_TO_ARUS_FIELD = {
    "r_hip_yaw":   "id7",
    "l_hip_yaw":   "id8",
    "r_hip_roll":  "id9",
    "l_hip_roll":  "id10",
    "r_hip_pitch": "id11",
    "l_hip_pitch": "id12",
    "r_ank_pitch": "id15",
    "l_ank_pitch": "id16",
    "r_ank_roll":  "id17",
    "l_ank_roll":  "id18",
}

JOINT_TO_VOLT_FIELD = {
    "r_hip_yaw":   "id7",
    "l_hip_yaw":   "id8",
    "r_hip_roll":  "id9",
    "l_hip_roll":  "id10",
    "r_hip_pitch": "id11",
    "l_hip_pitch": "id12",
    "r_ank_pitch": "id15",
    "l_ank_pitch": "id16",
    "r_ank_roll":  "id17",
    "l_ank_roll":  "id18",
}

# Konversi satuan current raw Dynamixel -> Ampere (XM series: 2.69 mA / LSB)
CURRENT_RAW_TO_AMP = 0.00269


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger_node')

        # Menyimpan message terakhir dari /robotis/arus dan /robotis/volt
        self.current_arus_msg = None
        self.current_volt_msg = None

        # (Opsional) menyimpan tegangan baterai global dari /robotis/battery_voltage
        self.battery_voltage = 12.0

        # QoS Profile untuk JointState dari Robot Hardware
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # 1. Subscriber Joint State (posisi & kecepatan)
        self.create_subscription(
            JointState,
            '/robotis/present_joint_states',
            self.joint_state_callback,
            qos_profile
        )

        # 2. Subscriber Arus per joint dari OpenCR
        self.create_subscription(
            Arus,
            '/robotis/arus',
            self.arus_callback,
            10
        )

        # 3. Subscriber Tegangan per joint dari OpenCR
        self.create_subscription(
            Volt,
            '/robotis/volt',
            self.volt_callback,
            10
        )

        # (Opsional) kalau masih ingin log tegangan baterai global
        self.create_subscription(
            Float32,
            '/robotis/battery_voltage',
            self.battery_voltage_callback,
            10
        )

        # Inisialisasi file CSV
        home_dir = os.path.expanduser('~')
        output_filename = os.path.join(home_dir, 'data_skripsi_berdiri.csv')

        try:
            self.csv_file = open(output_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # Header CSV
            self.csv_writer.writerow([
                'timestamp_sec',
                'timestamp_nanosec',
                'joint_name',
                'position_rad',
                'velocity_rad_s',
                'current_ampere',   # dari /robotis/arus (sudah dikonversi ke A)
                'voltage_volt',     # dari /robotis/volt (0.1 * raw)
                'power_watt',       # V * I per joint
                'battery_voltage'   # opsional: tegangan baterai global
            ])
            self.csv_file.flush()
            self.get_logger().info(f"Siap merekam Power! File: {output_filename}")

        except IOError as e:
            self.get_logger().error(f"Gagal membuka file CSV: {e}")
            self.csv_file = None
            self.csv_writer = None

    # ---------------------- CALLBACK TOPIC ---------------------- #

    def arus_callback(self, msg: Arus):
        """Simpan message arus terakhir dari /robotis/arus."""
        self.current_arus_msg = msg

    def volt_callback(self, msg: Volt):
        """Simpan message tegangan terakhir dari /robotis/volt."""
        self.current_volt_msg = msg

    def battery_voltage_callback(self, msg: Float32):
        """(Opsional) Simpan tegangan baterai global."""
        self.battery_voltage = msg.data

    # -------------------- HELPER GETTER ------------------------ #

    def get_current_for_joint(self, joint_name: str) -> float:
        """
        Ambil arus joint (dalam Ampere) berdasarkan joint_name
        dari message /robotis/arus.
        """
        if self.current_arus_msg is None:
            return 0.0

        field = JOINT_TO_ARUS_FIELD.get(joint_name, None)
        if field is None:
            return 0.0

        # Ambil nilai raw dari field idX
        raw_value = getattr(self.current_arus_msg, field, 0.0)
        # Konversi ke Ampere
        current_ampere = raw_value * CURRENT_RAW_TO_AMP
        return current_ampere

    def get_voltage_for_joint(self, joint_name: str) -> float:
        """
        Ambil tegangan joint (Volt) berdasarkan joint_name
        dari message /robotis/volt.
        """
        if self.current_volt_msg is None:
            return 0.0

        field = JOINT_TO_VOLT_FIELD.get(joint_name, None)
        if field is None:
            return 0.0

        voltage = getattr(self.current_volt_msg, field, 0.0)
        return voltage

    # ----------------- CALLBACK JOINT STATE --------------------- #

    def joint_state_callback(self, msg: JointState):
        if self.csv_writer is None:
            return

        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec

        try:
            data_recorded = False

            for i in range(len(msg.name)):
                joint_name = msg.name[i]

                if joint_name in LEG_JOINTS:
                    position = msg.position[i] if i < len(msg.position) else 0.0
                    velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0

                    # --- Ambil arus & tegangan dari topic /robotis/arus dan /robotis/volt ---
                    current = self.get_current_for_joint(joint_name)   # Ampere
                    voltage = self.get_voltage_for_joint(joint_name)   # Volt

                    # --- Hitung daya per joint ---
                    power = voltage * abs(current)  # W (asumsi konsumsi -> nilai positif)

                    self.csv_writer.writerow([
                        stamp_sec,
                        stamp_nanosec,
                        joint_name,
                        position,
                        velocity,
                        current,
                        voltage,
                        power,
                        self.battery_voltage  # tegangan baterai global (opsional)
                    ])
                    data_recorded = True

            if data_recorded:
                self.csv_file.flush()

        except Exception as e:
            self.get_logger().error(f"Error saat menulis data: {e}")

    # ------------------- SHUTDOWN HANDLER ----------------------- #

    def on_shutdown(self):
        if hasattr(self, 'csv_file') and self.csv_file:
            self.get_logger().info("Menutup file CSV...")
            try:
                self.csv_file.flush()
            except Exception:
                pass
            self.csv_file.close()
            self.get_logger().info("File CSV ditutup.")

# -------------------------- MAIN ------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
