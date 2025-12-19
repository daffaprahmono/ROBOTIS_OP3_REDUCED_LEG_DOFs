#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from my_msg.msg import Arus, Volt

import csv
import os
from datetime import datetime
from zoneinfo import ZoneInfo


LEG_JOINTS = [
    "r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_ank_pitch", "r_ank_roll",
    "l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_ank_pitch", "l_ank_roll"
]

JOINT_TO_FIELD = {
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

DEFAULT_CURRENT_SCALE = 0.00269


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        # Params
        self.declare_parameter('output_dir', os.path.join(os.path.expanduser('~'), 'data_pengujian'))
        self.declare_parameter('output_filename', '')
        self.declare_parameter('log_hz', 10.0)
        self.declare_parameter('flush_every_n', 50)
        self.declare_parameter('include_battery', True)
        self.declare_parameter('current_scale', DEFAULT_CURRENT_SCALE)
        self.declare_parameter('volt_scale', 0.1)  # kamu pakai 0.1

        output_dir = self.get_parameter('output_dir').value
        output_dir = os.path.expanduser(str(output_dir))  # ✅ expand "~"
        output_filename = str(self.get_parameter('output_filename').value)

        self.log_hz = float(self.get_parameter('log_hz').value)
        self.flush_every_n = int(self.get_parameter('flush_every_n').value)
        self.include_battery = bool(self.get_parameter('include_battery').value)

        self.current_scale = float(self.get_parameter('current_scale').value)
        self.volt_scale = float(self.get_parameter('volt_scale').value)

        self.log_period = (1.0 / self.log_hz) if self.log_hz > 0 else 0.0

        # Cache
        self.arus_msg = None
        self.volt_msg = None
        self.have_current = False
        self.have_voltage = False

        self.battery_voltage = 0.0
        self.have_battery = False

        self.last_log_time_epoch = None
        self.t0_epoch = None

        self.tz = ZoneInfo("Asia/Jakarta")
        self.row_count = 0

        # QoS
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(JointState, '/robotis/present_joint_states', self.joint_state_callback, qos_profile)
        self.create_subscription(Arus, '/robotis/arus', self.arus_callback, 10)
        self.create_subscription(Volt, '/robotis/volt', self.volt_callback, 10)

        if self.include_battery:
            self.create_subscription(Float32, '/robotis/battery_voltage', self.battery_callback, 10)

        # CSV
        os.makedirs(output_dir, exist_ok=True)

        if output_filename:
            csv_path = os.path.join(output_dir, output_filename)
        else:
            ts = datetime.now(self.tz).strftime('%Y%m%d-%H%M%S')
            csv_path = os.path.join(output_dir, f"data_power_{ts}.csv")

        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        header = [
            't_rel_s', 'time_iso_wib',
            'timestamp_sec', 'timestamp_nanosec',
            'joint_name', 'position_rad', 'velocity_rad_s',
            'current_ampere', 'voltage_volt', 'power_watt'
        ]
        if self.include_battery:
            header.append('battery_voltage')

        self.csv_writer.writerow(header)
        self.csv_file.flush()

        self.get_logger().info(f"Data logger siap. Output: {csv_path}")
        self.get_logger().info(f"Downsampling aktif: log_hz={self.log_hz} Hz (period={self.log_period:.3f}s)")

    def arus_callback(self, msg: Arus):
        self.arus_msg = msg
        self.have_current = True

    def volt_callback(self, msg: Volt):
        self.volt_msg = msg
        self.have_voltage = True

    def battery_callback(self, msg: Float32):
        self.battery_voltage = float(msg.data)
        self.have_battery = True

    def _get_from_msg(self, msg, joint_name: str) -> float:
        field = JOINT_TO_FIELD.get(joint_name)
        if (msg is None) or (field is None):
            return 0.0
        return float(getattr(msg, field, 0.0))

    def joint_state_callback(self, msg: JointState):
        if not (self.have_current and self.have_voltage):
            return

        stamp = msg.header.stamp
        t_epoch = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        if self.t0_epoch is None:
            self.t0_epoch = t_epoch

        if self.last_log_time_epoch is not None and self.log_period > 0:
            if (t_epoch - self.last_log_time_epoch) < self.log_period:
                return

        self.last_log_time_epoch = t_epoch
        t_rel = t_epoch - self.t0_epoch
        time_iso_wib = datetime.fromtimestamp(t_epoch, tz=self.tz).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " WIB"

        names = msg.name
        pos = msg.position
        vel = msg.velocity

        for i, joint in enumerate(names):
            if joint not in LEG_JOINTS:
                continue

            position = float(pos[i]) if i < len(pos) else 0.0
            velocity = float(vel[i]) if i < len(vel) else 0.0

            raw_i = self._get_from_msg(self.arus_msg, joint)
            raw_v = self._get_from_msg(self.volt_msg, joint)

            current_a = raw_i * self.current_scale
            voltage_v = raw_v * self.volt_scale
            power_w = abs(current_a) * voltage_v

            row = [
                t_rel, time_iso_wib,
                stamp.sec, stamp.nanosec,
                joint, position, velocity,
                current_a, voltage_v, power_w
            ]
            if self.include_battery:
                row.append(self.battery_voltage if self.have_battery else 0.0)

            self.csv_writer.writerow(row)
            self.row_count += 1

            if self.flush_every_n > 0 and (self.row_count % self.flush_every_n == 0):
                self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ✅ jangan shutdown kalau sudah mati
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
