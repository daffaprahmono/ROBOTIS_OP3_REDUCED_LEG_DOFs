import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from .imu_ukf import ukf_denoise
import numpy as np
import time
from my_msg.msg import Data   # custom message

class DynamicComCalculator(Node):
    def __init__(self):
        super().__init__('zmp_node')
        self.publisher_ = self.create_publisher(Data, "zmp", 10)
        self.get_logger().info("ZMP node started.")

        # Data sensor
        self.accel = None
        self.orientation = None
        self.gyro_raw = None
        self.joint_positions = None
        self.previous_time = None

        # Subscriptions
        self.create_subscription(Imu, "/robotis/open_cr/imu", self.imu_callback, 10)
        self.create_subscription(JointState, "/robotis/present_joint_states", self.joint_state_callback, 10)

        # Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Servo data (dipertahankan sesuai punyamu)
        self.servo_data = [
            {"id": 20, "mass": 0.09, "CoM_offset": (0, 2.85, 45.1)},
            {"id": 3, "mass": 0.079, "CoM_offset": (-10.8, 1.3, 37.5)},
            {"id": 4, "mass": 0.079, "CoM_offset": (10.8, 1.3, 37.5)},
            {"id": 5, "mass": 0.031, "CoM_offset": (-23.4, 0.9, 37.5)},
            {"id": 6, "mass": 0.031, "CoM_offset": (23.4, 0.9, 37.5)},
            {"id": 1, "mass": 0.086, "CoM_offset": (-4.5, 1.8, 35.2)},
            {"id": 2, "mass": 0.086, "CoM_offset": (4.5, 1.8, 35.2)},
            {"id": 19, "mass": 1.179, "CoM_offset": (0, 1.8, 35.2)},
            {"id": 9, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 23.5)},
            {"id": 11, "mass": 0.164, "CoM_offset": (-3.2, 1, 23.5)},
            {"id": 10, "mass": 0.082, "CoM_offset": (3.2, -2.7, 23.5)},
            {"id": 12, "mass": 0.246, "CoM_offset": (3.2, 1, 23.5)},
            {"id": 13, "mass": 0.139, "CoM_offset": (-3.2, 0.7, 16.4)},
            {"id": 14, "mass": 0.139, "CoM_offset": (3.2, 0.7, 16.4)},
            {"id": 17, "mass": 0.082, "CoM_offset": (-3.2, -2.7, 3.7)},
            {"id": 18, "mass": 0.082, "CoM_offset": (3.2, -2.7, 3.7)},
            {"id": 15, "mass": 0.282, "CoM_offset": (-3.2, 0.4, 3.7)},
            {"id": 16, "mass": 0.282, "CoM_offset": (3.2, 0.4, 3.7)},
            {"id": 7, "mass": 0.086, "CoM_offset": (-4.5, 1.8, 28.4)},
            {"id": 8, "mass": 0.086, "CoM_offset": (4.5, 1.8, 28.4)}
        ]


        self.id_to_index_map = {
            1: 18, 2: 9, 3: 19, 4: 10, 5: 13, 6: 4, 7: 16, 8: 7,
            9: 15, 10: 6, 11: 14, 12: 5, 13: 17, 14: 8, 15: 11,
            16: 2, 17: 12, 18: 3, 19: 1, 20: 0,
        }

        self.FOOT_BASE_WIDTH = 0.08
        self.FOOT_BASE_LENGTH = 0.123

    def imu_callback(self, msg):
        self.accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.orientation = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        self.gyro_raw = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def joint_state_callback(self, msg):
        self.joint_positions = msg.position

    def quaternion_to_rotation_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
        ])

    def get_filtered_gyro(self, gyro_raw):
        process_noise = np.eye(3) * 0.1
        measurement_noise = np.eye(3) * 0.1
        dt = 0.01
        denoised_gyro = ukf_denoise(gyro_raw, process_noise, measurement_noise, dt)
        return denoised_gyro[-1]

    def calculate_dynamic_com(self):
        if self.accel is None or self.orientation is None or self.joint_positions is None or self.gyro_raw is None:
            return None

        total_mass = 0
        weighted_sum = np.array([0.0, 0.0, 0.0])
        gyro = self.get_filtered_gyro(self.gyro_raw)
        rotation_matrix = self.quaternion_to_rotation_matrix(self.orientation)

        current_time = time.time()
        delta_time = current_time - self.previous_time if self.previous_time else 0
        self.previous_time = current_time

        if delta_time > 0:
            rotation_adjustment = np.identity(3) + np.cross(np.identity(3), self.gyro_raw) * delta_time
        else:
            rotation_adjustment = np.identity(3)

        adjusted_rotation_matrix = np.dot(rotation_matrix, rotation_adjustment)

        for servo in self.servo_data:
            mass = servo["mass"]
            offset = np.array(servo["CoM_offset"])
            joint_index = self.id_to_index_map.get(servo["id"])
            joint_pos = self.joint_positions[joint_index] if joint_index is not None and joint_index < len(self.joint_positions) else 0

            local_position = offset + np.array([0, 0, joint_pos])
            transformed_position = adjusted_rotation_matrix.dot(local_position)
            weighted_sum += transformed_position * mass
            total_mass += mass

        CoM_x, CoM_y, CoM_z = weighted_sum / total_mass
        return CoM_x, CoM_y, CoM_z

    def calculate_dynamic_zmp(self, CoM_x, CoM_y, CoM_z):
        if self.accel is None:
            return None

        GRAVITY = 9.81
        F_z = self.accel[2] - GRAVITY
        total_mass = sum(servo["mass"] for servo in self.servo_data)
        F_normal = total_mass * GRAVITY
        mu = 0.5
        F_friction = mu * F_normal

        if abs(F_z) < 1e-5:
            return None

        moment_x = F_friction * (self.FOOT_BASE_WIDTH / 2)
        moment_y = F_friction * (self.FOOT_BASE_LENGTH / 2)

        ZMP_x = CoM_x - (CoM_z / F_normal) * (self.accel[0] + moment_y / F_normal)
        ZMP_y = CoM_y - (CoM_z / F_normal) * (self.accel[1] - moment_x / F_normal)

        return ZMP_x, ZMP_y

    def calculate_support_polygon(self, joint_states, imu_accel_z):
        l_ank_pitch = np.array([state[self.id_to_index_map[13]] for state in joint_states])
        r_ank_pitch = np.array([state[self.id_to_index_map[14]] for state in joint_states])

        if l_ank_pitch[0] > 0 and r_ank_pitch[0] > 0:
            current_phase = 'Double Support'
        elif l_ank_pitch[0] <= 0 and r_ank_pitch[0] <= 0:
            current_phase = 'Single Support'
        else:
            current_phase = 'Single Support' if imu_accel_z < -0.35 else 'Double Support'

        points = np.array([
            [0, 0],
            [0, -12.3],
            [1.1, -13],
            [16.6, -13] if current_phase == 'Double Support' else [7.1, -13],
            [17.7, -12.3] if current_phase == 'Double Support' else [8.7, -12.3],
            [17.7, 0] if current_phase == 'Double Support' else [8.7, 0]
        ])

        x = points[:, 0]
        y = points[:, 1]
        area = 0.5 * np.abs(np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:]))
        perimeter = np.sum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))

        return {"phase": current_phase, "area": area, "perimeter": perimeter}

    def timer_callback(self):
        com_data = self.calculate_dynamic_com()
        if not com_data:
            return

        CoM_x, CoM_y, CoM_z = com_data
        zmp_data = self.calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)
        if not zmp_data:
            return

        ZMP_x, ZMP_y = zmp_data

        support_polygon_data = None
        if self.joint_positions is not None and self.accel is not None:
            imu_accel_z = self.accel[1]
            joint_states = [self.joint_positions]
            support_polygon_data = self.calculate_support_polygon(joint_states, imu_accel_z)

        # Publish custom message
        msg = Data()
        msg.zmp_x = ZMP_x
        msg.zmp_y = ZMP_y
        msg.com_x = CoM_x
        msg.com_y = CoM_y
        msg.com_z = CoM_z
        msg.area = support_polygon_data["area"] if support_polygon_data else 0.0
        msg.perimeter = support_polygon_data["perimeter"] if support_polygon_data else 0.0


        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicComCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()