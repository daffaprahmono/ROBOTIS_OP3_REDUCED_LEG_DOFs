from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Jalankan Data Logger
        Node(
            package='data_logger',
            executable='data_logger_node',
            name='data_logger_manager',
            output='screen'
        ),

        # 2. Jalankan Action Sequencer
        Node(
            package='data_logger',
            executable='action_sequencer',
            name='walking_manager',
            output='screen',
            parameters=[
                {'walk_duration': 10.0}, # Kamu bisa ubah durasi di sini
                {'step_interval': 0.42}   # Sesuaikan dengan kecepatan langkah robot
            ]
        ),
    ])