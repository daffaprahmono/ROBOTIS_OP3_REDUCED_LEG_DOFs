from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zmp',        # nama package
            executable='zmp',   # sesuai entry_point di setup.py
            name='zmp_cecep_node',    # opsional
            output='screen'             # log ditampilkan ke terminal
        )
    ])