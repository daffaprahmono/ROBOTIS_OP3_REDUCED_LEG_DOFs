from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path package untuk mencari file konfigurasi jika ada
    haar_pkg_path = FindPackageShare('haar_cascade_robot')
    camera_param_path = PathJoinSubstitution([haar_pkg_path, 'config', 'camera_param.yaml'])

    return LaunchDescription([
        # Node USB camera
        Node(
            package='usb_cam',                 # package usb_cam
            executable='usb_cam_node_exe',    # executable usb_cam_node_exe
            name='usb_cam_node',
            output='screen',
            parameters=[camera_param_path],    # optional, jika ada file config
        ),

        # Node Haar cascade robot
        Node(
            package='haar_cascade_robot',
            executable='fifin',
            name='haar_cascade_robot',
            output='screen',
        )
    ])
