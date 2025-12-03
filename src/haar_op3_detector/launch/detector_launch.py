from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_path = os.path.join(
        os.getenv('HOME'),
        'robotis_ws/src/haar_op3_detector'
    )

    cascade_path = os.path.join(pkg_path, 'config', 'cascade.xml')
    cam_yaml     = os.path.join(pkg_path, 'config', 'camera_params.yaml')

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[cam_yaml],
        output='screen'
    )

    detector_node = Node(
        package='haar_op3_detector',
        executable='detector_node',
        name='haar_op3_detector_node',
        parameters=[{
            'image_topic': '/image_raw',
            'cascade_path': cascade_path,
            'scale_factor': 1.05,
            'min_neighbors': 8,
            'min_size': 80
        }],
        output='screen'
    )

    return LaunchDescription([
        usb_cam_node,
        detector_node
    ])
