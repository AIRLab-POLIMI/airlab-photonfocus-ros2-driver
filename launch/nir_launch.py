from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airlab-photonfocus-ros2-wrapper',
            executable='image_publisher_node',
            name='nir_photonfocus',
            namespace='nir',
            parameters=[
                {'topic': '/nir_image_raw',
                 'frame_id': 'nir_camera_link',
                 'ip_address': '10.79.2.145',
                 'config_file_path': '/home/airlab/ros2_ws/src/airlab-photonfocus-ros2-wrapper/config/nir_camera.yaml'}
            ]
        )
    ])
