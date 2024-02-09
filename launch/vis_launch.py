from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airlab-photonfocus-ros2-wrapper',
            executable='image_publisher_node',
            name='vis_photonfocus',
            namespace='vis',
            parameters=[
                {'topic': '/vis_image_raw',
                 'frame_id': 'vis_camera_link',
                 'ip_address': '10.79.2.78',
                 'config_file_path': '/home/airlab/ros2_ws/src/airlab-photonfocus-ros2-wrapper/config/vis_camera.yaml'}
            ]
        )
    ])