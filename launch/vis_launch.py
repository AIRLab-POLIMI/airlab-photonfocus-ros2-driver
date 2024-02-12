"""
=====================================================================================================================
File: photonfocus_camera.py
Author: Mirko Usuelli (Ph.D. Candidate, Politecnico di Milano @ AIRLab)
Email: mirko.usuell@polimi.it
Description: This file contains the launch file to start the PhotonFocus camera driver for the VIS camera.
---------------------------------------------------------------------------------------------------------------------
Created on: 05/02/2024
Last Modified: 12/02/2024
=====================================================================================================================
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# Launch the PhotonFocus camera driver for the VIS camera
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airlab-photonfocus-ros2-wrapper',
            executable='photonfocus_camera_node',
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