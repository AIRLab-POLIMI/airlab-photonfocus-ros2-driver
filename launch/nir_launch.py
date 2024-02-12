"""
=====================================================================================================================
File: photonfocus_camera.py
Author: Mirko Usuelli (Ph.D. Candidate, Politecnico di Milano @ AIRLab)
Email: mirko.usuell@polimi.it
Description: This file contains the launch file to start the PhotonFocus camera driver for the NIR camera.
---------------------------------------------------------------------------------------------------------------------
Created on: 05/02/2024
Last Modified: 12/02/2024
=====================================================================================================================
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# Launch the PhotonFocus camera driver for NIR images
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airlab-photonfocus-ros2-driver',
            executable='photonfocus_camera_node',
            name='nir_photonfocus',
            namespace='nir',
            parameters=[
                {'topic': '/nir_image_raw',
                 'frame_id': 'nir_camera_link',
                 'ip_address': '10.79.2.145',
                 'config_file_path': '/home/airlab/ros2_ws/src/airlab-photonfocus-ros2-driver/config/nir_camera.yaml'}
            ]
        )
    ])
