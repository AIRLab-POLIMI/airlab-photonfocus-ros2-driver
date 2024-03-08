"""
=====================================================================================================================
File: prisma.launch.py
Author: Mirko Usuelli (Ph.D. Candidate, Politecnico di Milano @ AIRLab)
Email: mirko.usuell@polimi.it
Description: This file contains the launch file to start the PhotonFocus camera driver for the prisma.
---------------------------------------------------------------------------------------------------------------------
Created on: 27/02/2024
Last Modified: 27/02/2024
=====================================================================================================================
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# Launch the Prisma
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airlab_photonfocus_ros2_driver',
            executable='photonfocus_prisma_node',
            name='prisma_photonfocus',
            namespace='prisma',
            parameters=[
                {'vis_topic': '/vis_image_raw',
                 'vis_frame_id': 'vis_camera_link',
                 'nir_topic': '/nir_image_raw',
                 'nir_frame_id': 'vis_camera_link',}
            ]
        )
    ])
    