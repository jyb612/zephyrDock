#!/usr/bin/env python
# # DISCLAIMER # #

# THIS LAUNCH BOTH CAMERA #
# THE CAMERA 'video_device' NEEDS TO BE CHECKED IF REFERRING THE RIGHT ONE #
# IMPORTANT #
# MIGHT BE MISTAKEN AS DEPTH CAMERA, WHILE THESE ARE FOR AR0234 GLOBAL SHUTTER CAMERA #

# CAMERA CALIBRATION FILE (.yaml) IN /calcam #








from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='color_camera',
            parameters=[
                {
                    'video_device': '/dev/video0',
                    'pixel_format': 'YUYV',
                    'output_encoding': 'yuv422_yuy2',
                    'image_size': [640,480],
                    'fps': 30,
                    'use_image_transport': False,
                    'camera_info_url': 'file:///home/crestjj/zephyrDock/src/ros2_v4l2_camera/config/calcam_color.yaml'
                }
            ],
            remappings=[
                ('image_raw/compressed', '/dev/null'),
                ('image_raw/compressedDepth', '/dev/null'),
                ('image_raw/theora', '/dev/null'),
            ]
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='bnw_camera',
            parameters=[
                {
                    'video_device': '/dev/video8',
                    'pixel_format': 'YUYV',
                    'output_encoding': 'yuv422_yuy2',
                    'image_size': [640,480],
                    'fps': 30,
                    'use_image_transport': False,
                    'saturation': 0,  # Force grayscale
                    'camera_info_url': 'file:///home/crestjj/zephyrDock/src/ros2_v4l2_camera/config/calcam_bnw.yaml'
                }
            ],
            remappings=[
                ('image_raw/compressed', '/dev/null'),
                ('image_raw/compressedDepth', '/dev/null'),
                ('image_raw/theora', '/dev/null'),

            ]
        )
    ])