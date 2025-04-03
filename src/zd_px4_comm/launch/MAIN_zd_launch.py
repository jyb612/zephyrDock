#!/usr/bin/env python

__author__ = "Chua Jun Yan"
__contact__ = "chua.junyan0612@gmail.com"

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    package_dir = get_package_share_directory('zd_px4_comm')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')

    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        
        

        Node(               # SIM
            package='zd_px4_comm',
            namespace='zd_px4_comm',
            executable='zd_processes',
            name='zd_processes',
            prefix='gnome-terminal --'
        ),
        
        # Launch Aruco Tracker node from aruco_tracker package
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
            ]
        ), 

        # Node(
        #     package='zd_px4_comm',
        #     namespace='zd_px4_comm',
        #     executable='zd_processes_actual',
        #     name='zd_processes_actual',
        #     prefix='gnome-terminal --'
        # ),

        # # # Launch Aruco Tracker node from aruco_tracker package
        # Node(
        #     package='aruco_tracker',
        #     executable='aruco_tracker',
        #     name='aruco_tracker',
        #     output='screen',
        #     parameters=[
        #         PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml']),
        #         {'camera_namespace': '/color_camera'}  # Adding camera_namespace parameter
        #     ]
        # ),    

        # # # Launch Aruco Tracker node from aruco_tracker package
        # Node(
        #     package='aruco_tracker',
        #     executable='aruco_tracker',
        #     name='aruco_tracker',
        #     output='screen',
        #     parameters=[
        #         PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml']),
        #         {'camera_namespace': '/bnw_camera'}
        #     ]
        # ),    

        # Add a delay (e.g., 5 seconds) before launching the precision_land node
        TimerAction(
            period=5.0,  # Delay for 5 seconds
            actions=[
                Node(
                    package='precision_land',
                    executable='precision_land',
                    name='precision_land',
                    output='screen',
                    parameters=[
                        PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
                    ]
                )
            ]
        ),
    ])