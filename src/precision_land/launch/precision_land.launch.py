# # DISCLAIMER # #

# UNUSED #
# ALREADY LAUNCH VIA zd_px4_comm #

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['screen', '-dmS', 'dds_agent', 'bash', '-c', 'MicroXRCEAgent udp4 -p 8888'],
        #     name='dds_agent_process'
        # ),
        # Run Micro XRCE-DDS Agent in a new tab
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--tab', '--', 'bash', '-c', 'cd ~/zephyrDock/Micro-XRCE-DDS-Agent && MicroXRCEAgent udp4 -p 8888; exec bash'],
        #     name='dds_agent_process',
        #     output='screen'
        # ),
        
        # # Run PX4 SITL simulation in a new tab
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--tab', '--', 'bash', '-c', 'cd ~/zephyrDock/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down; exec bash'],
        #     name='px4_sitl_process',
        #     output='screen'
        # ),
        
        # # Optionally: Run QGroundControl in a new tab (commented out here)
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--tab', '--', 'bash', '-c', 'cd ~/zephyrDock && ./QGroundControl-x86_64.AppImage; exec bash'],
        #     name='qgroundcontrol_process',
        #     output='screen'
        # ),
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
            ]
        ),
    ])
