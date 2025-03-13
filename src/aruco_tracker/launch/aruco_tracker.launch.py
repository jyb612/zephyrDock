from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # # Run Micro XRCE-DDS Agent in a new tab
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
        
        # # Run the image bridge in the same terminal
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--tab', '--', 'bash', '-c', 'ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image; exec bash'],
        #     name='image_bridge_process',
        #     output='screen'
        # ),
        
        # # Run the camera info bridge in the same terminal
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--tab', '--', 'bash', '-c', 'ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo; exec bash'],
        #     name='camera_info_bridge_process',
        #     output='screen'
        # ),
        
        # Run Aruco Tracker Node in the same terminal
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
            ]
        ),
    ])

