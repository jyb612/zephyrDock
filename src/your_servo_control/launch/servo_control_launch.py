import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node  # Correct import for ROS 2 Humble

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_servo_control',   # Your package name
            executable='servo_control_node',  # Name of the executable node
            name='servo_control',
            output='screen',
            parameters=[{'servo_position': 3072}]  # Example parameter
        ),
    ])
