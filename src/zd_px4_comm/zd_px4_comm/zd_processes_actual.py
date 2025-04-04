
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run rqt_image
    "ros2 run rqt_image_view rqt_image_view && exit",

    # # Run the Micro XRCE-DDS Agent
    # "cd && MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600 && exit",

    # # Run QGroundControl
    # "cd && ./QGroundControl-x86_64.AppImage",

    # # Run dual ultrasonic sensor
    # "ros2 run sensor_interface ultrasonic_dual && exit",

    "ros2 launch v4l2_camera raw_camera.launch.py && exit",

    "ros2 run sensor_interface lidar && exit",

    # "ros2 topic echo /fmu/out/vehicle_odometry && exit",

    # Run camera color plug first
    # "ros2 run v4l2_camera v4l2_camera_node \
    #   --ros-args \
    #   -r __ns:=/color_camera \
    #   -p video_device:=/dev/video0 \
    #   -p camera_info_url:=file:///home/crestjj/zephyrDock/src/ros2_v4l2_camera/config/calcam_color.yaml && exit",

    #Run camera bnw plug second
    # "ros2 run v4l2_camera v4l2_camera_node \
    #   --ros-args \
    #   -r __ns:=/bnw_camera \
    #   -p video_device:=/dev/video2 \
    #   -p camera_info_url:=file:///home/crestjj/zephyrDock/src/ros2_v4l2_camera/config/calcam_bnw.yaml && exit",

]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
    
