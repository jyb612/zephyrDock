
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run rqt_image
    "ros2 run rqt_image_view rqt_image_view && exit",

    # Run the Micro XRCE-DDS Agent
    "cd ~/zephyrDock/Micro-XRCE-DDS-Agent && MicroXRCEAgent udp4 -p 8888",

    # Run QGroundControl
    "cd ~/zephyrDock && ./QGroundControl-x86_64.AppImage",
       
    #Run image bridge
    "ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image && exit",
    
    #Run camera info bridge
    "ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo && exit",

    # Run the PX4 SITL simulation
    "cd ~/zephyrDock/PX4-Autopilot && make px4_sitl gz_x500_mono_cam_down",
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
    
