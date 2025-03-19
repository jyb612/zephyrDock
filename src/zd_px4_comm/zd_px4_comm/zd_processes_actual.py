
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run rqt_image
    "ros2 run rqt_image_view rqt_image_view && exit",

    # Run the Micro XRCE-DDS Agent
    "cd && MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600",

    # Run QGroundControl
    "cd && ./QGroundControl-x86_64.AppImage",

    # Run tf mini lidar
    "ros2 run sensor_interface lidar",

    # Run dual ultrasonic sensor
    "ros2 run sensor_interface ultrasonic_dual",
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
    
