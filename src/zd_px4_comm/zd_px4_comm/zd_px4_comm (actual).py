#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from std_msgs.msg import Int32, Bool, Float64  # For servo command
import math
import numpy as np

class ZDCommNode(Node):
    def __init__(self):
        super().__init__('zd_px4_command_node')

        # Define QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.servo_command_publisher = self.create_publisher(Int32, '/target_position_command', 10)  # Servo command publisher
        self.aruco_id_publisher = self.create_publisher(Int32, '/aruco_id', 10)
        self.marker_size_publisher = self.create_publisher(Float64, '/marker_size', 10)

        # Subscribe to current position
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        # Custom mode completion signal
        self.create_subscription(
            Bool, 
            '/precision_hovering_done', 
            self.precision_hovering_done_callback, 
            10
        )  
        
        self.create_subscription(
            Float64, 
            '/current_altitude', 
            self.current_altitude_callback, 
            10
        )
        
        self.create_subscription(
            Bool, 
            '/robot_return_flag', 
            self.robot_return_flag_callback, 
            10
        )
        
        self.create_subscription(
            Float64, 
            '/solar_panel_angle_in_rad', 
            self.solar_panel_angle_in_rad_callback, 
            10
        )

        # self.create_subscription(
        #     Float64, 
        #     '/solar_panel_robot_waypoint', 
        #     self.solar_panel_robot_waypoint_callback, 
        #     10
        # )

        # State variables
        self.service_mode = None

        self.current_position = [0.0, 0.0, 0.0]  # Current position in NED
        self.anchor_position = [0.0, 0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.current_euler = [0.0, 0.0, 0.0] # roll, pitch, yaw (rad)
        self.origin_position = [0.0, 0.0, 0.0]

        self.search_altitude = -8.0  # Takeoff altitude in NED (8 meters up)
        self.pre_home_descend_altitude = -3.5
        self.deploy_altitude = -5.0
        self.waypoint_home = [0.0, 0.0, self.search_altitude]
        self.waypoint_solar_panel = [-6.0, 0.0, self.search_altitude]
        self.angular_velocity_threshold = 0.01  # Threshold for angular velocity (radians per second)
        self.time_threshold = 3.0
        self.robot_return_flag = False
        self.solar_panel_angle_in_rad = 0.0
    
        self.waypoint = "HOME"
        self.current_mode = None  # Current flight mode
        self.aruco_id = 0
        self.armed = False  # Armed state
        self.custom_mode_done = False
        self.current_altitude = None  # To store the current altitude
        self.gripper_gripped = False 
        self.loop_once = False 
        self.drone_return = False 
        
        self.mode_callback_time = None
        self.hover_start_time = None  # Time when hovering starts      

        self.state = "SERVICE_SELECT"  # State machine state
        self.running = True

        # Timer to control the state machine
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2Hz

    def robot_return_flag_callback(self, msg):
        self.robot_return_flag = True if msg.data else False
    
    def solar_panel_angle_in_rad_callback(self, msg):
        self.solar_panel_angle_in_rad = msg.data

    def current_altitude_callback(self, msg):
        """Callback to handle current altitude."""
        self.current_altitude = msg.data
        # self.get_logger().info(f"Received current altitude: {self.current_altitude} meters")
        
    def odometry_callback(self, msg):
        """Callback to update the current position."""
        self.current_position = [
            float(msg.position[0]),  # X in NED
            float(msg.position[1]),  # Y in NED
            float(msg.position[2]),  # Z in NED
        ]
        self.angular_velocity = [
            float(msg.angular_velocity[0]),  # Roll angular velocity
            float(msg.angular_velocity[1]),  # Pitch angular velocity
            float(msg.angular_velocity[2]),  # Yaw angular velocity
        ]
        # self.get_logger().info(f"(X={msg.position[0]}, Y={msg.position[1]}, Z={msg.position[2]})")

        # Extract quaternion
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        self.current_euler = self.quaternion_to_euler(q)

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def vehicle_status_callback(self, msg):
        """Callback to update the current mode."""
        self.current_mode = msg.nav_state
        self.armed = True if msg.arming_state == 2 else False
        # self.get_logger().info(f"Current mode: {self.current_mode}")

    def precision_hovering_done_callback(self, msg):
        """Callback to handle custom precision land (hover) mode completion signal."""
        if msg.data:  # If the custom precision land (hover) mode is done
            self.custom_mode_done = True
            # self.get_logger().info("Custom mode completed. Ready for servo action.")
        else:
            self.custom_mode_done = False   # impossible

    def get_marker_size(self, aruco_id):
        """Return the marker size based on the ArUco ID."""
        return 0.5 if aruco_id == 2 else 0.18
        
    def publish_offboard_control_mode(self):
        """Publish OffboardControlMode message."""
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_control_mode_publisher.publish(offboard_msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-8.0, yaw=0.0):
        """Publish a trajectory setpoint."""
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        trajectory_msg.position = [x, y, z]  # Set desired position
        trajectory_msg.yaw = yaw  # Set desired yaw
        self.trajectory_setpoint_publisher.publish(trajectory_msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        """Publish a VehicleCommand."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        # self.get_logger().info(f"Published VehicleCommand: command={command}, param1={param1}, param2={param2}, param3={param3}")

    def arm_drone(self):
        """Command the drone to arm."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Drone armed.")

    def publish_aruco_info(self, aruco_id):
        """Publish the ArUco ID and corresponding marker size for the given ID."""
        # Determine the marker size based on the ArUco ID
        self.aruco_id = aruco_id
        marker_size = self.get_marker_size(aruco_id)

        # Publish ArUco ID
        aruco_msg = Int32()
        aruco_msg.data = aruco_id
        self.aruco_id_publisher.publish(aruco_msg)
        
        # Publish Marker Size
        marker_size_msg = Float64()
        marker_size_msg.data = marker_size
        self.marker_size_publisher.publish(marker_size_msg)

        msg = "Drone Home 0" if aruco_id == 0 else "Robot Home 1" if aruco_id == 1 else "Home Landmark 2" if aruco_id == 2 else "Robot 3"
        self.get_logger().info(f"Published ArUco ID: {msg}.")

    def publish_takeoff(self):
        """Send takeoff command to PX4."""
        # Takeoff command (mode 3 corresponds to takeoff mode)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 2.0) # original take off altitude
        self.get_logger().info(f"Sending arm and takeoff command.")
            
    def stable_check(self, isgood):
        if isgood:
            if self.hover_start_time is None:
                # If it's the first time descend_rate is 0, store the time
                self.hover_start_time = time.time()
            else:
                # If it's already been 3 seconds since descend_rate became 0
                if time.time() - self.hover_start_time >= self.time_threshold:
                    self.hover_start_time = None
                    self.get_logger().info("Drone stable, calling next function")
                    return True
        else:
            self.hover_start_time = None  # Reset the timer if descend_rate is not 0
        return False

    def timer_callback(self):
        if self.state != "SERVICE_SELECT" and time.time() - self.mode_callback_time >= 3:
            # self.get_logger().info(f"Current state = {self.state}")
            self.mode_callback_time = time.time()
        
        """Main loop that implements the state machine."""
        if self.state == "SERVICE_SELECT":
            self.mode_callback_time = time.time()
            self.service_mode = input("Input 'd' to deploy, 'r' to return robot: ").upper()
            if self.service_mode == 'D' or self.service_mode == 'R':
                self.origin_position[0] = self.current_position[0]
                self.origin_position[1] = self.current_position[1]
                self.origin_position[2] = self.current_position[2]
                self.state = "ARMING"
        elif self.state == "ARMING":
            if not self.armed:              # ensure arm and take off (repeatedly send signal)
                self.arm_drone()
                self.publish_takeoff()
            else:
                self.state = "TAKEOFF"
                time.sleep(8)               # spare time for takeoff mode
                self.anchor_position[3] = self.current_euler[2]
            
        elif self.state == "TAKEOFF":
            if not self.loop_once:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint(z=self.current_position[2], yaw=self.anchor_position[3])
                self.loop_once = True
                # self.get_logger().info("Switched to Offboard mode")
            
            if self.service_mode == 'D':
                takeoff_altitude = self.pre_home_descend_altitude
            else:
                takeoff_altitude = self.search_altitude # waypoint to solar panel (return robot)

            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(z=takeoff_altitude, yaw=self.anchor_position[3])
            
            if abs(self.current_position[2] - takeoff_altitude) <= 0.1:  # Allow small tolerance
                isgood = True
            else:
                isgood = False
                
            if self.stable_check(isgood):
                self.state = "HOVER"
                self.loop_once = False


        elif self.state == "HOVER":
            if not self.loop_once:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0) # Loiter mode
                self.loop_once = True
                self.hover_start_time = time.time()
                self.get_logger().info("Hovering, calling next action")
            if time.time() - self.hover_start_time > 5.0:  # Hover for 5 seconds
                self.hover_start_time = None
                if self.drone_return:
                    self.publish_aruco_info(0)
                    self.state = "PRE_HOME_DESCEND"
                elif self.service_mode == "D":
                    self.state = "CUSTOM_PRECISION_DESCEND"
                    self.publish_aruco_info(1)
                elif self.service_mode == "R":
                    self.state = "WAYPOINT_SOLAR_PANEL"
                    self.publish_aruco_info(3)
                    if self.gripper_gripped:
                        self.state = "PRE_HOME_DESCEND"
                        self.publish_aruco_info(1)
                        
                self.loop_once = False
                self.hover_start_time = None

        elif self.state == "PRE_HOME_DESCEND":
            descend_rate = 0.3
            if not self.loop_once:
                self.anchor_position[0] = 0.0
                self.anchor_position[1] = 0.0
                self.anchor_position[2] = self.search_altitude
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Switch to Offboard mode
                self.loop_once = True
                self.get_logger().info("Initiate Phase 1 Descent.")
            if self.current_position[2] - self.pre_home_descend_altitude >= -0.1:
                self.state = "CUSTOM_PRECISION_DESCEND"
                self.loop_once = False
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.current_position[2]+descend_rate, yaw=self.anchor_position[3])

        elif self.state == "CUSTOM_PRECISION_DESCEND":  # if id=0 land, else hover
            if not self.loop_once:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 11.0)  # Switch to custom precision land (hover) mode
                self.custom_mode_done = False
                self.loop_once = True
                self.get_logger().info("Initiate precision landing (align to aruco marker).")
                
            # try:

            #     if self.aruco_id == 2:
            #         if self.current_altitude <= 3.5:
            #             if self.drone_return:
            #                 self.publish_aruco_info(0)  # Return Robot
            #             elif self.gripper_gripped:
            #                 self.publish_aruco_info(3)  # Land Drone Home
            #             elif self.gripper_gripped:
            #                 self.publish_aruco_info(1)
            # except TypeError:
            #     pass
            
            if (self.aruco_id == 0):
                self.state = "COMPLETE"
                self.get_logger().info("Precision Landing on Drone Home automatically...")

            elif self.custom_mode_done:  # Wait for custom precision land (hover) mode completion signal
                self.anchor_position[3] = self.current_euler[2]
                self.state = "SERVO_ACTION"
                self.loop_once = False

        elif self.state == "SERVO_ACTION":
            # Publish servo command
            if not self.loop_once:
                self.anchor_position[0] = self.current_position[0]
                self.anchor_position[1] = self.current_position[1]
                self.anchor_position[2] = self.current_position[2]
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Switch to Offboard mode
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.anchor_position[2], yaw=self.anchor_position[3])
                servo_msg = Int32()
                self.loop_once = True
                self.hover_start_time = time.time()
                if not self.gripper_gripped:
                    servo_msg.data = 2048  # Example servo position
                    self.gripper_gripped = True
                    self.get_logger().info("Grip - servo command published.")
                else:
                    servo_msg.data = 1024
                    self.gripper_gripped = False
                    self.drone_return = True 
                    self.get_logger().info("Release - servo command published.")
                self.servo_command_publisher.publish(servo_msg)
            elif time.time() - self.hover_start_time >= 2.0:
                self.state = "ASCEND"
                self.loop_once = False
                self.hover_start_time = None
                self.get_logger().info("Ascending.")
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.anchor_position[2], yaw=self.anchor_position[3])

        elif self.state == "ASCEND":
            self.publish_offboard_control_mode()
            if self.drone_return and self.waypoint == "HOME":
                takeoff_altitude = self.pre_home_descend_altitude
            else:
                takeoff_altitude = self.search_altitude
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=takeoff_altitude, yaw=self.anchor_position[3])  # Ascend to takeoff altitude
            if abs(self.current_position[2] - takeoff_altitude) <= 0.5:  # Allow small tolerance
                isgood = True
            else:
                isgood = False
            
            if self.stable_check(isgood):
                if self.waypoint == "SOLAR PANEL": # far from home
                    self.state = "WAYPOINT_HOME"
                elif self.drone_return:
                    self.state = "CUSTOM_PRECISION_DESCEND"
                    self.publish_aruco_info(0)
                elif self.waypoint == "HOME":
                    self.state = "WAYPOINT_SOLAR_PANEL"
                # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)  # Switch to Loiter (Hold) mode
                # time.sleep(5)
        
        elif self.state == "WAYPOINT_SOLAR_PANEL":
            if not self.loop_once:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Switch to Offboard mode
                self.loop_once = True
                self.waypoint = "SOLAR PANEL"
                self.get_logger().info("Initiate waypoint to Solar Panel")
            
            self.publish_trajectory_setpoint(x=self.waypoint_solar_panel[0], y=self.waypoint_solar_panel[1], z=self.waypoint_solar_panel[2], yaw=self.anchor_position[3])
            self.publish_offboard_control_mode()

            if abs(self.current_position[0] - self.waypoint_solar_panel[0]) <= 0.1 and abs(self.current_position[1] - self.waypoint_solar_panel[1]) <= 0.1:  # Allow small tolerance
                isgood = True
            else:
                isgood = False
                
            if self.stable_check(isgood):
                if self.service_mode == 'D':
                    self.state = "ALIGN_SOLAR_PANEL"
                    # Begin aligning with solar panel (yawing)
                    # self.get_logger().info("Yawing until ultrasonic sensor readings tally...")
                else:
                    self.state = "CUSTOM_PRECISION_DESCEND" #id3
                self.loop_once = False
            
        elif self.state == "ALIGN_SOLAR_PANEL":
            if not self.loop_once:
                self.hover_start_time = time.time()
                self.loop_once = True
                self.get_logger().info("Payload aligning with slope, drone yawing")
            # Assuming you already have the sensor readings as variables `left_sensor` and `right_sensor`
            # These sensor values will be set through the sensor callback

            # If the ultrasonic readings match, proceed
            # if abs(self.left_sensor_reading - self.right_sensor_reading) <= 2:  # Define a small tolerance
            if time.time() - self.hover_start_time >= 10:
                self.state = "DEPLOY_DESCEND"
                self.hover_start_time = None
                self.loop_once = False
                self.anchor_position[3] = self.current_euler[2]
                self.publish_trajectory_setpoint(x=self.waypoint_solar_panel[0], y=self.waypoint_solar_panel[1], z=self.waypoint_solar_panel[2], yaw = self.anchor_position[3])
                self.get_logger().info("Descending for payload release")
                # self.get_logger().info("Ultrasonic readings matched, proceeding to deploy.")
            #     if self.hover_start_time is None:
            #         # If it's the first time descend_rate is 0, store the time
            #         self.hover_start_time = time.time()
            #     else:
            #         # If it's already been 3 seconds since descend_rate became 0
            #         if time.time() - self.hover_start_time >= self.time_threshold:
            #             self.state = "HOVER"
            #             self.hover_start_time = None
            # else:
            #     self.hover_start_time = None  # Reset the timer if descend_rate is not 0
            else:
                # Continue yawing slowly if sensor readings don't match
                yaw_rate = 0.05  # Set the slow yaw rate (you can adjust this value)
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint(x=self.waypoint_solar_panel[0], y=self.waypoint_solar_panel[1], z=self.waypoint_solar_panel[2], yaw=self.current_euler[2] + yaw_rate)
                # self.get_logger().info(f"Yawing: current yaw = {math.degrees(self.current_euler[2])}°, yaw rate = {math.degrees(yaw_rate)}°/signal")
                # Optionally add a timeout to prevent infinite yawing if needed
                if time.time() - self.hover_start_time > 60:  # If yawing exceeds 60 seconds, stop and check
                    self.state = "WAYPOINT_SOLAR_PANEL"
                    self.get_logger().warn("Yawing exceeded time limit, checking sensor again.")
                    self.hover_start_time = None
            self.publish_offboard_control_mode()

        elif self.state == "DEPLOY_DESCEND": 
            altitude_difference = self.current_position[2] - self.deploy_altitude

            if altitude_difference <= -4:  # If much higher than target (more negative)
                descend_rate = 0.3  # Faster descent
            elif altitude_difference <= -1.5:  # A little higher than the target
                descend_rate = 0.2  # Moderate descent
            elif altitude_difference >= 0.25:  # If lower than target (positive means lower)
                descend_rate = -0.15  # Ascend
            elif altitude_difference >= -0.2:  # Close to the target
                descend_rate = 0  # Stop descending
            else:  # If the altitude difference is very small (close enough to target)
                descend_rate = 0.05  # Slow descent, keeping a small movement


            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.waypoint_solar_panel[0], y=self.waypoint_solar_panel[1], z=self.current_position[2]+descend_rate, yaw = self.anchor_position[3])
            # instant altitude + descend rate -> more positive -> more low altitude
            # self.get_logger().info(f"Current altitude NED: {self.current_position[2]}")
            isgood = True if descend_rate == 0 else False
                
            if self.stable_check(isgood):
                self.state = "SERVO_ACTION"


        elif self.state == "WAYPOINT_HOME":
            if not self.loop_once:
                self.loop_once = True
                self.get_logger().info("Waypoint to Home")
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(z=self.search_altitude, yaw = self.anchor_position[3])
            
            if abs(self.current_position[0]) <= 0.1 and abs(self.current_position[1]) <= 0.1:
                isgood = True
            else:
                isgood = False

            if self.stable_check(isgood):
                self.state = "HOVER"
                self.waypoint = "HOME"
                self.loop_once = False
                        
        elif self.state == "COMPLETE":
            if abs(self.current_position[2] - self.origin_position[2]) <= 0.2:
                if self.loop_once:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)  # Land
                if not self.armed:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)  # Land
                    self.get_logger().warn("Exiting Node...")
                    self.running = False
            # Do nothing, mission is complete
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ZDCommNode()

    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
            # rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
    finally:
        node.destroy_node()
        # Check if ROS is still running before shutting down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
