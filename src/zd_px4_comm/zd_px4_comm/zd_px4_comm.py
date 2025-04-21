#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from std_msgs.msg import Int32, Bool, Float32, Float32MultiArray  # For servo command
import math
import numpy as np
# import csv
# from datetime import datetime
# import os

MAIN_VEHICLE_MODE_OFFBOARD = 6.0         # Offboard param 1 = 1.0

SWAP_TO_SUB_VEHICLE_MODE = 4.0          # Auto  param 2
SUB_VEHICLE_MODE_TAKEOFF = 2.0          # Takeoff       param3          param1 = 1.0 param2 = 4.0
SUB_VEHICLE_MODE_LOITER = 3.0           # Loiter (Hold) param3          param1 = 1.0 param2 = 4.0
SUB_VEHICLE_MODE_CUSTOM_MODE = 11.0     # External mode 1   param3      param1 = 1.0 param2 = 4.0
SUB_VEHICLE_MODE_LAND = 6.0             # LAND   param3                 param1 = 1.0 param2 = 4.0

RELEASE = 3072
GRIP = 2048
GRIP_STRONG = 1024


class ZDCommNode(Node):
    def __init__(self):
        super().__init__('zd_px4_command_node')
        # Define all QoS profiles at the top for clarity and consistency
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        critical_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Or RELIABLE if critical
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5  # Moderate buffer for sensor data
        )

        # Then apply these consistently:

        # PX4 Interface (BEST_EFFORT)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', critical_qos)

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, px4_qos)
        # self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, px4_qos)

        # Critical Commands (RELIABLE)
        self.create_subscription(Bool, '/precision_hovering_done', self.precision_hovering_done_callback, critical_qos)
        self.create_subscription(Bool, '/robot_return_flag', self.robot_return_flag_callback, critical_qos)
        self.create_subscription(Bool, '/isloaded', self.isloaded_callback, critical_qos)

        # Sensor Data (choose appropriate reliability)
        self.create_subscription(Float32, '/gyus42v2/left_range32', self.ultrasonic_left_range32_callback, sensor_qos)
        self.create_subscription(Float32, '/gyus42v2/right_range34', self.ultrasonic_right_range34_callback, sensor_qos)
        self.create_subscription(Float32, '/tfmini/range', self.lidar_range_callback, sensor_qos)

        # Other topics
        self.servo_command_publisher = self.create_publisher(Int32, '/servo_command', critical_qos)
        self.aruco_id_publisher = self.create_publisher(Int32, '/aruco_id', critical_qos)
        self.marker_size_publisher = self.create_publisher(Float32, '/marker_size', critical_qos)
        self.is_active_cam_color_publisher = self.create_publisher(Bool, '/is_active_cam_color', critical_qos)
        
        # # Define QoS profile for PX4 compatibility
        # px4_besteffort_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        # bool_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures message delivery
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10  # Store last 10 messages in queue
        # )


        # # Publishers
        # self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_besteffort_qos)
        # self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_besteffort_qos)
        # self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_besteffort_qos)
        # self.servo_command_publisher = self.create_publisher(Int32, '/servo_command', 10)  # Servo command publisher
        # self.aruco_id_publisher = self.create_publisher(Int32, '/aruco_id', 10)
        # self.marker_size_publisher = self.create_publisher(Float32, '/marker_size', 10)
        # self.is_active_cam_color_publisher = self.create_publisher(Bool, '/is_active_cam_color', 10)

        # # Subscribe to current position
        # self.create_subscription(
        #     VehicleOdometry,
        #     '/fmu/out/vehicle_odometry',
        #     self.odometry_callback,
        #     px4_besteffort_qos
        # )

        # self.create_subscription(
        #     VehicleStatus,
        #     '/fmu/out/vehicle_status',
        #     self.vehicle_status_callback,
        #     px4_besteffort_qos
        # )

        # # Custom mode completion signal
        # self.create_subscription(
        #     Bool, 
        #     '/precision_hovering_done', 
        #     self.precision_hovering_done_callback, 
        #     bool_qos
        # )  
        
        # self.create_subscription(               # external signal (undone)
        #     Bool, 
        #     '/robot_return_flag', 
        #     self.robot_return_flag_callback, 
        #     bool_qos
        # )
        
        # self.create_subscription(               # external signal (James)
        #     Float32, 
        #     '/solar_panel_angle_in_rad', 
        #     self.solar_panel_angle_in_rad_callback, 
        #     bool_qos
        # )

        # # self.create_subscription(               # external signal (James)
        # #     Float32MultiArray, 
        # #     '/solar_panel_robot_waypoint', 
        # #     self.solar_panel_robot_waypoint_callback, 
        # #     bool_qos
        # # )

        # self.create_subscription(
        #     Bool, 
        #     '/isloaded', 
        #     self.isloaded_callback, 
        #     bool_qos
        # )

        # self.create_subscription(
        #     Float32, 
        #     '/gyus42v2/left_range32', 
        #     self.ultrasonic_left_range32_callback, 
        #     bool_qos
        # )

        # self.create_subscription(
        #     Float32, 
        #     '/gyus42v2/right_range34', 
        #     self.ultrasonic_right_range34_callback, 
        #     bool_qos
        # )

        # self.create_subscription(
        #     Float32, 
        #     '/tfmini/range', 
        #     self.lidar_range_callback, 
        #     bool_qos
        # )

        # State variables
        self.service_mode = None

        self.current_position = [0.0, 0.0, 0.0]  # Current position in NED
        self.anchor_position = [0.0, 0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.current_yaw = None
        # self.current_euler = [0.0, 0.0, 0.0] # roll, pitch, yaw (rad)
        self.origin_position = [0.0, 0.0, 0.0]
        self.above_ground_altitude = None  # To store the current altitude
        self.solar_panel_angle_in_rad = 0.0
        self.ultrasonic_left_range32 = None
        self.ultrasonic_right_range34 = None
        self.aruco_id = 0

        self.search_altitude = -4.5  # Takeoff altitude in NED (8 meters up)
        self.pre_home_descend_altitude = -3.5
        self.deploy_altitude = -3.5
        self.waypoint_home = [0.0, 0.0, self.search_altitude]
        self.waypoint_solar_panel = [0.0, 0.0, self.search_altitude]
        self.waypoint_solar_panel_distance = 5.0
        self.angular_velocity_threshold = 0.01  # Threshold for angular velocity (radians per second)
        self.time_threshold = 3.0
        self.hori_grip_height = None
        self.lidar_gripper_offset_front = -0.15      ## ATTENTION
        self.z_offset = -0.5
        self.lidar_ground_level = None

        self.waypoint = "HOME"
        self.current_mode = None  # Current flight mode
        self.armed = False  # Armed state
        self.custom_mode_done = False

        self.aruco_descend = False
        self.gripper_gripped = False 
        self.loop_once = False 
        self.drone_return = False 
        self.is_active_cam_color = True
        self.isloaded = False
        self.gripper_retry = False
        self.robot_return_flag = False
                
        self.hover_start_time = None  # Time when hovering starts      

        self.state = "SERVICE_SELECT"  # State machine state
        self.running = True

        # Timer to control the state machine
        self.timer = self.create_timer(0.05, self.timer_callback)  # 2Hz

        # # Create logs directory if needed
        # os.makedirs('logs', exist_ok=True)
        
        # # Open CSV file with additional lidar column
        # log_filename = f"logs/odometry_lidar_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        # self.odom_log = open(log_filename, 'w')
        # self.odom_writer = csv.writer(self.odom_log)
        # self.odom_writer.writerow(['timestamp', 'x', 'y', 'z', 'lidar_altitude'])
        # self.get_logger().info(f"Logging data to: {os.path.abspath(log_filename)}")

    def lidar_range_callback(self, msg):
        self.above_ground_altitude = -float(msg.data)   # negative sign for FRD NED coordinate system

    def ultrasonic_left_range32_callback(self, msg):
        self.ultrasonic_left_range32 = float(msg.data)
   
    def ultrasonic_right_range34_callback(self, msg):
        self.ultrasonic_right_range34 = float(msg.data)

    def isloaded_callback(self, msg):
        self.isloaded = True if msg.data else False

    def robot_return_flag_callback(self, msg):
        self.robot_return_flag = True if msg.data else False
    
    def solar_panel_angle_in_rad_callback(self, msg):

        self.get_logger().info(f'Published: {msg.data}')
    
    def local_position_callback(self, msg):
        """Callback to update the current position from vehicle_local_position."""
        # Check if position estimates are valid before using them
        if not (msg.xy_valid and msg.z_valid):
            self.get_logger().warn("Position estimate not valid!")
            return

        # Store NED position (X, Y, Z)
        self.current_position = [
            float(msg.x),  # North (X) in meters
            float(msg.y),  # East (Y) in meters
            float(msg.z),  # Down (Z) in meters (negative for altitude)
        ]

        # # Store velocities (if needed)
        # self.current_velocity = [
        #     float(msg.vx),  # North velocity
        #     float(msg.vy),  # East velocity
        #     float(msg.vz),  # Down velocity
        # ]

        # Store yaw (heading) in radians
        self.current_yaw = float(msg.heading)

        # # Logging (optional)
        # self.get_logger().info(
        #     f"Position (NED): X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}, Yaw={np.degrees(msg.heading):.2f}°"
        # )

    # def odometry_callback(self, msg):
    #     """Callback to update the current position."""
    #     self.current_position = [
    #         float(msg.position[0]),  # X in NED
    #         float(msg.position[1]),  # Y in NED
    #         float(msg.position[2]),  # Z in NED
    #     ]
    #     self.angular_velocity = [
    #         float(msg.angular_velocity[0]),  # Roll angular velocity
    #         float(msg.angular_velocity[1]),  # Pitch angular velocity
    #         float(msg.angular_velocity[2]),  # Yaw angular velocity
    #     ]
    #     # self.get_logger().info(f"(X={msg.position[0]}, Y={msg.position[1]}, Z={msg.position[2]})")

    #     # Simple logging - just position and timestamp
    #     # self.odom_writer.writerow([
    #     #     time.time(),
    #     #     self.current_position[0],
    #     #     self.current_position[1],
    #     #     self.current_position[2],
    #     #     self.above_ground_altitude
    #     # ])

    #     # self.odom_log.flush()  # Ensure data is written immediately
        
    #     # Extract quaternion
    #     q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        
    #     # Convert quaternion to Euler angles (roll, pitch, yaw)
    #     self.current_euler = self.quaternion_to_euler(q)
    #     # yaw = -180 ~ 180

    # def quaternion_to_euler(self, q):
    #     """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    #     # Roll (x-axis rotation)
    #     sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    #     cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    #     roll = np.arctan2(sinr_cosp, cosr_cosp)

    #     # Pitch (y-axis rotation)
    #     sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    #     if abs(sinp) >= 1:
    #         pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    #     else:
    #         pitch = np.arcsin(sinp)

    #     # Yaw (z-axis rotation)
    #     siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    #     cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    #     yaw = np.arctan2(siny_cosp, cosy_cosp)

    #     return roll, pitch, yaw

    def vehicle_status_callback(self, msg):
        """Callback to update the current mode."""
        self.current_mode = msg.nav_state
        self.armed = True if msg.arming_state == 2 else False
        # self.get_logger().info(f"Current mode: {self.current_mode}")
        # uint8 nav_state                                 # Currently active mode
        # uint8 NAVIGATION_STATE_MANUAL = 0               # Manual mode
        # uint8 NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode
        # uint8 NAVIGATION_STATE_POSCTL = 2               # Position control mode
        # uint8 NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode
        # uint8 NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode
        # uint8 NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode
        # uint8 NAVIGATION_STATE_POSITION_SLOW = 6
        # uint8 NAVIGATION_STATE_FREE5 = 7
        # uint8 NAVIGATION_STATE_FREE4 = 8
        # uint8 NAVIGATION_STATE_FREE3 = 9
        # uint8 NAVIGATION_STATE_ACRO = 10                # Acro mode
        # uint8 NAVIGATION_STATE_FREE2 = 11
        # uint8 NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)
        # uint8 NAVIGATION_STATE_TERMINATION = 13         # Termination mode
        # uint8 NAVIGATION_STATE_OFFBOARD = 14
        # uint8 NAVIGATION_STATE_STAB = 15                # Stabilized mode
        # uint8 NAVIGATION_STATE_FREE1 = 16
        # uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
        # uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land
        # uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
        # uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target
        # uint8 NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle
        # uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
        # uint8 NAVIGATION_STATE_EXTERNAL1 = 23
        # uint8 NAVIGATION_STATE_EXTERNAL2 = 24
        # uint8 NAVIGATION_STATE_EXTERNAL3 = 25
        # uint8 NAVIGATION_STATE_EXTERNAL4 = 26
        # uint8 NAVIGATION_STATE_EXTERNAL5 = 27
        # uint8 NAVIGATION_STATE_EXTERNAL6 = 28
        # uint8 NAVIGATION_STATE_EXTERNAL7 = 29
        # uint8 NAVIGATION_STATE_EXTERNAL8 = 30
        # uint8 NAVIGATION_STATE_MAX = 31

    def precision_hovering_done_callback(self, msg):
        """Callback to handle custom precision land (hover) mode completion signal."""
        if msg.data:  # If the custom precision land (hover) mode is done
            self.custom_mode_done = True
            # self.get_logger().info("Custom mode completed. Ready for servo action.")
        else:
            self.custom_mode_done = False   # impossible

    def get_marker_size(self, aruco_id):
        """Return the marker size based on the ArUco ID."""
        return 0.5 if aruco_id == 2 else 0.17
        
    def publish_offboard_control_mode(self):
        """Publish OffboardControlMode message."""
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_control_mode_publisher.publish(offboard_msg)
    
    # def publish_trajectory_setpoint(self, x, y, z, yaw, speed=4.0):
    #     """
    #     Modified version that:
    #     - Uses speed parameter for both XY and Z movements
    #     - Adjusts Z incrementally based on speed
    #     - Uses LiDAR only for validation (never publishes directly)
    #     - Maintains FRD coordinate convention
    #     """
    #     # Current state (all in FRD frame)
    #     current_x = self.current_position[0]
    #     current_y = self.current_position[1]
    #     current_z = self.current_position[2]  # Negative down
    #     current_lidar_z = self.above_ground_altitude  # Already in FRD (negative)

    #     if (abs(x-current_x) <= 7 and abs(y-current_y) <= 7 ):

    #         # XY movement (using speed parameter)
    #         dx = x - current_x
    #         dy = y - current_y
    #         distance_xy = (dx**2 + dy**2)**0.5
            
    #         if distance_xy < 0.1:  # XY reached
    #             next_x, next_y = x, y
    #         else:
    #             step_xy = min(speed * 0.1, distance_xy)  # 10Hz control loop
    #             next_x = current_x + (dx/distance_xy)*step_xy
    #             next_y = current_y + (dy/distance_xy)*step_xy

    #         # # Z adjustment (using speed parameter scaled for vertical movement)
    #         # error = z - current_lidar_z  # Both negative values
    #         # vertical_speed = speed # Reduce speed for vertical (adjust factor as needed)
    #         # z_step_size = vertical_speed * 0.1  # 10Hz control loop

    #         # if abs(error) < 0.1:  # Within 15cm tolerance
    #         #     next_z = current_z  # Freeze Z adjustment
    #         #     z_status = "HOLD"
    #         # elif error > 0:  # Need to move downward (more negative)
    #         #     next_z = current_z + z_step_size
    #         #     z_status = "DESCEND"
    #         # else:  # Need to move upward (less negative)
    #         #     next_z = current_z - z_step_size
    #         #     z_status = "ASCEND"

    #         # Current and target Z values in FRD frame:
    #         # current_z = +5.0 means 5m downward (below origin)
    #         # target_z = +3.0 means 3m downward (want to ascend by 2m)

    #         Z_P_GAIN = 0.5
    #         error = z - current_lidar_z  # Both in FRD

    #         if abs(error) < 0.1:
    #             # Within tolerance → HOLD
    #             p_output = 0.0
    #             next_z = current_z
    #             z_status = "HOLD"
    #         else:
    #             # P-control output
    #             p_output = error * Z_P_GAIN

    #             # Clamp to max step size (speed=4m/s → max_z_step=0.4m per 10Hz)
    #             max_z_step = speed * 0.1
    #             p_output = max(-max_z_step, min(max_z_step, p_output))

    #             # Apply movement (FRD convention):
    #             next_z = current_z + p_output

    #             z_status = "DESCEND (DOWNWARD +Z)" if p_output > 0 else "ASCEND (UPWARD -Z)"


    #         # Publish setpoint (all FRD)
    #         trajectory_msg = TrajectorySetpoint()
    #         trajectory_msg.position = [next_x, next_y, next_z]
    #         trajectory_msg.yaw = yaw
    #         self.trajectory_setpoint_publisher.publish(trajectory_msg)

    #         # # Enhanced logging
    #         self.get_logger().info(
    #             f"Z Control: Target={z:.1f}m | Current={current_lidar_z:.1f}m | "
    #             f"Error={error:.2f}m | Output={p_output:.2f}m\n"
    #             f"Movement: {z_status} | New Z={next_z:.1f}m (FRD)"
    #         )
    #     else:
    #         trajectory_msg = TrajectorySetpoint()
    #         trajectory_msg.position = [current_x,current_y,current_z]
    #         trajectory_msg.yaw = yaw
    #         self.trajectory_setpoint_publisher.publish(trajectory_msg)

    #         self.get_logger().info(f"target position exceed limit\ntarget:({x},{y},{z}), current({current_x},{current_y},{current_z})")  

    def publish_trajectory_setpoint(self, x, y, z, yaw, speed=0.5):
        # Position error
        
        current_lidar_z = self.above_ground_altitude
        current_yaw = self.current_yaw  # You must track this from odometry or estimator

        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        dz = z - current_lidar_z
        yaw_error = math.atan2(math.sin(yaw - current_yaw), math.cos(yaw - current_yaw))  # shortest angle
        
        if (abs(x-self.origin_position[0]) <= 7 and abs(y-self.origin_position[2]) <= 7 ):

            # Proportional gain
            p_gain = 1.2
            yaw_p_gain = 1.5

            # Velocity calculation (P control)
            velocity_x = dx * p_gain
            velocity_y = dy * p_gain
            velocity_z = dz * p_gain

            # Clamp velocities
            velocity_x = max(min(velocity_x, speed), -speed)
            velocity_y = max(min(velocity_y, speed), -speed)
            velocity_z = max(min(velocity_z, speed), -speed)

            # Yaw control
            
            yaw_speed = yaw_error * yaw_p_gain
            yaw_speed = max(min(yaw_speed, speed), -speed)

            # Prepare trajectory setpoint message
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

            # Use pure velocity control (position = NaN)
            trajectory_msg.position = [math.nan, math.nan, math.nan]
            trajectory_msg.velocity = [velocity_x, velocity_y, velocity_z]
            trajectory_msg.yaw = math.nan  # PX4 ignores this if NaN
            trajectory_msg.yawspeed = yaw_speed

            # Publish the message
            self.trajectory_setpoint_publisher.publish(trajectory_msg)
        else:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position = [self.current_position[0],self.current_position[1],self.current_position[2]]
            trajectory_msg.yaw = yaw
            self.trajectory_setpoint_publisher.publish(trajectory_msg)

            self.get_logger().info(f"target position exceed limit\ntarget:({x},{y},{z}), current({self.current_position[0]},{self.current_position[1]},{current_lidar_z})")
    # def publish_trajectory_setpoint(self, x, y, z, yaw, speed=4.0):
    #     """
    #     Publishes a trajectory setpoint while ensuring correct handling of ENU (East-North-Up) 
    #     and FRD (Forward-Right-Down) coordinate frames.
    #     """

    #     target_z = z
    #     current_z = self.current_position[2]

    #     current_x = self.current_position[0]  # FRD X
    #     current_y = self.current_position[1]  # FRD Y

    #     # Compute direction vector in FRD
    #     dx = x - current_x
    #     dy = y - current_y
    #     dz = target_z - current_z  # Ensure FRD handling

    #     distance = (dx**2 + dy**2 + dz**2)**0.5

    #     if distance < 0.1:  # Reached target (tolerance in meters)
    #         next_x, next_y, next_z = x, y, target_z
    #     else:
    #         # Normalize and scale by speed (10Hz assumed)
    #         step = min(speed * 0.1, distance)
    #         next_x = current_x + (dx / distance) * step
    #         next_y = current_y + (dy / distance) * step
    #         next_z = current_z + (dz / distance) * step  # FRD Z handled correctly

    #     # self.get_logger().info(
    #     #     f"\ndx = {dx}, \ndy = {dy}, \ndz = {dz}, \nnext_x = {next_x}, \nnext_y = {next_y}, \nnext_z = {next_z}"
    #     # )

    #     # Publish setpoint (FRD frame)
    #     trajectory_msg = TrajectorySetpoint()
    #     trajectory_msg.position = [next_x, next_y, next_z]  # Ensure FRD compliance
    #     trajectory_msg.yaw = yaw
    #     self.trajectory_setpoint_publisher.publish(trajectory_msg)

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
        marker_size_msg = Float32()
        marker_size_msg.data = marker_size
        self.marker_size_publisher.publish(marker_size_msg)

        msg = "Drone Home 0" if aruco_id == 0 else "Robot Home 1" if aruco_id == 1 else "Home Landmark 2" if aruco_id == 2 else "Robot 3"
        self.get_logger().info(f"Published ArUco ID: {msg}.")

    def publish_active_cam_color(self, is_active_cam_color):
        is_active_cam_color_msg = Bool()
        is_active_cam_color_msg.data = True if is_active_cam_color else False
        self.is_active_cam_color_publisher.publish(is_active_cam_color_msg)

    def arm_drone(self):
        """Command the drone to arm."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Drone armed.")

    def publish_takeoff(self):
        """Send takeoff command to PX4."""
        # Takeoff command (mode 3 corresponds to takeoff mode)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, SWAP_TO_SUB_VEHICLE_MODE, SUB_VEHICLE_MODE_TAKEOFF) # original take off altitude
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

    def pre_flight_check(self):
        # if self.ultrasonic_left_range32 != None:
            # if self.ultrasonic_right_range34 != None:
                if self.above_ground_altitude != None:
                    if self.solar_panel_angle_in_rad != None:
                        return True
                    else:
                        self.get_logger().info(f"solar_panel_angle not ready, set to 0")
                        self.solar_panel_angle_in_rad = 0.0
                        return True
                else:
                    self.get_logger().info(f"lidar range not ready")
        #     else:
        #         self.get_logger().info(f"ultrasonic range right34 not ready")
        # else:
        #     self.get_logger().info(f"ultrasonic range left32 not ready")

    def calculate_destination(self, x0, y0, theta, phi, d):
        """
        Calculate the destination coordinates based on the drone's current position, yaw angle, relative angle, and distance.

        Parameters:
            x0 (float): Drone's current x-coordinate.
            y0 (float): Drone's current y-coordinate.
            theta (float): Drone's current yaw angle in degrees (0° = north, 90° = east).
            phi (float): Relative angle to the destination in degrees (e.g., +90° for right-hand side).
            d (float): Distance to the destination in meters.

        Returns:
            tuple: (x, y) coordinates of the destination.
        """

        # Convert angles from degrees to radians
        theta_rad = math.radians(theta)
        phi_rad = math.radians(phi)

        # Calculate the destination angle relative to the x-axis
        destination_angle = theta_rad + phi_rad
        # Calculate the x and y displacements
        delta_x = d * math.cos(destination_angle)
        delta_y = d * math.sin(destination_angle)

        # Calculate the destination coordinates
        x = x0 + delta_x
        y = y0 + delta_y

        return x, y

    def timer_callback(self):        
        """Main loop that implements the state machine."""
        if self.state == "SERVICE_SELECT":
            if not self.loop_once:
                self.hover_start_time = time.time()
                self.loop_once = True
            if time.time() - self.hover_start_time >= 1.5:
                # if (True):                      # SIM
                if (self.pre_flight_check()): # ACTUAL
                    servo_msg = Int32()
                    servo_msg.data = RELEASE
                    self.servo_command_publisher.publish(servo_msg)
                    
                    self.origin_position[0] = round(self.current_position[0], 1)
                    self.origin_position[1] = round(self.current_position[1], 1)

                    self.anchor_position[0] = self.origin_position[0]
                    self.anchor_position[1] = self.origin_position[1]

                    # # define waypoint to solar panel
                    x, y =self.calculate_destination(x0=self.origin_position[0], y0=self.origin_position[1], theta=math.degrees(self.current_yaw), phi=90.0, d=self.waypoint_solar_panel_distance)
                    # x, y =self.calculate_destination(x0=self.origin_position[0], y0=self.origin_position[1], theta=math.degrees(self.current_euler[2]), phi=90.0, d=self.waypoint_solar_panel_distance)
                    self.waypoint_solar_panel[0] = x
                    self.waypoint_solar_panel[1] = y

                    self.get_logger().info(f"!==ZephyrDock==\nsolar panel=({self.waypoint_solar_panel[0]}, {self.waypoint_solar_panel[1]}, {self.waypoint_solar_panel[2]})")
                    
                    self.anchor_position[3] = self.current_yaw
                    # self.anchor_position[3] = self.current_euler[2]

                    # # define waypoint to home
                    self.waypoint_home[0] = self.origin_position[0]
                    self.waypoint_home[1] = self.origin_position[1]

                    self.lidar_ground_level = self.above_ground_altitude
                    self.origin_position[2] = self.lidar_ground_level

                    self.get_logger().info(f"origin=({self.origin_position[0]}, {self.origin_position[1]}, {self.origin_position[2]}, yaw = {math.degrees(self.current_yaw)})")

                    self.get_logger().info(f"\norigin above ground altitude = {self.lidar_ground_level}")
                    self.service_mode = input("Input 'd' to deploy, 'r' to return robot: ").upper()

                    if self.service_mode == 'D' or self.service_mode == 'R':
                        self.state = "ARMING"
                        self.hover_start_time = None
                        self.loop_once = False
            else:
                pass
        

        elif self.state == "ARMING":
            if not self.loop_once:
                if (self.current_mode != 4 and self.current_mode != 18 and self.current_mode != 5):
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                self.loop_once = True
            if not self.armed:              # ensure arm and take off (repeatedly send signal)
                self.arm_drone()
                self.publish_takeoff()
            elif self.hover_start_time is None:
                self.hover_start_time = time.time()
            elif time.time() - self.hover_start_time >= 10.0:
                self.state = "OFFBOARDTAKEOFF"
                self.loop_once = False
                self.hover_start_time = None
                self.anchor_position[3] = self.anchor_position[3] + math.radians(90)  # right turn 90 deg (cw) facing solar panel
                self.anchor_position[2] = self.origin_position[2]
                self.waypoint_solar_panel[2] = self.origin_position[2] + self.search_altitude
                self.waypoint_home[2] = self.origin_position[2] + self.search_altitude

        elif self.state == "OFFBOARDTAKEOFF":
            if not self.current_mode == 14:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, MAIN_VEHICLE_MODE_OFFBOARD)
            if not self.loop_once:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, MAIN_VEHICLE_MODE_OFFBOARD)
                self.publish_offboard_control_mode()
                self.loop_once = True
                if self.service_mode == 'D':
                    takeoff_altitude = self.pre_home_descend_altitude
                else:
                    takeoff_altitude = self.search_altitude # waypoint to solar panel (return robot)
                z = self.origin_position[2] + takeoff_altitude

                self.get_logger().info(f"Current odometry z: {self.above_ground_altitude} Target odometery z: {z}m")
                self.publish_trajectory_setpoint(x=self.origin_position[0], y=self.origin_position[1], z=z, yaw=self.anchor_position[3])

            if self.service_mode == 'D':
                takeoff_altitude = self.pre_home_descend_altitude
            else:
                takeoff_altitude = self.search_altitude # waypoint to solar panel (return robot)
            z = self.origin_position[2] + takeoff_altitude

            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.origin_position[0], y=self.origin_position[1], z=z, yaw=self.anchor_position[3])
            
            
            if abs(self.above_ground_altitude - z) <= 0.2:  # Allow small tolerance
                isgood = True
            else:
                isgood = False
                
            if self.stable_check(isgood):
                self.state = "HOVER"
                self.loop_once = False
                # if self.service_mode == "R":
                #     self.anchor_position[3] = self.anchor_position[3] + math.radians(90)
                

        elif self.state == "HOVER":
            if not self.loop_once:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, SWAP_TO_SUB_VEHICLE_MODE, SUB_VEHICLE_MODE_LOITER) # Loiter mode
                self.loop_once = True
                self.hover_start_time = time.time()
                self.get_logger().info("Hovering 2 sec, calling next action")
                if not self.drone_return and self.service_mode == "D":
                    self.publish_active_cam_color(False)
                    self.publish_aruco_info(3)
            if time.time() - self.hover_start_time > 2.0:  # Hover for 2 seconds
                self.hover_start_time = None
                if self.drone_return:
                    self.state = "PRE_HOME_DESCEND"
                    self.publish_aruco_info(0)
                    # self.publish_aruco_info(1)
                elif self.service_mode == "D":
                    self.state = "CUSTOM_PRECISION_DESCEND"
                    # self.publish_aruco_info(1)          # SIM
                    # self.publish_aruco_info(3)        # ACTUAL
                elif self.service_mode == "R":
                    self.state = "WAYPOINT_SOLAR_PANEL"
                    self.publish_aruco_info(3)
                    if self.gripper_gripped:
                        self.state = "PRE_HOME_DESCEND"
                        self.publish_aruco_info(1)
                        
                self.loop_once = False
                self.hover_start_time = None


        elif self.state == "PRE_HOME_DESCEND":
            # descend_rate = 0.2
            if (self.current_mode != 14 and self.current_mode != 18 and self.current_mode != 5):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, MAIN_VEHICLE_MODE_OFFBOARD)  # Switch to Offboard mode
            if not self.loop_once:
                self.anchor_position[0] = self.origin_position[0]
                self.anchor_position[1] = self.origin_position[1]
                self.anchor_position[2] = self.origin_position[2] + self.search_altitude
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, MAIN_VEHICLE_MODE_OFFBOARD)  # Switch to Offboard mode
                self.loop_once = True
                self.get_logger().info("Initiate Phase 1 Descent.")
                z = self.origin_position[2] + self.pre_home_descend_altitude
                self.get_logger().info(f"Current altitude: {self.above_ground_altitude} Target altitude: {z}m")
            z = self.origin_position[2] + self.pre_home_descend_altitude
            if self.above_ground_altitude - z >= -0.1:
                self.state = "CUSTOM_PRECISION_DESCEND"
                self.loop_once = False
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=z, yaw=self.anchor_position[3])


        elif self.state == "CUSTOM_PRECISION_DESCEND":  # if id=0 land, else hover
            if (self.current_mode != 23 and self.current_mode != 18 and self.current_mode != 5):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, SWAP_TO_SUB_VEHICLE_MODE, SUB_VEHICLE_MODE_CUSTOM_MODE)  # Switch to custom precision land (hover) mode
            if not self.loop_once:
                # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, SWAP_TO_SUB_VEHICLE_MODE, SUB_VEHICLE_MODE_CUSTOM_MODE)  # Switch to custom precision land (hover) mode
                self.custom_mode_done = False
                self.loop_once = True
                self.get_logger().info("Initiate precision landing (align to aruco marker).")
            
            if (self.aruco_id == 0):
                self.state = "COMPLETE"
                self.aruco_descend = True
                self.get_logger().info("Precision Landing on Drone Home automatically...")

            elif self.custom_mode_done:  # Wait for custom precision land (hover) mode completion signal
                self.anchor_position[3] = self.current_yaw
                # self.anchor_position[3] = self.current_euler[2]
                self.state = "SERVO_ACTION"
                self.aruco_descend = True
                self.loop_once = False
        

        elif self.state == "SERVO_ACTION":
            # Publish servo command
            servo_msg = Int32()
            if not self.loop_once:
                if not self.aruco_descend:
                    self.anchor_position[0] = self.current_position[0]
                    self.anchor_position[1] = self.current_position[1]
                    # self.anchor_position[2] = self.current_position[2]
                    self.anchor_position[2] = self.above_ground_altitude
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, MAIN_VEHICLE_MODE_OFFBOARD)  # Switch to Offboard mode
                    self.publish_offboard_control_mode()
                    self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.anchor_position[2], yaw=self.anchor_position[3])
                    self.get_logger().info(f"Current altitude: {self.above_ground_altitude} Target altitude: {self.anchor_position[2]}m")
                self.loop_once = True
                self.hover_start_time = time.time()
                if not self.gripper_gripped:
                    servo_msg.data = GRIP_STRONG
                    self.gripper_gripped = True
                    # self.hori_grip_height = self.current_position[2]      # SIM
                    self.hori_grip_height = self.above_ground_altitude      # ACTUAL
                    self.get_logger().info("Grip - servo command published.")
                else:
                    servo_msg.data = RELEASE
                    self.gripper_gripped = False
                    self.drone_return = True 
                    self.get_logger().info("Release - servo command published.")
                self.servo_command_publisher.publish(servo_msg)
            elif (time.time() - self.hover_start_time) >= 1.0 and self.gripper_retry:
                self.gripper_retry = False
                servo_msg.data = GRIP_STRONG
                self.gripper_gripped = True
                self.drone_return = False 
                self.get_logger().info("Grip - servo command published.")
                self.servo_command_publisher.publish(servo_msg)
            elif time.time() - self.hover_start_time >= 3.0:
                self.anchor_position[0] = self.current_position[0]
                self.anchor_position[1] = self.current_position[1]
                # self.anchor_position[2] = self.current_position[2]
                self.anchor_position[2] = self.above_ground_altitude
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, MAIN_VEHICLE_MODE_OFFBOARD)  # Switch to Offboard mode
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.anchor_position[2], yaw=self.anchor_position[3])  # should I use offboard?
                self.state = "ASCEND"
                self.publish_active_cam_color(True)
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
            z = self.origin_position[2] + takeoff_altitude
            # self.get_logger().info(f"Current: {self.current_position[2]} Target: {takeoff_altitude}")
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=z, yaw=self.anchor_position[3])  # Ascend to takeoff altitude
            if abs(self.above_ground_altitude - z) <= 1.5:
                if not self.loop_once:
                    if self.drone_return and self.waypoint == "HOME":
                        self.anchor_position[3] = self.anchor_position[3] - math.radians(90)
                    self.loop_once = True
            if abs(self.above_ground_altitude - z) <= 0.2:  # Allow small tolerance
                isgood = True
            else:
                isgood = False
            
            if self.stable_check(isgood):
                grip_check = False
                if self.gripper_gripped:
                    if (False):                  # SIM
                    # if not self.isloaded:     # ACTUAL
                        self.state = "CUSTOM_PRECISION_DESCEND"
                        self.gripper_retry = True
                        self.publish_aruco_info(3)
                        self.publish_active_cam_color(False)
                        self.get_logger().info("!!!\n!!!\nPayload gripping fail, retrying...\n")
                    else:
                        grip_check = True
                else:
                    grip_check = True
                if grip_check:
                    if self.waypoint == "SOLAR PANEL": # far from home
                        self.state = "WAYPOINT_HOME"
                        if not self.gripper_gripped:
                            self.anchor_position[3] = self.anchor_position[3] - math.radians(90)
                    elif self.drone_return:
                        self.state = "CUSTOM_PRECISION_DESCEND"
                        self.publish_aruco_info(0)
                        # self.publish_aruco_info(1)
                    elif self.waypoint == "HOME":
                        self.state = "WAYPOINT_SOLAR_PANEL"
                    self.loop_once = False
                # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)  # Switch to Loiter (Hold) mode
                # time.sleep(5)
        

        elif self.state == "WAYPOINT_SOLAR_PANEL":
            if not self.loop_once:
                self.anchor_position[0] = self.waypoint_solar_panel[0]
                self.anchor_position[1] = self.waypoint_solar_panel[1]
                self.anchor_position[2] = self.waypoint_solar_panel[2]
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, MAIN_VEHICLE_MODE_OFFBOARD)  # Switch to Offboard mode
                self.loop_once = True
                self.waypoint = "SOLAR PANEL"
                self.get_logger().info("Initiate waypoint to Solar Panel")
                self.get_logger().info(f"Current altitude: {self.above_ground_altitude} Target altitude: {self.anchor_position[2]}m")
            
            self.publish_trajectory_setpoint(x=self.waypoint_solar_panel[0], y=self.waypoint_solar_panel[1], z=self.waypoint_solar_panel[2], yaw=self.anchor_position[3])
            self.publish_offboard_control_mode()

            if abs(self.current_position[0] - self.waypoint_solar_panel[0]) <= 0.1 and abs(self.current_position[1] - self.waypoint_solar_panel[1]) <= 0.1:  # Allow small tolerance
                isgood = True
            else:
                isgood = False
                
            if self.stable_check(isgood):
                if self.service_mode == 'D':
                    self.state = "DEPLOY_DESCEND"
                    self.get_logger().info("Start to deploy (blinded descend)")

                    # self.state = "ALIGN_SOLAR_PANEL"
                    # Begin aligning with solar panel (yawing)
                    # self.get_logger().info("Yawing until ultrasonic sensor readings tally...")
                    self.get_logger().info(f"Target altitude: {self.origin_position[2] + self.hori_grip_height + self.lidar_gripper_offset_front*math.tan(self.solar_panel_angle_in_rad) + self.deploy_altitude}")
                else:
                    self.publish_active_cam_color(False)
                    self.state = "CUSTOM_PRECISION_DESCEND" #id3
                self.loop_once = False
            

        # elif self.state == "ALIGN_SOLAR_PANEL":
        #     if not self.loop_once:
        #         self.hover_start_time = time.time()
        #         self.loop_once = True
        #         self.get_logger().info("Payload aligning with slope, drone yawing")
        #     # Assuming you already have the sensor readings as variables `left_sensor` and `right_sensor`
        #     # These sensor values will be set through the sensor callback

        #     # If the ultrasonic readings match, proceed
        #     # if abs(self.ultrasonic_left_range32 - self.ultrasonic_right_range34) <= 2:        # ACTUAL
        #     if time.time() - self.hover_start_time >= 10:       # SIM
        #         self.state = "DEPLOY_DESCEND"
        #         self.hover_start_time = None
        #         self.loop_once = False
        # !!!!!!!!!! self.current euler changed
        #         self.anchor_position[3] = self.current_euler[2]
        #         self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.waypoint_solar_panel[2], yaw = self.anchor_position[3])
        #         self.get_logger().info("Descending for payload release")
        #         # self.get_logger().info("Ultrasonic readings matched, proceeding to deploy.")
        #     #     if self.hover_start_time is None:
        #     #         # If it's the first time descend_rate is 0, store the time
        #     #         self.hover_start_time = time.time()
        #     #     else:
        #     #         # If it's already been 3 seconds since descend_rate became 0
        #     #         if time.time() - self.hover_start_time >= self.time_threshold:
        #     #             self.state = "HOVER"
        #     #             self.hover_start_time = None
        #     # else:
        #     #     self.hover_start_time = None  # Reset the timer if descend_rate is not 0
        #     else:
        #         # Continue yawing slowly if sensor readings don't match
        #         yaw_rate = 0.05  # Set the slow yaw rate (you can adjust this value)
        #         self.publish_offboard_control_mode()
        #         self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.waypoint_solar_panel[2], yaw=self.current_euler[2] + yaw_rate)
        #         # self.get_logger().info(f"Yawing: current yaw = {math.degrees(self.current_euler[2])}°, yaw rate = {math.degrees(yaw_rate)}°/signal")
        #         # Optionally add a timeout to prevent infinite yawing if needed
        #         if time.time() - self.hover_start_time > 60:  # If yawing exceeds 60 seconds, stop and check
        #             self.state = "WAYPOINT_SOLAR_PANEL"
        #             self.get_logger().warn("Yawing exceeded time limit, checking sensor again.")
        #             self.hover_start_time = None
        #     self.publish_offboard_control_mode()


        elif self.state == "DEPLOY_DESCEND":
            if not self.loop_once:
                self.loop_once = True
                self.deploy_altitude = self.hori_grip_height + self.lidar_gripper_offset_front * math.tan(self.solar_panel_angle_in_rad) + self.z_offset
                self.get_logger().info(f"Deploy Altitude: {self.deploy_altitude}")

            altitude_difference = self.deploy_altitude - self.above_ground_altitude
            speed = 0.5  # Default slow descent
            isgood = False
            if altitude_difference > 4.0:  # Much higher than target
                speed = 2.0/2  # Faster descent, but not too fast
            elif altitude_difference > 1.5:  # Moderately higher
                speed = 1.5/2  
            elif altitude_difference > 0.5:  # Slightly higher
                speed = 1.3/2  
            elif altitude_difference > 0.1:  # Close to target
                speed = 0.5/2  
            elif altitude_difference < -0.1:  # If too low, ascend
                speed = 2.0/2  
                self.get_logger().info("Too low, ascending slightly...")
            else:  # At the correct altitude
                speed = 0.3/2
                isgood = True
                self.get_logger().info("Holding at deployment altitude.")

            # Compute target z to maintain gradual descent and corrections
            z = self.above_ground_altitude + altitude_difference  

            # Publish trajectory setpoint
            self.publish_trajectory_setpoint(x=self.anchor_position[0], 
                                            y=self.anchor_position[1], 
                                            z=z, 
                                            yaw=self.anchor_position[3], 
                                            speed=speed)


            self.publish_offboard_control_mode()
            # self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.origin_position[2] - 6.0, yaw = self.anchor_position[3])   ## ATTENTION
            # instant altitude + descend rate -> more positive -> more low altitude
            # self.get_logger().info(f"Current altitude NED: {self.current_position[2]}")
                
            if self.stable_check(isgood):
                self.state = "SERVO_ACTION"
                self.aruco_descend = False
                self.loop_once = False


        elif self.state == "WAYPOINT_HOME":
            if not self.loop_once:
                self.loop_once = True
                self.get_logger().info("Waypoint to Home")
                self.get_logger().info(f"Current altitude: {self.above_ground_altitude} Target altitude: {self.waypoint_home[2]}m")
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.waypoint_home[0], y=self.waypoint_home[1], z=self.waypoint_home[2], yaw = self.anchor_position[3])
            
            if abs(self.current_position[0]-self.waypoint_home[0]) <= 0.1 and abs(self.current_position[1]-self.waypoint_home[1]) <= 0.1:
                isgood = True
            else:
                isgood = False

            if self.stable_check(isgood):
                self.state = "HOVER"
                self.waypoint = "HOME"
                self.loop_once = False
                        
                        
        elif self.state == "COMPLETE":
            if abs(self.above_ground_altitude - self.origin_position[2]) <= 0.5:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, SWAP_TO_SUB_VEHICLE_MODE, SUB_VEHICLE_MODE_LAND)  # Land
                if not self.armed:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, SWAP_TO_SUB_VEHICLE_MODE, SUB_VEHICLE_MODE_LOITER)  # Loiter
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
        # node.odom_log.close()  # Ensure file is properly closed
        # Check if ROS is still running before shutting down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
