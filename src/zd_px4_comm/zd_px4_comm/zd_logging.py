import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from std_msgs.msg import Int32, Bool, Float32, Float32MultiArray  # For servo command
import math
import numpy as np
import csv
from datetime import datetime
import os


class ZDLoggingNode(Node):
    def __init__(self):
        super().__init__('zd_logging_node')
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
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, px4_qos)
        # self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, px4_qos)
        self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.trajectory_setpoint_callback, critical_qos)

        self.create_subscription(Bool, '/precision_hovering_done', self.precision_hovering_done_callback, critical_qos)
        # self.create_subscription(Bool, '/isloaded', self.isloaded_callback, critical_qos)
        self.create_subscription(Float32, '/tfmini/range', self.lidar_range_callback, sensor_qos)
        self.create_subscription(Int32, '/aruco_id', self.aruco_id_callback, critical_qos)
        self.create_subscription(Int32, '/servo_command', self.servo_command_callback, critical_qos)
        self.create_subscription(Bool, '/is_active_cam_color', self.is_active_cam_color_callback, critical_qos)
        self.create_subscription(Point, '/setpoint_logger', self.setpoint_logger_callback, critical_qos)

        # self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, px4_qos)
        # self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, px4_qos)
        # self.create_subscription(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.trajectory_setpoint_callback, critical_qos)

        # self.create_subscription(Bool, '/precision_hovering_done', self.precision_hovering_done_callback, critical_qos)
        # # self.create_subscription(Bool, '/isloaded', self.isloaded_callback, critical_qos)
        # self.create_subscription(Float32, '/tfmini/range', self.lidar_range_callback, sensor_qos)
        # self.create_subscription(Int32, '/aruco_id', self.aruco_id_callback, critical_qos)
        # self.create_subscription(Int32, '/servo_command', self.servo_command_callback, critical_qos)
        # self.create_subscription(Bool, '/is_active_cam_color', self.is_active_cam_color_callback, critical_qos)

        self.current_position = [0.0, 0.0, 0.0, 0.0]
        self.trajectory_setpoint = [0.0, 0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0, 0.0]
        self.above_ground_altitude = None  # To store the current altitude
        self.aruco_id = 0
        self.current_mode = None
        self.custom_mode_done = False
        self.is_active_cam_color = True
        self.armed = False
        self.servo_position = 2048
        # self.isloaded = False

        # Create logs directory if needed
        os.makedirs('logs', exist_ok=True)

        # Open CSV file with additional lidar column
        log_filename = f"logs/zd_logging_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.odom_log = open(log_filename, 'w')
        self.odom_writer = csv.writer(self.odom_log)
        self.odom_writer.writerow(['timestamp', 
                                   'x', 
                                   'y', 
                                   'z', 
                                   'yaw', 
                                   'lidar_altitude', 
                                   'traj_x', 
                                   'traj_y', 
                                   'traj_z', 
                                   'traj_yaw',
                                   'velo_x',
                                   'velo,y',
                                   'velo_z',
                                   'velo_yaw', 
                                   'mode', 
                                   'arm', 
                                   'aruco id', 
                                   'servo pos', 
                                   'precision descend done'])
        self.get_logger().info(f"Logging data to: {os.path.abspath(log_filename)}")
    
    def lidar_range_callback(self, msg):
        self.above_ground_altitude = -float(msg.data)   # negative sign for FRD NED coordinate system

    # def isloaded_callback(self, msg):
    #     self.isloaded = True if msg.data else False
  
    
    def local_position_callback(self, msg):
        """Callback to update the current position from vehicle_local_position."""
        
        self.odom_log.flush()  # Ensure data is written immediately

        # Check if position estimates are valid before using them
        if not (msg.xy_valid and msg.z_valid):
            self.get_logger().warn("Position estimate not valid!")
            return

        # Store NED position (X, Y, Z)
        self.current_position = [
            float(msg.x),  # North (X) in meters
            float(msg.y),  # East (Y) in meters
            float(msg.z),  # Down (Z) in meters (negative for altitude)
            float(msg.heading),
        ]

        # Simple logging - just position and timestamp
        self.odom_writer.writerow([
            time.time(), 
            self.current_position[0],       # current odometry x (NED) (float32)
            self.current_position[1],       # current odometry y (NED) (float32)
            self.current_position[2],       # current odometry z (NED) (float32)
            self.current_position[3],       # current yaw (0 = North, 90 = East, -90 = West) (float32)
            self.above_ground_altitude,     # current lidar range (height to nearest ground) (float32)
            self.trajectory_setpoint[0],    # next trajectory setpoint odometry x (NED) (float32)
            self.trajectory_setpoint[1],    # next trajectory setpoint odometry y (NED) (float32)
            self.trajectory_setpoint[2],    # next trajectory setpoint odometry z (NED) (float32)
            self.trajectory_setpoint[3],    # next trajectory setpoint yaw (float32)
            self.velocity[0],               # vx
            self.velocity[1],               # vy
            self.velocity[2],               # vz
            self.velocity[3],               # yawspeed
            self.current_mode,              # current nav_state (int)
            self.armed,                     # arm state (bool)
            self.aruco_id,                  # current target ArUco marker id (int)
            self.servo_position,            # current serial bus servo position (int)
            self.custom_mode_done,          # custom mode status (bool)
        ])

        # # Store velocities (if needed)
        # self.current_velocity = [
        #     float(msg.vx),  # North velocity
        #     float(msg.vy),  # East velocity
        #     float(msg.vz),  # Down velocity
        # ]

        # Store yaw (heading) in radians
        # self.current_yaw = float(msg.heading)

    # def odometry_callback(self, msg):
    #     """Callback to update the current position."""
        

    #     self.odom_log.flush()  # Ensure data is written immediately
        
    #     # Extract quaternion
    #     q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        
    #     # # Convert quaternion to Euler angles (roll, pitch, yaw)
    #     # _,_, self.current_position[3] = self.quaternion_to_euler(q)
    #     # yaw = -180 ~ 180
    #     self.current_position = [
    #         float(msg.position[0]),  # X in NED
    #         float(msg.position[1]),  # Y in NED
    #         float(msg.position[2]),  # Z in NED
    #         self.quaternion_to_euler(q)[2],
    #     ]

    #     # Simple logging - just position and timestamp
    #     self.odom_writer.writerow([
    #         time.time(), 
    #         self.current_position[0],       # current odometry x (NED) (float32)
    #         self.current_position[1],       # current odometry y (NED) (float32)
    #         self.current_position[2],       # current odometry z (NED) (float32)
    #         self.current_position[3],       # current yaw (0 = North, 90 = East, -90 = West) (float32)
    #         self.above_ground_altitude,     # current lidar range (height to nearest ground) (float32)
    #         self.trajectory_setpoint[0],    # next trajectory setpoint odometry x (NED) (float32)
    #         self.trajectory_setpoint[1],    # next trajectory setpoint odometry y (NED) (float32)
    #         self.trajectory_setpoint[2],    # next trajectory setpoint odometry z (NED) (float32)
    #         self.trajectory_setpoint[3],    # next trajectory setpoint yaw (float32)
    #         self.current_mode,              # current nav_state (int)
    #         self.armed,                     # arm state (bool)
    #         self.aruco_id,                  # current target ArUco marker id (int)
    #         self.servo_position,            # current serial bus servo position (int)
    #         self.custom_mode_done,          # custom mode status (bool)
    #     ])

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

    def aruco_id_callback(self, msg):
        self.aruco_id = int(msg.data)
    
    def servo_command_callback(self, msg):
        self.servo_position = int(msg.data)

    def is_active_cam_color_callback(self, msg):
        self.is_active_cam_color = msg.data
    
    def trajectory_setpoint_callback(self, msg):
        """Callback to update the current position."""
        # self.get_logger().info("trajectory setpoint yes")
        self.velocity[0] = float(msg.velocity[0])
        self.velocity[1] = float(msg.velocity[1])
        self.velocity[2] = float(msg.velocity[2])
        self.velocity[3] = float(msg.yawspeed)
        self.trajectory_setpoint[3] = float(msg.yaw)
    
    def setpoint_logger_callback(self, msg):
        self.trajectory_setpoint[0] = msg.x
        self.trajectory_setpoint[1] = msg.y
        self.trajectory_setpoint[2] = msg.z
        


def main(args=None):
    rclpy.init(args=args)
    node = ZDLoggingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
    finally:
        node.destroy_node()
        node.odom_log.close()  # Ensure file is properly closed
        # Check if ROS is still running before shutting down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()