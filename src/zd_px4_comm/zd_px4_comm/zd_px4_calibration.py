#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus, VehicleGlobalPosition, VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from std_msgs.msg import Int32, Bool, Float64  # For servo command
import math
import numpy as np

class ZDCalNode(Node):
    def __init__(self):
        super().__init__('zd_px4_calibration_node')

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

        self.global_sub = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_callback, qos_profile)

        self.local_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_callback, qos_profile)
        
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

        # State variables
        self.current_position = [0.0, 0.0, 0.0]  # Current position in NED
        self.current_euler = [0.0, 0.0, 0.0]
        self.anchor_position = [0.0, 0.0, 0.0, 0.0]
        self.origin_position = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.current_mode = None  # Current flight mode
        self.armed = False  # Armed state
        self.state = "PREFLIGHT"  # State machine state
        self.takeoff_altitude = -5.0  # Takeoff altitude in NED (5 meters up)
        self.mode_callback_time = None
        self.hover_start_time = None  # Time when hovering starts
        self.motion_callback_time = None
        self.hover = False
        self.yaw_angle = 0 # Instantaneous target yaw
        # self.yaw_angles = [0]
        self.yaw_angles = [45, 90, 135, 180, 225, 270, 315, 0]  # List of yaw angles for calibration
        self.current_yaw_index = 0  # To keep track of the current yaw angle in the list
        self.angular_velocity_threshold = 0.01  # Threshold for angular velocity (radians per second)
        self.loop_once = False
        self.running = True
        self.linear_x = 2.0
        self.linear_y = 2.0
        self.linear_z = -2.0

        # Timer to control the state machine
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2Hz
        
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
        # self.get_logger().info(f"\nmsg.position: ({self.current_position[0]}, {self.current_position[1]}, {self.current_position[2]})\nmsg.angular_velocity: ({self.angular_velocity[0]}, {self.angular_velocity[1]}, {self.angular_velocity[2]})")

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
        if msg.arming_state == 2:
            self.armed = True
            # self.get_logger().info(f"Current mode: {self.current_mode}")
        else:
            self.armed = False

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

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-6.0, yaw=0.0, velocity_limit=1.0):
        """Publish a trajectory setpoint with velocity control."""
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Set position target (x, y, z) - optional
        trajectory_msg.position = [x, y, z]
        trajectory_msg.yaw = yaw  # Set desired yaw

        # Calculate velocity towards the target
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        dz = z - self.current_position[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Velocity calculation to target with a defined limit
        if distance > 0:
            velocity_x = (dx / distance) * velocity_limit
            velocity_y = (dy / distance) * velocity_limit
            velocity_z = (dz / distance) * velocity_limit
        else:
            velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0

        # Set the velocity in the trajectory message
        # trajectory_msg.velocity = [velocity_x, velocity_y, velocity_z]

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
        self.get_logger().info(f"Published VehicleCommand: command={command}, param1={param1}, param2={param2}, param3={param3}")

    def arm_drone(self):
        """Command the drone to arm."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_takeoff(self):
        """Send takeoff command to PX4."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 2.0) # original take off altitude
        self.get_logger().info(f"Sending takeoff command.")

    def global_callback(self, msg):
        # self.get_logger().info(f'Global Position: Lat={msg.lat}, Lon={msg.lon}, Alt={msg.alt}')
        pass

    def local_callback(self, msg):
        # self.get_logger().info(f'Local Position: X={msg.x}, Y={msg.y}, Z={msg.z}, VX={msg.vx}, VY={msg.vy}, VZ={msg.vz}')
        pass

    
    def timer_callback(self):
        if self.state == "PREFLIGHT":
            if not self.loop_once:
                self.hover_start_time = time.time()
                self.loop_once = True
            if time.time() - self.hover_start_time >= 5:
                self.origin_position[0] = round(self.current_position[0], 1)
                self.origin_position[1] = round(self.current_position[1], 1)
                self.anchor_position[0] = self.origin_position[0]
                # self.anchor_position[0] = 0.0
                self.anchor_position[1] = self.origin_position[1]
                # self.anchor_position[1] = 0.0
                if (True):
                # if (self.pre_flight_check()):
                    self.get_logger().info(f"origin=({self.current_position[0]}, {self.current_position[1]})")
                    self.get_logger().info(f"origin=({self.anchor_position[0]}, {self.anchor_position[1]})")
                    input_check = input("Input 'c' to test").upper()
                    if input_check == 'C':
                        self.state = "ARMING"
                        self.hover_start_time = None

                else:
                    pass

        """Main loop that implements the state machine."""
        if self.state == "ARMING":  
            if not self.armed:
                if (self.current_mode != 4):
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                self.arm_drone()
                self.publish_takeoff()
                # hover_time = time.time()
            else:
                # if (self.current_mode != 4):
                #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                self.get_logger().info("Drone armed and takeoff")
                self.get_logger().info(f"origin=({self.anchor_position[0], self.anchor_position[1]})")
                time.sleep(8)
                self.anchor_position[3] = self.current_euler[2]
                self.state = "TAKEOFF"
                # pass

        elif self.state == "TAKEOFF":
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.takeoff_altitude, yaw=self.anchor_position[3])  # Start with yaw 0
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Switched to Offboard mode")
            self.get_logger().info(f"Toward altitude: {self.takeoff_altitude}")
            if abs(self.current_position[2] - self.takeoff_altitude) <= 0.1:  # Allow small tolerance
                self.state = "HOVER"
                self.hover_start_time = time.time()

        elif self.state == "HOVER":
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.takeoff_altitude, yaw=self.anchor_position[3])
            if time.time() - self.hover_start_time > 1.0:  # Hover for 3 seconds
                self.state = "YAW_CALIBRATION"
                self.current_yaw_index = 0

        elif self.state == "YAW_CALIBRATION":
            if self.current_yaw_index < len(self.yaw_angles):
                self.yaw_angle = self.yaw_angles[self.current_yaw_index]
                self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.takeoff_altitude, yaw=math.radians(self.yaw_angle))
                self.get_logger().info(f"Yawing to {self.yaw_angle} degrees.")
                self.state = "WAIT_FOR_YAW"
                self.get_logger().info("Waiting for Yaw.")
                self.hover = False
            else:
                self.anchor_position[3] = self.current_euler[2]
                self.state = "LINEAR_CALIBRATION_X"
                self.hover = False
                self.get_logger().info(f"Current Position 0 0 {self.takeoff_altitude}")

        elif self.state == "WAIT_FOR_YAW":
            # Check if the angular velocity is near zero (yaw is done)
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0], y=self.anchor_position[1], z=self.takeoff_altitude, yaw=math.radians(self.yaw_angle))
            if not self.hover:
                if all(abs(v) < self.angular_velocity_threshold for v in self.angular_velocity):
                    self.hover_start_time = time.time()
                    self.hover = True
                    self.get_logger().info(f"Yaw completed at {self.yaw_angles[self.current_yaw_index]} degrees.")
            else:
                if (time.time() - self.hover_start_time > 0.5):
                    self.state = "YAW_CALIBRATION"
                    self.current_yaw_index += 1
                    self.motion_callback_time = time.time()

        elif self.state == "LINEAR_CALIBRATION_X":
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0] + self.linear_x, y=self.anchor_position[1], z=self.takeoff_altitude, yaw=self.anchor_position[3])
            if (time.time() - self.motion_callback_time > 1):
                self.motion_callback_time = time.time()
                self.get_logger().info(f"Toward *{self.anchor_position[0]}+{self.linear_x}* {self.anchor_position[1]}] {self.takeoff_altitude}")
            if not self.hover:
                if abs(self.current_position[0] - (self.anchor_position[0]+self.linear_x)) <= 0.1:  # Allow small tolerance
                    self.hover_start_time = time.time()
                    self.hover = True
            else:
                if (time.time() - self.hover_start_time > 1.0):
                    self.state = "LINEAR_CALIBRATION_Y"
                    self.hover = False
                    self.motion_callback_time = time.time()
        
        elif self.state == "LINEAR_CALIBRATION_Y":
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(x=self.anchor_position[0] + self.linear_x, y=self.anchor_position[1] + self.linear_y, z=self.takeoff_altitude, yaw=self.anchor_position[3])
            if (time.time() - self.motion_callback_time > 1):
                self.motion_callback_time = time.time()
                self.get_logger().info(f"Toward {self.anchor_position[0] + self.linear_x} *{self.anchor_position[1]}+{self.linear_y}* {self.takeoff_altitude}")
            if not self.hover:
                if abs(self.current_position[1] - (self.anchor_position[1] + self.linear_y)) <= 0.1:  # Allow small tolerance
                    self.hover_start_time = time.time()
                    self.hover = True
            else:
                if (time.time() - self.hover_start_time > 1.0):
                    self.state = "LINEAR_CALIBRATION_Z"
                    self.hover = False
                    self.motion_callback_time = time.time()
        
        elif self.state == "LINEAR_CALIBRATION_Z":
            self.publish_offboard_control_mode()
            if (time.time() - self.motion_callback_time > 1):
                self.motion_callback_time = time.time()
                self.publish_trajectory_setpoint(x=self.anchor_position[0] + self.linear_x, y=self.anchor_position[1] + self.linear_y, z=self.takeoff_altitude+self.linear_z, yaw=self.anchor_position[3])
            self.get_logger().info(f"Toward {self.anchor_position[0] + self.linear_x} {self.anchor_position[1] + self.linear_y} *{self.takeoff_altitude}+{self.linear_z}*")
            if not self.hover:
                if abs(self.current_position[2] - (self.takeoff_altitude+self.linear_z)) <= 0.1:  # Allow small tolerance
                    self.hover_start_time = time.time()
                    self.hover = True
            else:
                if (time.time() - self.hover_start_time > 1.0):
                    self.state = "COMPLETE"
                    self.hover = False
                    self.motion_callback_time = time.time()


        elif self.state == "COMPLETE":
            self.get_logger().info("Calibration complete.")
            # Hold position if necessary or enter loiter mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)  # Loiter mode
            rtl_confirm = input("Enter 'y' to return, 'c' to exit program: ").upper()
            self.get_logger().info(f"User input: *{rtl_confirm}*")
            if (rtl_confirm == 'Y'):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 5.0)  # RTL mode
                self.get_logger().warn(f"Return and Exiting node...")
                self.running = False  # Stop the spin loop
            elif (rtl_confirm == 'C'):
                self.get_logger().warn(f"Exiting node...")
                self.running = False  # Stop the spin loop
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ZDCalNode()

    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
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
