import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
import pandas as pd
import os
from datetime import datetime

class ZDCalNode(Node):
    def __init__(self):
        super().__init__('px4_control')

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
        
        # Subscribe to current position
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
        )

        # State variables
        self.current_position = [0.0, 0.0, 0.0]  # Current position in NED
        self.current_yaw = 0.0  # Current yaw angle (radians)
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.current_mode = None  # Current flight mode
        self.armed = False  # Armed state
        self.state = "ARMING"  # State machine state
        self.takeoff_altitude = -8.0  # Takeoff altitude in NED (8 meters up)
        self.hover_start_time = None  # Time when hovering starts
        self.movement_start_time = None  # Time when movement starts
        self.running = True
        self.initial_yaw = None  # Will store the yaw at takeoff completion
        
        # Z-position controller variables
        self.initial_z_position = None  # To store the initial z position
        self.z_kp = 1.0  # Proportional gain for z controller
        self.maintain_z = False  # Flag to indicate whether to maintain z position
        self.moving_in_z = False  # Flag to indicate explicit z movement
        
        # Timer to control the state machine
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # Teraranger LIDAR Subscription
        # Create subscriber
        self.create_subscription(
            Float32MultiArray,
            '/teraranger_evo/distances',
            self.teraranger_callback,
            10
        )
        
        # Define sensor positions [front, right, back, left]
        self.directions = ['front', 'right', 'back', 'left']
        self.processed_distances = [float('inf'), float('inf'), float('inf'), float('inf')]
        
        # Max distance when sensor returns inf (in meters)
        self.lidar_max_distance = 60.0
        
        # Obstacle avoidance parameters
        self.safe_distance = 5.0  # Distance threshold in meters to start slowing down
        self.min_distance = 2.0   # Minimum distance for complete stop
        
        # Maintain distance parameters
        self.maintain_safe_distance = 2.0  # Safe distance to maintain in meters
        self.distance_kp = 0.5    # Proportional gain for maintain distance controller
        
        # PID controller parameters for velocity control
        self.kp = 0.3  # Proportional gain
        self.ki = 0.05 # Integral gain
        self.kd = 0.1  # Derivative gain
        
        # PID controller state variables for each direction
        self.prev_errors = {'front': 0.0, 'right': 0.0, 'back': 0.0, 'left': 0.0}
        self.integral_terms = {'front': 0.0, 'right': 0.0, 'back': 0.0, 'left': 0.0}
        self.last_time = {'front': None, 'right': None, 'back': None, 'left': None}
        
        # Data logging variables
        self.current_velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz]
        self.data_log = []
        self.log_timer = self.create_timer(0.1, self.log_data_callback)  # 10Hz data logging
        
        # Create directory for data logs if it doesn't exist
        self.log_dir = os.path.expanduser('~/drone_data_logs')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Create a timestamp for the log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f'drone_data_{timestamp}.xlsx')
        
        # Log the file location
        self.get_logger().info(f"Data will be logged to: {self.log_file}")
    
    def log_data_callback(self):
        """Collect data at regular intervals"""
        current_time = time.time()
        
        # Create data row
        data_row = {
            'timestamp': current_time,
            'state': self.state,
            'position_x': self.current_position[0],
            'position_y': self.current_position[1],
            'position_z': self.current_position[2],
            'velocity_x': self.current_velocity[0],
            'velocity_y': self.current_velocity[1],
            'velocity_z': self.current_velocity[2],
            'yaw': self.current_yaw,
            'distance_front': self.processed_distances[0] if len(self.processed_distances) > 0 else float('inf'),
            'distance_right': self.processed_distances[1] if len(self.processed_distances) > 1 else float('inf'),
            'distance_back': self.processed_distances[2] if len(self.processed_distances) > 2 else float('inf'),
            'distance_left': self.processed_distances[3] if len(self.processed_distances) > 3 else float('inf'),
        }
        
        # Append to the data log
        self.data_log.append(data_row)

        # Periodically print the size of the data log
        if len(self.data_log) % 10 == 0:  # Every 10 entries
            self.get_logger().info(f"Data log size: {len(self.data_log)} entries")
        
    def save_data_to_excel(self):
        """Save collected data to Excel file"""
        self.get_logger().info(f"Attempting to save {len(self.data_log)} data entries")
    
        """Save collected data to Excel file"""
        if len(self.data_log) == 0:
            self.get_logger().warn("No data to save")
            return
            
        try:
            # Convert to DataFrame
            df = pd.DataFrame(self.data_log)
            
            # Add human-readable timestamps
            start_time = df['timestamp'].iloc[0]
            df['elapsed_time'] = df['timestamp'] - start_time
            df['datetime'] = pd.to_datetime(df['timestamp'], unit='s')
            
            # Convert radians to degrees for better readability
            df['yaw_degrees'] = df['yaw'].apply(lambda x: math.degrees(x))
            
            # Reorder columns for better readability
            columns_order = [
                'datetime', 'elapsed_time', 'state',
                'position_x', 'position_y', 'position_z',
                'velocity_x', 'velocity_y', 'velocity_z',
                'yaw', 'yaw_degrees',
                'distance_front', 'distance_right', 'distance_back', 'distance_left'
            ]
            df = df[columns_order]
            
            # Save to Excel
            df.to_excel(self.log_file, index=False, engine='openpyxl')
            self.get_logger().info(f"Data saved to {self.log_file}")
        except Exception as e:
            self.get_logger().error(f"Error saving data to Excel: {e}")
    
    def teraranger_callback(self, msg):
        # Process each distance value            
        self.processed_distances = []
        for i, distance in enumerate(msg.data):
            direction = self.directions[i]
            
            # Handle special cases
            if distance == float('-inf'):
                # Object too close - set to 0
                self.processed_distances.append(0.0)
                self.get_logger().debug(f"{direction}: Below minimum range (-inf), setting to 0.0m")
            elif distance == float('inf'):
                # Object too far - set to max distance
                self.processed_distances.append(self.lidar_max_distance)
                self.get_logger().debug(f"{direction}: Above maximum range (inf), setting to {self.lidar_max_distance}m")
            elif distance != distance:  # Check for NaN
                # Invalid reading - set to 0
                self.processed_distances.append(0.0)
                self.get_logger().debug(f"{direction}: Invalid reading (nan), setting to 0.0m")
            else:
                # Normal reading
                self.processed_distances.append(distance)
                self.get_logger().debug(f"{direction}: Valid reading: {distance:.3f}m")
        
        # Log the processed distances
        self.get_logger().info(
            f"Processed distances - Front: {self.processed_distances[0]:.2f}m, "
            f"Right: {self.processed_distances[1]:.2f}m, "
            f"Back: {self.processed_distances[2]:.2f}m, "
            f"Left: {self.processed_distances[3]:.2f}m"
        )

    def get_distance_in_direction(self, direction):
        """Get the distance from the LIDAR in a specific direction."""
        direction_index = {
            "forward": 0,  # front
            "right": 1,    # right
            "backward": 2, # back
            "left": 3      # left
        }
        
        idx = direction_index.get(direction, 0)
        return self.processed_distances[idx] if len(self.processed_distances) > idx else self.lidar_max_distance

    def calculate_velocity_with_obstacle_avoidance(self, direction, desired_speed=1.0):
        """
        Calculate velocity components with obstacle avoidance based on LIDAR readings.
        Returns velocity scaled by PID controller for collision avoidance.
        """
        # Get raw velocity components based on direction
        vx, vy = self.calculate_velocity(direction, speed=desired_speed)
        
        # Get the distance in the current movement direction
        distance = self.get_distance_in_direction(direction)
        
        # Calculate velocity scale factor using PID controller
        scale_factor = self.calculate_velocity_scale(direction, distance)
        
        # Apply scale factor to velocity components
        adjusted_vx = vx * scale_factor
        adjusted_vy = vy * scale_factor
        
        self.get_logger().info(
            f"Direction: {direction}, Distance: {distance:.2f}m, "
            f"Scale: {scale_factor:.2f}, "
            f"Velocity adjusted from ({vx:.2f}, {vy:.2f}) to ({adjusted_vx:.2f}, {adjusted_vy:.2f})"
        )
        
        return adjusted_vx, adjusted_vy

    def calculate_velocity_scale(self, direction, distance):
        """
        Calculate velocity scale factor using PID controller based on distance to obstacle.
        Returns a value between 0.0 (stop) and 1.0 (full speed).
        """
        current_time = time.time()
        
        # If distance is above safe_distance, move at full speed
        if distance >= self.safe_distance:
            # Reset PID terms when we're in safe zone
            self.prev_errors[direction] = 0.0
            self.integral_terms[direction] = 0.0
            self.last_time[direction] = current_time
            return 1.0
            
        # If distance is below min_distance, stop
        if distance <= self.min_distance:
            return 0.0
            
        # Calculate error: how far we are from safe_distance
        # As we get closer to min_distance, error grows
        error = (distance - self.min_distance) / (self.safe_distance - self.min_distance)
        
        # Initialize time if first run
        if self.last_time[direction] is None:
            self.last_time[direction] = current_time
            self.prev_errors[direction] = error
            return error  # Return proportional term only on first run
            
        # Calculate time delta
        dt = current_time - self.last_time[direction]
        if dt <= 0:
            dt = 0.05  # Use default if time hasn't advanced
            
        # Calculate PID terms
        # Proportional term
        p_term = error
        
        # Integral term with anti-windup
        self.integral_terms[direction] += error * dt
        self.integral_terms[direction] = max(0.0, min(1.0, self.integral_terms[direction]))  # Clamp
        i_term = self.ki * self.integral_terms[direction]
        
        # Derivative term
        d_term = self.kd * (error - self.prev_errors[direction]) / dt if dt > 0 else 0.0
        
        # Calculate PID output
        pid_output = self.kp * p_term + i_term + d_term
        
        # Clamp output between 0 and 1
        velocity_scale = max(0.0, min(1.0, pid_output))
        
        # Store values for next iteration
        self.prev_errors[direction] = error
        self.last_time[direction] = current_time
        
        return velocity_scale
        
    def calculate_maintain_distance_velocity(self):
        """
        Calculate velocity components to maintain safe distance from obstacles.
        Uses P controller to adjust velocity based on distance readings.
        """
        # Initialize velocity components
        vx = 0.0
        vy = 0.0
        
        # Get distances in all directions
        distances = {
            "forward": self.processed_distances[0] if len(self.processed_distances) > 0 else float('inf'),
            "right": self.processed_distances[1] if len(self.processed_distances) > 1 else float('inf'),
            "backward": self.processed_distances[2] if len(self.processed_distances) > 2 else float('inf'),
            "left": self.processed_distances[3] if len(self.processed_distances) > 3 else float('inf')
        }
        
        # Determine if any direction needs adjustment
        need_adjustment = False
        
        # Check each direction
        for direction, distance in distances.items():
            # Only adjust if distance is less than safe distance
            if distance < self.maintain_safe_distance:
                need_adjustment = True
                
                # Calculate error (how much closer than safe distance)
                error = self.maintain_safe_distance - distance
                
                # Calculate velocity component with P controller
                speed = self.distance_kp * error
                
                # Clamp speed to reasonable values
                speed = min(max(speed, 0.0), 1.0)
                
                # Calculate opposite direction to move away from obstacle
                opposite_direction = {
                    "forward": "backward",
                    "backward": "forward",
                    "right": "left",
                    "left": "right"
                }[direction]
                
                # Get velocity components for the opposite direction
                dir_vx, dir_vy = self.calculate_velocity(opposite_direction, speed=speed)
                
                # Add to total velocity components
                vx += dir_vx
                vy += dir_vy
                
                self.get_logger().info(
                    f"Adjusting for {direction} obstacle at {distance:.2f}m, "
                    f"moving {opposite_direction} with speed {speed:.2f}"
                )
        
        if not need_adjustment:
            self.get_logger().info("All directions clear, maintaining position")
            
        return vx, vy

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
 
        # Extract quaternion
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        self.current_euler = self.quaternion_to_euler(q)
        roll, pitch, yaw = self.current_euler
    
        # Store yaw separately for easy access
        self.current_yaw = yaw

        # Only set initial_yaw once after takeoff if it hasn't been set
        if self.state == "MAINTAIN_DISTANCE" and self.initial_yaw is None:
            self.initial_yaw = yaw
            self.get_logger().info(f"Initial yaw set to: {math.degrees(self.initial_yaw):.1f}°")
 
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

    def calculate_velocity(self, direction, speed=1.0):
        """
        Calculate velocity components based on initial yaw after takeoff.
        """
        # Use initial_yaw if set, otherwise use current_yaw (shouldn't happen)
        yaw = self.initial_yaw if self.initial_yaw is not None else self.current_yaw
        
        # Base direction vectors (at 0° yaw)
        base_vectors = {
            "forward": (1, 0),
            "backward": (-1, 0),
            "right": (0, 1),
            "left": (0, -1)
        }
        
        # Get the base vector
        base_vx, base_vy = base_vectors.get(direction, (0, 0))
        
        # Apply rotation
        vx = speed * (base_vx * math.cos(yaw) - base_vy * math.sin(yaw))
        vy = speed * (base_vx * math.sin(yaw) + base_vy * math.cos(yaw))
            
        self.get_logger().info(
            f"Raw movement {direction} relative to initial yaw: {(yaw):.1f} rad, {math.degrees(yaw):.1f}° | "
            f"Velocities - X: {vx:.2f}, Y: {vy:.2f}"
        )
            
        return vx, vy

    def vehicle_status_callback(self, msg):
        """Callback to update the current mode."""
        self.current_mode = msg.nav_state
        if msg.arming_state == 2:
            self.armed = True
        else:
            self.armed = False

    def publish_offboard_control_mode(self):
        """Publish OffboardControlMode message."""
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True  # Enable velocity control
        offboard_msg.acceleration = False
        self.offboard_control_mode_publisher.publish(offboard_msg)

    def publish_trajectory_setpoint(self, vx=0.0, vy=0.0, vz=0.0, yaw=None):
        """Publish a trajectory setpoint in velocity mode with z position maintenance."""
        # Calculate z velocity for position maintenance when needed
        calculated_vz = vz
        
        if self.maintain_z and not self.moving_in_z and self.initial_z_position is not None:
            z_error = self.initial_z_position - self.current_position[2]
            calculated_vz = self.z_kp * z_error
            calculated_vz = max(min(calculated_vz, 1.0), -1.0)
            
        # Publish control mode first
        self.publish_offboard_control_mode()

        # Then publish trajectory setpoint
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = vx
        trajectory_msg.velocity[1] = vy
        trajectory_msg.velocity[2] = calculated_vz
        trajectory_msg.position[0] = float('nan')
        trajectory_msg.position[1] = float('nan')
        trajectory_msg.position[2] = float('nan')
        trajectory_msg.acceleration[0] = float('nan')
        trajectory_msg.acceleration[1] = float('nan')
        trajectory_msg.acceleration[2] = float('nan')
        
        # Use current yaw if not specified
        if yaw is None:
            trajectory_msg.yaw = self.current_yaw
        else:
            trajectory_msg.yaw = yaw
            
        # Store current velocity for logging
        self.current_velocity = [vx, vy, calculated_vz]
            
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
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 2.0)
        self.get_logger().info(f"Sending takeoff command.")

    def timer_callback(self):
        """Main loop that implements the state machine."""
        if self.state == "ARMING":
            if not self.armed:
                if self.current_mode != 4:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                self.arm_drone()
                self.publish_takeoff()
            else:
                if self.current_mode != 4:
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                self.get_logger().info("Drone armed and taking off")
                time.sleep(5)
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            self.state = "MOVING_UP"
            time.sleep(10)
            self.publish_offboard_control_mode()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.movement_start_time = time.time()
            self.moving_in_z = True
            self.maintain_z = False

        elif self.state == "MOVING_UP":
            self.moving_in_z = True
            self.maintain_z = False
            self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=-1.0)
            
            if time.time() - self.movement_start_time >= 5:
                self.initial_z_position = self.current_position[2]
                self.state = "MAINTAIN_DISTANCE"  # Change to maintain distance state
                self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)
                self.movement_start_time = time.time()
                self.moving_in_z = False
                self.maintain_z = True
                
        elif self.state == "MAINTAIN_DISTANCE":
            # This is the new state for maintaining distance from obstacles
            self.moving_in_z = False
            self.maintain_z = True
            
            # Calculate velocity components to maintain safe distance
            vx, vy = self.calculate_maintain_distance_velocity()
            
            # Apply the calculated velocities
            self.publish_trajectory_setpoint(vx=vx, vy=vy, vz=0.0)
            
            # Optional: Add a transition to the next state after a certain time
            # For example, after 30 seconds of maintaining distance
            if time.time() - self.movement_start_time >= 60:
                self.state = "MOVING_BACKWARD"
                self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)
                self.movement_start_time = time.time()

        elif self.state == "MOVING_BACKWARD":
            self.moving_in_z = False
            self.maintain_z = True
            vx, vy = self.calculate_velocity_with_obstacle_avoidance("backward", desired_speed=1.0)
            self.publish_trajectory_setpoint(vx=vx, vy=vy, vz=0.0)
            
            if time.time() - self.movement_start_time >= 10:
                self.state = "MOVING_LEFT"
                self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)
                self.movement_start_time = time.time()

        elif self.state == "MOVING_LEFT":
            self.moving_in_z = False
            self.maintain_z = True
            vx, vy = self.calculate_velocity_with_obstacle_avoidance("left", desired_speed=1.0)
            self.publish_trajectory_setpoint(vx=vx, vy=vy, vz=0.0)
            
            if time.time() - self.movement_start_time >= 10:
                self.state = "MOVING_RIGHT"
                self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)
                self.movement_start_time = time.time()

        elif self.state == "MOVING_RIGHT":
            self.moving_in_z = False
            self.maintain_z = True
            vx, vy = self.calculate_velocity_with_obstacle_avoidance("right", desired_speed=1.0)
            self.publish_trajectory_setpoint(vx=vx, vy=vy, vz=0.0)
            
            if time.time() - self.movement_start_time >= 10:
                self.state = "MOVING_FORWARD"
                self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)
                self.movement_start_time = time.time()

        elif self.state == "MOVING_FORWARD":
            self.moving_in_z = False
            self.maintain_z = True
            vx, vy = self.calculate_velocity_with_obstacle_avoidance("forward", desired_speed=1.0)
            self.publish_trajectory_setpoint(vx=vx, vy=vy, vz=0.0)
            
            if time.time() - self.movement_start_time >= 10:
                self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)
                self.state = "HOVER"
                self.hover_start_time = time.time()
                self.moving_in_z = False
                self.maintain_z = True

        elif self.state == "HOVER":
            self.moving_in_z = False
            self.maintain_z = True
            self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)

            # Add periodic saving during hover
            if time.time() - self.hover_start_time >= 2.5:  # Save halfway through hover
                self.get_logger().info("Saving data during hover")
                self.save_data_to_excel()
            
            if time.time() - self.hover_start_time >= 5:
                self.state = "LANDING"
                self.maintain_z = False

        elif self.state == "LANDING":
            self

def main(args=None):
    rclpy.init(args=args)
    node = ZDCalNode()

    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
        # Save data on keyboard interrupt
        # node.save_data_to_excel()
    finally:
        node.destroy_node()
        # Check if ROS is still running before shutting down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()