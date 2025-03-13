import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus2
import time

# I2C Addresses of the GY-US42v2 sensors after address change
SENSOR_1_ADDRESS = 0x32  # First sensor address
SENSOR_2_ADDRESS = 0x34  # Second sensor address

class DualUltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_dual_node')

        # Initialize I2C bus 1 (on Jetson Orin Nano)
        self.bus = smbus2.SMBus(1)

        # Create ROS2 publishers for both sensors
        self.publisher_left = self.create_publisher(Float32, '/gyus42v2/left_range', 10)
        self.publisher_right = self.create_publisher(Float32, '/gyus42v2/right_range', 10)

        # Read sensors at 10Hz (100ms interval)
        self.timer = self.create_timer(0.1, self.read_sensors)

    def read_distance(self, address, label):
        """Reads distance from the GY-US42v2 ultrasonic sensor over I2C."""
        try:
            self.bus.write_byte(address, 0x51)  # Command to start measurement
            time.sleep(0.07)  # Wait for the measurement to complete

            # Request 2 bytes from the sensor (high and low byte for distance)
            distance_data = self.bus.read_i2c_block_data(address, 0x00, 2)
            distance = (distance_data[0] << 8) | distance_data[1]  # Combine bytes to get distance in cm
            return distance / 100.0  # Convert to meters
        except Exception as e:
            self.get_logger().warn(f"Error reading {label} sensor: {e}")
            return -1.0  # Return -1 if the sensor read fails

    def read_sensors(self):
        """Reads both ultrasonic sensors and publishes the data."""
        # Read from the first sensor (left sensor)
        left_distance = self.read_distance(SENSOR_1_ADDRESS, "LEFT")
        
        # Read from the second sensor (right sensor)
        right_distance = self.read_distance(SENSOR_2_ADDRESS, "RIGHT")

        # Create message objects for both sensors
        left_msg = Float32()
        right_msg = Float32()
        
        left_msg.data = left_distance
        right_msg.data = right_distance

        # Publish data
        self.publisher_left.publish(left_msg)
        self.publisher_right.publish(right_msg)

        # Log the readings
        self.get_logger().info(f"Left: {left_distance:.2f} m | Right: {right_distance:.2f} m")

def main():
    rclpy.init()
    node = DualUltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
