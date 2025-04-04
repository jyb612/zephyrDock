import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import serial

class TFMiniLidarNode(Node):
    def __init__(self):
        super().__init__('tfmini_lidar_node')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Or RELIABLE if critical
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5  # Moderate buffer for sensor data
        )

        # Initialize publisher for distance and signal strength
        self.distance_publisher = self.create_publisher(Float32, '/tfmini/range', sensor_qos)
        self.strength_publisher = self.create_publisher(Float32, '/tfmini/strength', sensor_qos)

        # Setup serial communication
        self.serial_port = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
        self.timer = self.create_timer(0.1, self.get_tfmini_data)  # 10Hz polling rate

    def get_tfmini_data(self):
        """ Reads data from TF Mini LiDAR and publishes distance and signal strength. """
        if self.serial_port.in_waiting >= 9:  # At least 9 bytes required
            recv = self.serial_port.read(9)
            self.serial_port.reset_input_buffer()

            if recv[0] == 0x59 and recv[1] == 0x59:  # Data packet start
                distance = recv[2] + (recv[3] << 8)  # Convert bytes to integer (low + high byte)
                strength = recv[4] + (recv[5] << 8)  # Signal strength

                # Publish distance
                distance_msg = Float32()
                distance_msg.data = distance / 100.0  # Convert cm to meters
                self.distance_publisher.publish(distance_msg)

                # Publish signal strength
                strength_msg = Float32()
                strength_msg.data = float(strength)
                self.strength_publisher.publish(strength_msg)

                # Logging output
                self.get_logger().info(f"Distance: {distance_msg.data:.2f} m, Strength: {strength}")

def main():
    rclpy.init()
    node = TFMiniLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
