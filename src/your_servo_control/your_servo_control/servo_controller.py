import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from pymodbus.client import ModbusSerialClient
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Define Serial Modbus Connection
        self.servo_port = "/dev/ttyUSB1"  # Change this to the correct port
        self.baud_rate = 115200
        self.client = None
        self.connected = False

        self.POSITION_REGISTER = 257  # Register to check position

        # Attempt to connect to the servo
        self.connect_to_servo()

        if self.connected:
            # Subscribe to grip command topic
            self.create_subscription(
                Int32,
                '/servo_command',  
                self.command_callback,
                10
            )

            # Publishers for feedback data
            self.position_pub = self.create_publisher(Int32, '/servo_position', 10)
            self.speed_pub = self.create_publisher(Int32, '/servo_speed', 10)
            self.voltage_pub = self.create_publisher(Float32, '/servo_voltage', 10)
            self.temperature_pub = self.create_publisher(Int32, '/servo_temperature', 10)
            self.moving_pub = self.create_publisher(Int32, '/servo_moving', 10)
            self.current_pub = self.create_publisher(Int32, '/servo_current', 10)

            # Start a timer to read the feedback every 100ms
            self.feedback_timer = self.create_timer(0.1, self.feedback_callback)
        else:
            self.get_logger().error("Failed to connect to the servo motor.")

    def connect_to_servo(self):
        """Connects to the servo using Modbus RTU."""
        try:
            self.client = ModbusSerialClient(
                port=self.servo_port,
                baudrate=self.baud_rate,
                timeout=1
            )
            if self.client.connect():
                self.connected = True
                self.get_logger().info("Successfully connected to the servo motor.")
            else:
                self.get_logger().error("Failed to connect to the servo motor.")
                self.connected = False
        except Exception as e:
            self.get_logger().error(f"Connection Error: {e}")
            self.connected = False

    def command_callback(self, msg):
        if msg > 2048:
            self.get_logger().info(f"Received command - STRONG GRIP: moving to position {msg}")
        elif msg == 2048:
            self.get_logger().info(f"Received command - GRIP: moving to position {msg}")
        else:
            self.get_logger().info(f"Received command - RELEASE: moving to position {msg}")
        self.send_command(self.POSITION_REGISTER, msg)

    def send_command(self, register, position):
        """Sends a write command to move the servo to the target position."""
        try:
            result = self.client.write_register(register, position)
            if result.isError():
                self.get_logger().error(f"Failed to write register {register}: {result}")
            else:
                self.get_logger().info(f"Servo moving to position {position}.")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")

    def feedback_callback(self):
        """Polls the servo for feedback data and publishes it to ROS topics."""
        if self.client:
            try:
                # Read registers
                position_result = self.client.read_holding_registers(257, count=1)
                speed_result = self.client.read_holding_registers(258, count=1)
                voltage_result = self.client.read_holding_registers(260, count=1)
                temperature_result = self.client.read_holding_registers(261, count=1)
                moving_result = self.client.read_holding_registers(262, count=1)
                current_result = self.client.read_holding_registers(263, count=1)

                # Publish data if read is successful
                if not position_result.isError():
                    self.position_pub.publish(Int32(data=position_result.registers[0]))
                if not speed_result.isError():
                    self.speed_pub.publish(Int32(data=speed_result.registers[0]))
                if not voltage_result.isError():
                    self.voltage_pub.publish(Float32(data=voltage_result.registers[0] / 10.0))  # Convert to volts
                if not temperature_result.isError():
                    self.temperature_pub.publish(Int32(data=temperature_result.registers[0]))
                if not moving_result.isError():
                    self.moving_pub.publish(Int32(data=moving_result.registers[0]))
                if not current_result.isError():
                    self.current_pub.publish(Int32(data=current_result.registers[0]))

            except Exception as e:
                self.get_logger().error(f"Error reading feedback: {e}")

def main():
    rclpy.init()
    node = ServoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
