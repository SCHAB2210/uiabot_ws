import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct  # Import struct for packing data

class CmdVelListener(Node):

    def __init__(self):
        super().__init__('cmd_vel_listener')

        # Create a subscriber to listen to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Configure the serial connection to ESP32
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # Replace with the actual USB port
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            exit(1)

    def listener_callback(self, msg):
        """Callback to handle incoming Twist messages."""
        linear = msg.linear.x
        angular = msg.angular.z

        # Pack data into a byte buffer ('=BBff' = '=(Byte)(Byte)(float)(float)')
        try:
            buff = struct.pack('=BBff', 36, 36, linear, angular)

            # Send the packed data over serial
            self.ser.write(buff)
            self.get_logger().info(f"Sent packed data: {buff}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send packed data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
