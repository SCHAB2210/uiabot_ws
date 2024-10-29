import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import os  # Import os to execute shell commands

class CarKeyboardControl(Node):
    def __init__(self):
        super().__init__('car_keyboard_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

        # Car-like control parameters
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.max_linear_speed = 10.0  # Max forward/backward speed
        self.max_angular_speed = 10.0  # Max turning speed
        self.linear_acceleration = 0.2  # Acceleration step size
        self.angular_acceleration = 0.1  # Turning step size

        # Print the teleop keyboard help message
        self.print_help_message()

    def print_help_message(self):
        """Print the help message similar to teleop_twist_keyboard."""
        print("""
This node takes keypresses from the keyboard and publishes them
as Twist messages to control the robot. It works best with a US keyboard layout.
---------------------------
Moving around:
   w : forward
   s : backward
   a : turn left
   d : turn right
   space : execute special command

anything else : stop

CTRL-C to quit

currently:  speed {0:.1f}  turn {1:.1f}
""".format(self.linear_speed, self.angular_speed))

    def get_key(self):
        """Get a single character from standard input (non-blocking)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def update_twist(self):
        """Publish the current twist message based on linear and angular speeds."""
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed
        self.publisher.publish(self.twist)
        print("currently:  speed {0:.1f}  turn {1:.1f}".format(self.linear_speed, self.angular_speed))

    def run(self):
        """Main loop to continuously capture keypresses."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':
                    # Move forward (positive linear velocity)
                    self.linear_speed = min(self.linear_speed + self.linear_acceleration, self.max_linear_speed)
                elif key == 's':
                    # Move backward (negative linear velocity)
                    self.linear_speed = max(self.linear_speed - self.linear_acceleration, -self.max_linear_speed)
                elif key == 'a':
                    # Turn left
                    self.angular_speed = min(self.angular_speed + self.angular_acceleration, self.max_angular_speed)
                elif key == 'd':
                    # Turn right
                    self.angular_speed = max(self.angular_speed - self.angular_acceleration, -self.max_angular_speed)
                elif key == ' ':
                    # Stop the robot
                    os.system("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
                elif key == '\x03':  # CTRL-C
                    break
                else:
                    # Stop the robot
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0

                self.update_twist()
        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    node = CarKeyboardControl()
    node.run()  # Start the main loop
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

