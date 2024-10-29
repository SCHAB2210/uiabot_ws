from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [0.0, 0.0]  # Update based on encoder data if necessary
        joint_state.velocity = [left_wheel_speed, right_wheel_speed]  # Use values from teleop_serial node
        self.publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

