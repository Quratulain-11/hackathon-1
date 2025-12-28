import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class BasicController(Node):
    """
    Basic movement and control node for humanoid robot
    """

    def __init__(self):
        super().__init__('basic_controller')

        # Create publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Basic Controller node initialized')

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')

    def control_loop(self):
        """Main control loop"""
        # Example: Publish a simple movement command
        cmd = Twist()
        cmd.linear.x = 0.1  # Move forward slowly
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    basic_controller = BasicController()

    try:
        rclpy.spin(basic_controller)
    except KeyboardInterrupt:
        pass
    finally:
        basic_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()