import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MagnifyCmdVel(Node):
    def __init__(self):
        super().__init__('magnify_cmd_vel')
        self.declare_parameter('magnification_factor', 5.0)
        self.magnification_factor = self.get_parameter('magnification_factor').value

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_navv',
            self.cmd_vel_callback,
            1
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

    def cmd_vel_callback(self, msg:Twist):
        magnified_msg = Twist()
        magnified_msg.linear.x = msg.linear.x * self.magnification_factor
        magnified_msg.linear.y = 0.0
        magnified_msg.linear.z = 0.0
        magnified_msg.angular.x = 0.0
        magnified_msg.angular.y = 0.0
        magnified_msg.angular.z = msg.angular.z * self.magnification_factor*4.3
        self.publisher.publish(magnified_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MagnifyCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
