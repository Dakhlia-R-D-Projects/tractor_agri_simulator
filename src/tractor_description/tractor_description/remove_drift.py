import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance

class RelativeOdomNode(Node):
    def __init__(self):
        super().__init__('relative_odom_node')
        self.sub = self.create_subscription(Odometry, '/gt_odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, '/gt_pose_no_drift', 1)
        self.spawn_set = True
        self.spawn_x = 3.7
        self.spawn_y = -1.595

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.spawn_set:
            self.spawn_x = x
            self.spawn_y = y
            self.spawn_set = True
            self.get_logger().info(f"Spawn location set at: x={self.spawn_x}, y={self.spawn_y}")
            return

        rel_pose = Odometry()
        rel_pose.header.stamp = self.get_clock().now().to_msg()
        rel_pose.header.frame_id = "map"  # or "odom", depending on your TF setup

        rel_pose.pose.pose.position.x = (y - self.spawn_y)
        rel_pose.pose.pose.position.y =  -(x - self.spawn_x)
        rel_pose.pose.pose.position.z = msg.pose.pose.position.z
        rel_pose.pose.pose.orientation = msg.pose.pose.orientation
        rel_pose.pose.covariance = msg.pose.covariance

        rel_pose.twist.twist = msg.twist.twist
        self.pub.publish(rel_pose)

def main(args=None):
    rclpy.init(args=args)
    node = RelativeOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
