import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32, Int32, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray, ByteMultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DashboardBridge(Node):
    def __init__(self):
        super().__init__('dashboard_bridge')
        self.bridge = CvBridge()
        
        # Publishers
        self.linear_vel_pub = self.create_publisher(Float32, 'robot/linear_velocity', 1)
        self.angular_vel_pub = self.create_publisher(Float32, 'robot/angular_velocity', 1)
        self.imu_pitch_pub = self.create_publisher(Float32, 'robot/imu/pitch', 1)
        self.imu_roll_pub = self.create_publisher(Float32, 'robot/imu/roll', 1)
        self.imu_yaw_pub = self.create_publisher(Float32, 'robot/imu/yaw', 1)
        self.navsat_location_latitude = self.create_publisher(Float32, 'robot/location/latitude', 1)
        self.navsat_location_longitude = self.create_publisher(Float32, 'robot/location/longitude', 1)
        self.camera_pub= self.create_publisher(Image, 'robot/camera/image_raw', 1)
        # subscribers
        self.create_subscription(Odometry, '/odom', self.publishSpeedCallback, 1)
        self.create_subscription(Imu, '/imu', self.publishImuCallback, 1)
        self.create_subscription(NavSatFix, '/navsat', self.publishNavSatCallback, 1)
        self.create_subscription(Image, '/front/rgb/image', self.publishCameraCallback, 1)
        
    def publishSpeedCallback(self, msg: Odometry):
        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z
        
        linear_vel_msg = Float32()
        linear_vel_msg.data = linear_velocity
        angular_vel_msg = Float32()
        angular_vel_msg.data = angular_velocity
        self.angular_vel_pub.publish(angular_vel_msg)
        self.linear_vel_pub.publish(linear_vel_msg)
    def publishImuCallback(self, msg:Imu):
        pitch = msg.orientation.x
        roll = msg.orientation.y
        yaw = msg.orientation.z
        # Create Float32 messages for pitch, roll, and yaw
        imu_pitch_msg = Float32()
        imu_pitch_msg.data = pitch
        imu_roll_msg = Float32()
        imu_roll_msg.data = roll
        imu_yaw_msg = Float32()
        imu_yaw_msg.data = yaw
        # Publish IMU data
        self.imu_roll_pub.publish(imu_roll_msg)
        self.imu_yaw_pub.publish(imu_yaw_msg)
        self.imu_pitch_pub.publish(imu_pitch_msg)
    def publishNavSatCallback(self, msg:NavSatFix):
        latitude = msg.latitude
        longitude = msg.longitude
        # Create Float32 messages for latitude and longitude
        navsat_latitude_msg = Float32()
        navsat_latitude_msg.data = latitude
        self.navsat_location_latitude.publish(navsat_latitude_msg)
        # Publish latitude
        navsat_longitude_msg = Float32()
        navsat_longitude_msg.data = longitude
        self.navsat_location_longitude.publish(navsat_longitude_msg)
    def publishCameraCallback(self, msg:Image):
        # Convert the ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Publish the camera image
        self.camera_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
def main(args=None):
    rclpy.init(args=args)
    node = DashboardBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()