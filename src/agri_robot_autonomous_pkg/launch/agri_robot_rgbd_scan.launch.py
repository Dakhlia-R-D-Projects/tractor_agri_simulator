# Example:
#
#   Bringup turtlebot3:
#     $ export TURTLEBOT3_MODEL=waffle
#     $ export LDS_MODEL=LDS-01
#     $ ros2 launch turtlebot3_bringup robot.launch.py
#
#   SLAM:
#     $ ros2 launch rtabmap_demos turtlebot3_rgbd_scan.launch.py
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters={
          'rgbd_cameras':3,
          'frame_id':'base_link',
          'odom_frame_id':'odom',
          'use_sim_time': use_sim_time,
          'odom_tf_linear_variance':0.001,
          'odom_tf_angular_variance':0.001,
          'subscribe_odom_info':False,
          'subscribe_rgbd':True,
          'subscribe_rgb':True,
          'subscribe_depth':True,
          'subscribe_scan':False,
          'approx_sync':True,
          'sync_queue_size': 10,
          # RTAB-Map's internal parameters should be strings
          'RGBD/NeighborLinkRefining': 'true',    # Do odometry correction with consecutive laser scans
          'RGBD/ProximityBySpace':     'true',    # Local loop closure detection (using estimated position) with locations in WM
          'RGBD/ProximityByTime':      'false',   # Local loop closure detection with locations in STM
          'RGBD/ProximityPathMaxNeighbors': '10', # Do also proximity detection by space by merging close scans together.
          'Reg/Strategy':              '2',       # 0=Visual, 1=ICP, 2=Visual+ICP
          'Vis/MinInliers':            '12',      # 3D visual words minimum inliers to accept loop closure
          'RGBD/OptimizeFromGraphEnd': 'true',   # Optimize graph from initial node so /map -> /odom transform will be generated
          'RGBD/OptimizeMaxError':     '4',       # Reject any loop closure causing large errors (>3x link's covariance) in the map
          'Reg/Force3DoF':             'true',    # 2D SLAM
          'Grid/FromDepth':            'true',   # Create 2D occupancy grid from laser scan
          'Mem/STMSize':               '30',      # increased to 30 to avoid adding too many loop closures on just seen locations
          'RGBD/LocalRadius':          '2',       # limit length of proximity detections
          'Icp/CorrespondenceRatio':   '0.2',     # minimum scan overlap to accept loop closure
          'Icp/PM':                    'false',
          'Icp/PointToPlane':          'true',    # Use point-to-plane ICP
          'Icp/MaxCorrespondenceDistance': '0.15',
          'Icp/VoxelSize':             '0.2',
          "Rtabmap/DetectionRate": "5", # 5Hz
          "Grid/NoiseFilteringMinNeighbors": "5", # 2D occupancy grid noise filtering
          "Grid/RangeMax": "20.0", # 2D occupancy grid max range
          "GridGlobal/FootprintRadius": "0.55", # 2D occupancy grid robot footprint radius
          "GTSAM/Optimizer":'1', # 0=Levenberg 1=GaussNewton 2=Dogleg
          "GTSAM/Incremental":"true",
    }
    remappings_slam=[
        ('rgbd_image0', 'front/rgbd_image'),
        ('rgbd_image1', 'right/rgbd_image'),
        ('rgbd_image2', 'left/rgbd_image'),
        ('odom', 'odom'),
    ]
    remappings1=[
         ('/rgb/image',       'front/rgb/image'),
         ('/depth/image',     'front/depth/image'),
         ('/rgb/camera_info', 'front/rgb/camera_info'),
         ('rgbd_image', 'front/rgbd_image')]
    remappings2=[
         ('/rgb/image',       'right/rgb/image'),
         ('/depth/image',     'right/depth/image'),
         ('/rgb/camera_info', 'right/rgb/camera_info'),
         ('rgbd_image', 'right/rgbd_image')]
    remappings3=[
         ('/rgb/image',       'left/rgb/image'),
         ('/depth/image',     'left/depth/image'),
         ('/rgb/camera_info', 'left/rgb/camera_info'),
         ('rgbd_image', 'left/rgbd_image')]
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync',name="front_rgbd", output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings1),
        
        Node(
            package='rtabmap_sync', executable='rgbd_sync',name="right_rgbd", output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings2),
        
        Node(
            package='rtabmap_sync', executable='rgbd_sync',name="left_rgbd",output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings3),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings_slam,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings1),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters]),
        
        # Obstacle detection with the camera for nav2 local costmap.
        # First, we need to convert depth image to a point cloud.
        # Second, we segment the floor from the obstacles.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz',name="left_cloud", output='screen',
            parameters=[{'decimation': 1,
                         'max_depth': 20.0,
                         'voxel_size': 0.001}],
            remappings=[('depth/image', 'front/depth/image'),
                        ('depth/camera_info', 'front/rgb/camera_info'),
                        ('cloud', 'front/camera/points')]),
        Node(
            package='rtabmap_util', executable='point_cloud_xyz',name="right_cloud", output='screen',
            parameters=[{'decimation': 1,
                         'max_depth': 20.0,
                         'voxel_size': 0.001}],
            remappings=[('depth/image', 'right/depth/image'),
                        ('depth/camera_info', 'right/rgb/camera_info'),
                        ('cloud', 'right/camera/points')]),
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', name="left_cloud", output='screen',
            parameters=[{'decimation': 1,
                         'max_depth': 20.0,
                         'voxel_size': 0.001}],
            remappings=[('depth/image', 'left/depth/image'),
                        ('depth/camera_info', 'left/rgb/camera_info'),
                        ('cloud', 'left/camera/points')]),
    ])
