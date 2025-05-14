
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_fusion')  # Replace with your package
    ukf_config = os.path.join(pkg_share, 'config', 'ukf_config.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_filter_node',
            output='screen',
            parameters=[ukf_config],
        ),
    ])