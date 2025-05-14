# Tractor Agriculture Simulator
<p align="center">
  <img src="https://github.com/user-attachments/assets/4ad0beef-f8e4-481f-86b7-011c8708b095" />
  <img src="https://github.com/user-attachments/assets/eb0caf3b-e7ff-4000-8de7-06225c68402a" />
   
</p>
## Overview

Tractor Agriculture Simulator is a ROS 2-based simulation environment for agricultural robotics research and development forsensor fusion, SLAM, navigation, and virtual field generation to enable the testing and development of autonomous agricultural robots in realistic scenarios.
## Branches
|**Branch**         |**Description**                                      |
|-------------------|-----------------------------------------------------|
| main              | Used to test module like control and perception     |
| localization_test_bench | used to test different localization and odometry techniques with groundtruth odometery |


## Main Features
- Simulate tractors in virtual Agriculture fields with different crops type(Palm, Orange tree, Lemon tree).
- Sensor fusion using IMU and other sensors.
- 2D&3D SLAM with RTAB-Map.
- Teleoperation using Joystick.
- Customizable field/world generation.

## Project Structure
- `src/tractor_description/`: Tractor robot model, URDF, meshes, and launch files.
- `src/agri_robot_autonomous_pkg/`: contian the launch files for the odometry and visual SLAM.
- `src/imu_tools/`: IMU filters and sensor fusion nodes.
- `src/rtabmap_ros/`: RTAB-Map SLAM integration and demos.
- `src/Simulation_Agriculture_Field_Generator/`: Virtual field/world generator.

## Installation
1. **Install ROS 2 (Jazzy recommended)**
2. **Clone this repository**
3. **Install dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. **Build the workspace for multi camera support**:
   ```bash
   colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release
   ```
5. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Usage
### Generate a Virtual Field
```bash
ros2 launch virtual_maize_field simulation.launch.py
```
- To select a specific world:
  ```bash
    ros2 run virtual_maize_field generate_world --crop_type natroon_palm --plant_spacing_min 7.0 --plant_spacing_max 7.1 --row_width 10.0 --row_segments straight --ground_resolution 0.5 --hole_size 0 --hole_prob 0 --ground_ditch_depth 0.0001 --ground_elevation_max 0.0001 --rows_count 3 --ground_headland 10
  ```
- copy 
    ```bash
    cp ~/.ros/virtual_maize_field/generated.world ~/tractor_agri_simulator/src/tractor_description/world/

    ``` 
### Launching the Tractor Simulator
```bash
source install/setup.bash
ros2 launch tractor_description gz_simulator_launch.py
```

### Running visual mapping and visual odometery
```bash
source install/setup.bash
ros2 launch agri_robot_autonomous_pkg agri_robot_rgbd_scan_demo.launch.py localization:=false #ground_truth for removing the spawn shift from the ground truth odom
```
### Teleoperation
```bash
source install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py
```

## Customization
- **Field Generation**: Use the tools in `Simulation_Agriculture_Field_Generator` to create custom fields.
- **Robot Models**: Edit URDF and mesh files in `tractor_description`.
- **SLAM**: Modify parameters in the respective config files.
## Sensors and Topics

### Sensor List
|**Sensor Name**   |**model**  | **Type**       | **Description**                   |**Usage**                  |
|------------------|-----------|----------------|-----------------------------------|---------------------------|
| IMU              | TD        | IMU            | Inertial Measurement Unit         |fused with visual odometry |
| NavSat           | TD        | GNSS/GPS       | Global Navigation Satellite System|                 |
| Front RGB-D      | ZED2i     | Camera (RGB-D) | Front stereo camera               |Used to obtain visual odometry and mapping                 |
| Left RGB-D       | ZED2i     | Camera (RGB-D) | Left stereo camera                |Used to obtain visual odometry and mapping and used in object detection          |
| right RGB-D      | ZED2i     | Camera (RGB-D) | right stereo camera               |Used to obtain visual odometry and mapping and used in object detection          |
| LIDAR            | rplidar s2| 2D LIDAR       | Laser scanner                     |Fused with visual odometry to give better reading  |

### Main ROS Topics
|**Topic Name**             | **Message Type**              | **Description**                     |
|---------------------------|-------------------------------|-------------------------------------|
| /imu                      | sensor_msgs/msg/Imu           | IMU data                            |
| /navsat                   | sensor_msgs/msg/NavSatFix     | GNSS/GPS data                       |
| /scan                     | sensor_msgs/msg/LaserScan     | 2D LIDAR scan                       |
| /front/rgb/image          | sensor_msgs/msg/Image         | Front RGB camera image              |
| /front/depth/image        | sensor_msgs/msg/Image         | Front depth camera image            |
| /front/rgb/camera_info    | sensor_msgs/msg/CameraInfo    | Front RGB camera info               |
| /left/rgb/image           | sensor_msgs/msg/Image         | Left RGB camera image               |
| /left/depth/image         | sensor_msgs/msg/Image         | Left depth camera image             |
| /left/rgb/camera_info     | sensor_msgs/msg/CameraInfo    | Left RGB camera info                |
| /right/rgb/image          | sensor_msgs/msg/Image         | Right RGB camera image              |
| /right/depth/image        | sensor_msgs/msg/Image         | Right depth camera image            |
| /right/rgb/camera_info    | sensor_msgs/msg/CameraInfo    | Right RGB camera info               |
| /cmd_vel                  | geometry_msgs/msg/Twist       | Velocity command                    |
| /tf                       | tf2_msgs/msg/TFMessage        | Transforms                          |
| /odom                     | nav_msgs/msg/Odometry         | Ground truth odometry (sim) with spawn shift     |
| /clock                    | rosgraph_msgs/msg/Clock       | Simulation clock                   |

> Note: Topic names may be remapped in launch/config files. See `tractor_description/config/gz_bridge.yaml` and launch files for details.

## Contributing
Contributions are welcome! Please open issues and pull requests for bug fixes, new features, or documentation improvements.

## Authors
- Ebrahim Abdelghafar <ibrahim.abdelghafar@dakahlia.net>

## License
This project is licensed under the [TODO: License declaration].
