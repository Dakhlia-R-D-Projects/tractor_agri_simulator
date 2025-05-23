from __future__ import annotations
import os
from os import environ, path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, SomeSubstitutionsType
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from pathlib import Path
from launch_ros.actions import Node

def has_nvidia_gpu():
    """
    Check if the system has an NVIDIA GPU on Linux.
    Returns True if an NVIDIA GPU is detected, False otherwise.
    Uses only standard libraries.
    """
    import subprocess
    
    try:
        # Try using lspci command (common on most Linux distributions)
        process = subprocess.run(['lspci'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if 'nvidia' in process.stdout.lower():
            return True
            
        # As a backup, check if nvidia-smi command exists and runs successfully
        nvidia_smi = subprocess.run(['nvidia-smi'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return nvidia_smi.returncode == 0
            
    except FileNotFoundError:
        # lspci or nvidia-smi command not found
        return False
    except Exception:
        # Any other error, assume no NVIDIA GPU
        return False
    
def construct_gz_args(
    world_file: SomeSubstitutionsType,
    paused: SomeSubstitutionsType,
    headless: SomeSubstitutionsType,
) -> SomeSubstitutionsType:
    paused_arg = PythonExpression(['"" if "', paused, '".lower() == "true" else "-r "'])
    headless_arg = PythonExpression(
        ['"-s " if "', headless, '".lower() == "true" else ""']
    )

    return PythonExpression(["'", headless_arg, paused_arg, world_file, "'"])


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    Pkg_directory = get_package_share_directory('virtual_maize_field')
    _ros_home_path = environ.get("ROS_HOME", path.join(path.expanduser("~"), ".ros"))
    cache_dir = path.join(_ros_home_path, "virtual_maize_field/")

    use_sim_time = LaunchConfiguration("use_sim_time")
    paused = LaunchConfiguration("paused")
    headless = LaunchConfiguration("headless")
    world_path = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")

    # Create launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time", default_value="True"
    )
    declare_paused_cmd = DeclareLaunchArgument(
        name="paused", default_value="False", description="Start simulation paused."
    )
    declare_headless_cmd = DeclareLaunchArgument(
        name="headless",
        default_value="False",
        description="Start headless simulation (without rendering).",
    )
    declare_world_path_cmd = DeclareLaunchArgument(
        name="world_path",
        default_value=cache_dir,
        description="Path to the directory containing the world SDF files.",
    )
    declare_world_name_cmd = DeclareLaunchArgument(
        name="world_name",
        default_value="generated.world",
        description="Name of the world file.",
    )

    gz_args = construct_gz_args(
        PathJoinSubstitution([world_path, world_name]),
        paused,
        headless,
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(Pkg_directory, 'models'),
            str(Path(Pkg_directory).parent.resolve())
        ])
    )

    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_MODEL_PATH',
        value=':'.join([
            os.path.join(Pkg_directory, 'models'),
            str(Path(Pkg_directory).parent.resolve())
        ])
    )
    
    if has_nvidia_gpu(): #check if the gpu is nvidia to export the following variables to make simulation work on gpu
        nvidia_prime = SetEnvironmentVariable(
        name='__NV_PRIME_RENDER_OFFLOAD',
        value='1')
    
        nvidia_ = SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME',
            value='nvidia')
        
        ld.add_action(nvidia_prime)
        ld.add_action(nvidia_)

    log_gz_args = LogInfo(msg=["Start Ignition Gazebo with gz_args: '", gz_args, "'"])

    # Create nodes
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ),
        ),
        launch_arguments={
            "gz_args": gz_args,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    sim_time_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        condition=IfCondition(use_sim_time),
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # Declare the launch options
    ld.add_action(gz_resource_path)
    ld.add_action(gz_model_path)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_paused_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_path_cmd)
    ld.add_action(declare_world_name_cmd)

    # Add nodes
    ld.add_action(log_gz_args)
    ld.add_action(gz)
    ld.add_action(sim_time_bridge)

    return ld
