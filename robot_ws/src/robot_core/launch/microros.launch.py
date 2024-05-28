import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    node_microros = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        # output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )
    
    ld.add_action(node_microros)
    
    return ld