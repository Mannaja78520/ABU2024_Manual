import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    # function_config_path = PathJoinSubstitution(
    #     [FindPackageShare("robot_core"), "config", "params.yaml"]
    # )

    node_microros = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )
    node_joy = Node(package="joy", executable="joy_node")
    node_joy_drive = Node(
        package="robot_core",
        executable="joy_drive",
        parameters=[function_config_path],
    )
    ld.add_action(node_joy)
    ld.add_action(node_joy_drive)
    # ld.add_action(node_microros)

    os.system("gnome-terminal -e 'bash -c \"ros2 launch abu_core microros.launch.py\"'")

    return ld