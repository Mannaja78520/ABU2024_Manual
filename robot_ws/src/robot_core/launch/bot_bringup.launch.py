import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    microros_launch_path = PathJoinSubstitution(
        [FindPackageShare("robot_core"), "launch", "microros.launch.py"]
    )
    
    robot_run_node = Node(
        package="robot_core",
        # output="screen",
        executable="runRobot"
    )
    
    launch_microros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(microros_launch_path),
    )
    ld.add_action(launch_microros)
    ld.add_action(robot_run_node)
    
    # os.system("gnome-terminal -e 'bash -c \"ros2 launch robot_core microros.launch.py\"'")
    # os.system("gnome-terminal -e 'bash -c \"ros2 launch robot_core microros.launch.py\"'")
    
    return ld