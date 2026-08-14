from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
def generate_launch_description:

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher"
        parameter=[{"robot_description": robot_description}]
    )

    return LaunchDescription(


    )