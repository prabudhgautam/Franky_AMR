from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    franky_description_dir = get_package_share_directory("franky_description")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(franky_description_dir, "URDF", "franky.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model"), " is_ignition:=True"]),
        value_type=str,
    )

    # Add robot_state_publisher FIRST
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    # Start controller_manager with robot_description
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(franky_description_dir, "config", "controllers.yaml")
        ],
        output="screen"
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
                ],
                output="screen"
            )
        ]
    )

    simple_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        simple_controller
    ])