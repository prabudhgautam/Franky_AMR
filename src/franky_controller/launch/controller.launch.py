from launch import LaunchDescription 
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction, DeclareLaunchArgument 
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    controllers_file = os.path.join(
        get_package_share_directory("franky_controller"),
        "config",
        "franky_controllers.yaml",
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation") 


    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--param-file",
                    controllers_file,
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        ],
    )

    simple_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--param-file",
                    controllers_file,
                    "--controller-manager-timeout",
                    "60",
                ],
                output="screen",
            )
        ],
    )

    simple_controller_py = TimerAction(
        period=5.0,
        actions=[
            Node(
                package = "franky_controller",
                executable = "simple_controller.py",
                parameters = [
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation}
                ],
                output = "screen"
            )
        ],
    )


    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        simple_controller_spawner,
        simple_controller_py
    ])