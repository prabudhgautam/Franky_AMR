from launch import LaunchDescription
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

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="True"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )


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

    simple_controller_py = Node()

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simple_controller_spawner,
    ])