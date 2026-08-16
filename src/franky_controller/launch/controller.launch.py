from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    # Wait before spawning, Gazebo's controller_manager should be running
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
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
        period=4.0,
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
        joint_state_broadcaster_spawner,
        simple_controller
    ])