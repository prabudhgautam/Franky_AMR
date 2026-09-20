from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():

    static_transform_publisher = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
                )
        ]
    )

    robot_localization = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[os.path.join(get_package_share_directory("franky_localization"), "config", "ekf.yaml"), 
                            {'use_sim_time': True},
                ],
            )
        ]
    )

    imu_republisher_py = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="franky_localization",
                executable="imu_republisher.py",
            )
        ]
    )

    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        imu_republisher_py,  
    ])