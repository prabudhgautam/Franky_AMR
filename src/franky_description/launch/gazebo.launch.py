import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    franky_description_dir = get_package_share_directory("franky_description")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "jazzy" else "False"

    model_arg = DeclareLaunchArgument(name="model",default_value=os.path.join(
                                        franky_description_dir, "URDF", "franky.urdf.xacro"
                                        ),
                                      description="Absolute path to robot URDF file ") 
    
    robot_description = ParameterValue(Command([
           "xacro ",
           LaunchConfiguration("model"),
           " is_ignition:=",
           is_ignition
           ]),
           value_type=str,
    )
    # instruction for starting state publishers
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(franky_description_dir).parent.resolve())
        ]
    )

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r"])
        ]
    )

    gz_spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-topic", "robot_description", "-name", "franky"],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity
        ]
    )