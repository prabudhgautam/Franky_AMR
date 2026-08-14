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
echo "# Franky_AMR" >> README.md
git init
git add .
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/prabudhgautam/Franky_AMR.git
git push -u origin main
git remote add origin https://github.com/prabudhgautam/Franky_AMR.git
git branch -M main
git push -u origin main