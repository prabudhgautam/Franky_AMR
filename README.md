# Franky AMR

This repository contains a ROS 2 Jazzy workspace for the Franky AMR robot model and controller examples.

## Workspace structure

- `src/franky_controller` — ROS controller and odometry publisher node
- `src/franky_description` — robot URDF/Xacro and Gazebo/RViz launch files
- `src/franky_utils` — trajectory visualization helper node

## Build

```bash
cd /home/prabudh/frankybot_ws
colcon build
source install/setup.bash
```

## Run

Launch the robot description and display:

```bash
ros2 launch franky_description display.launch.py
```

Run the controller node:

```bash
ros2 run franky_controller simple_controller
```

Run the trajectory visualization node:

```bash
ros2 run franky_utils Trajectory_Viz_Node.py
```

## Notes

- RViz fixed frame should be `odom` when displaying the path topic.
- The trajectory path topic should be `/franky_controller/trajectory`.
- The model has TF frames such as `odom -> base_footprint` and `base_link` relationships.
