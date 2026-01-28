# ROS 2 Self-Balancing Robot
> **Note:** In this README, all examples and commands refer to package name `twbot_demo`.
> >⚠️ If you rename the package or you have a diffrent package name, you may need to update the package name in `launch_sim.launch.py`, `online_async_launch.py`, `CMakeLists.txt`, `package.xml`, and any other scripts that reference the package.

## Overview
This repository contains a simple self-balancing robot implemented using ROS 2 and simulated in Gazebo.  
The goal of this project is to learn and experiment with sensor feedback and control algorithms for balancing robots.

This project supports two control methods:
- PID control
- ADRC control

Additionally, the robot can be:
- Teleoperated using the keyboard
- Used for online mapping experiments
- Equipped with an onboard camera that can be visualized in RViz to see the world from the robot's point of view (POV)

---

## Prerequisites
- ROS 2 **Jazzy Jalisco**  
- Gazebo **Harmonic**  
- Python3 and `ros2-teleop-twist-keyboard` package (for teleoperation)  
- RViz 2 (for camera visualization)  

## Install teleop package if not already done:
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```
## Installation
Clone the repository:
```
git clone https://github.com/Ha1t3mRobobo/Self_Balancing_Robot_ROS2
cd Self_Balancing_Robot_ROS2
```
## Build the workspace:
```
colcon build
source install/setup.bash
```
# Usage
# 1. Launch simulation
Start the Gazebo simulation with the robot:
```
ros2 launch twbot_demo launch_sim.launch.py
```
## 2. Teleoperation
Control the robot manually with the keyboard:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
You can now drive the robot forward, backward, and turn while it balances.

## 3. Map creation (online mapping)
Launch the robot with online asynchronous mapping mode:
```
ros2 launch twbot_demo online_async_launch.py use_sim_time:=true
```
This will allow the robot to create a map in real-time using its sensors.

## 4. Camera visualization in RViz
The robot has an onboard camera. To visualize it in RViz:
```
ros2 run rviz2 rviz2
```
Add an Image display in RViz.

Set the topic to the robot camera (e.g., /camera/image_raw) to see the world from the robot’s POV.

# Control Methods
PID control: Classic Proportional-Integral-Derivative control for balancing.

ADRC control: Active Disturbance Rejection Control for more robust performance.

## Switching between controllers:
If you want to switch the control to PID, open launch_sim.launch.py and go to the end of the code:
```
return LaunchDescription([
    rsp,
    gazebo_launch,
    spawn_entity,
    gz_bridge,
    torque_driver_node,
    ADRC_node,
])
```
Replace **ADRC_node** with **balance_node** like so :
```
return LaunchDescription([
    rsp,
    gazebo_launch,
    spawn_entity,
    gz_bridge,
    torque_driver_node,
    balance_node,
])
```
## Configuration
```
config/gz_bridge.yaml → ROS-Gazebo bridge configuration

config/mapper_params_online_async.yaml → Parameters for online mapping

config/my_map_save.pgm → Example saved map
```

# Tips
Always source the workspace before running commands:
```
source install/setup.bash
```

Adjust PID/ADRC parameters in scripts/ : **ADRC_control.py** for ADRC and **balancing.py** for PID.

Use RViz to visualize sensors and camera data to debug the robot behavior.

# Project Structure
```
├── config/                  # Config files for Gazebo and controllers
│   ├── gz_bridge.yaml
│   ├── mapper_params_online_async.yaml
│   ├── my_map_save.pgm
├── description/             # URDF or robot description files
├── launch/                  # ROS 2 launch files
│   ├── launch_sim.launch.py
│   ├── online_async_launch.py
├── scripts/                 # Utility scripts for controllers
├── worlds/                  # Gazebo world files
├── CMakeLists.txt
├── package.xml
└── README.md
```
