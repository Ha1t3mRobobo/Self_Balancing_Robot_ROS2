# ROS 2 Self-Balancing Robot

## Overview
This repository contains a simple self-balancing robot implemented using ROS 2 and simulated in Gazebo.
The goal of this project is to learn and experiment with sensor feedback and control algorithms for balancing robots.

This project supports two control methods:
- PID control
- ADRC control

## Prerequisites
- ROS 2 JazzyJalisco
- Gazebo Harmonic

## Installation
```bash
git clone https://github.com/Ha1t3mRobobo/Self_Balancing_Robot_ROS2
```
## Run
```bash
cd twbot_ws/
source install/setup.bash
ros2 launch twbot_demo launch_sim.launch.py
```
