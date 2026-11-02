# Lab 1 – Introduction to Robotics and Intelligent Systems

**Program:** BSc in Software Engineering  
**Course:** Introduction to Robotics and Intelligent Systems  
**Lecturer:** Mohammed Sraj (MSc AI)  
**Academic Year:** 2026

## Project Overview

This project implements the Lab 1 exercises using ROS 2 Jazzy and Gazebo Harmonic.

It contains four packages:

- `diffdrive_bot` – Robot model, sensors, and Gazebo simulation.
- `lab_interfaces` – Obstacle and distance-driving interfaces.
- `my_robot_interfaces` – Custom message, service, and action interfaces.
- `my_robot_nodes` – Obstacle avoidance, rotation, and square-driving nodes.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- RViz2
- Colcon

## Build Instructions

Clone the repository into your workspace's `src` folder:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <YOUR_GITHUB_REPOSITORY_URL>
```

Build the packages:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For every new terminal, run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Running the Exercises

### Step 1: Depth Camera and IMU

Start the simulation:

```bash
ros2 launch diffdrive_bot gazebo.launch.py
```

Check the sensor topics:

```bash
ros2 topic list
ros2 topic echo /imu --once
```

In RViz, add Image (`/depth_camera/image`) and PointCloud2 (`/depth_camera/points`) displays.

### Steps 2–5: Obstacle Avoidance, Parameters, Message, and Services

Start the simulation:

```bash
ros2 launch diffdrive_bot gazebo.launch.py
```

In separate terminals, start the obstacle detector and avoidance node:

```bash
ros2 run diffdrive_bot obstacle_detector.py
ros2 run my_robot_nodes turn_away
```

Enable obstacle avoidance:

```bash
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
```

Change parameters:

```bash
ros2 param set /turn_away forward_speed 0.1
ros2 param set /turn_away turn_speed 0.8
```

View robot status:

```bash
ros2 topic echo /robot_status
```

Set turning speed:

```bash
ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 0.7}"
```

Disable obstacle avoidance:

```bash
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"
```

### Step 6: Rotation Action Server

Start the simulation, then run:

```bash
ros2 run my_robot_nodes rotate_server
```

Send a 90-degree rotation goal:

```bash
ros2 action send_goal /rotate my_robot_interfaces/action/Rotate "{angle: 1.5708}" --feedback
```

### Step 7: Square Driving

Run the complete square-driving launch:

```bash
ros2 launch my_robot_nodes square.launch.py
```

To change the square's side length:

```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
```

Open the node graph:

```bash
rqt_graph
```

## Screenshots

The `screenshots` folder contains:

- `gazebo.png` – Gazebo simulation.
- `rviz.png` – RViz showing the depth image and point cloud.
- `rqt_graph.png` – ROS 2 node graph during the square launch.


## Repository Structure

```text
lab1_exercise/
├── diffdrive_bot/
├── lab_interfaces/
├── my_robot_interfaces/
├── my_robot_nodes/
├── screenshots/
│   ├── gazebo.png
│   ├── rviz.png
│   └── rqt_graph.png
└── README.md
```