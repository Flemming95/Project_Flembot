# 🤖 Project Flembot

A differential drive robot simulation with LiDAR-based perception and reactive navigation, built for **ROS 2 Jazzy** and **Gazebo Harmonic**.

---

## ✨ Features

- **Differential Drive Robot** — Two-wheeled robot with caster for stable movement
- **360° LiDAR Sensor** — Full environment scanning with GPU-accelerated simulation
- **Object Detection** — Point cloud clustering to identify obstacles
- **Reactive Navigation** — Condition-based movement (e.g., "move forward until close to wall, then turn")
- **Multiple Test Worlds** — 10 pre-built environments for testing
- **Docker Support** — Run without local ROS 2 installation

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Gazebo Simulation                       │
│   ┌──────────────┐              ┌──────────────┐           │
│   │    Robot     │◄─────────────│    LiDAR     │           │
│   │    Model     │              │   Sensor     │           │
│   └──────────────┘              └──────┬───────┘           │
└────────────────────────────────────────┼────────────────────┘
                                         │
                               ┌─────────▼─────────┐
                               │   ROS-Gazebo      │
                               │     Bridge        │
                               └─────────┬─────────┘
                                         │
         ┌───────────────────────────────┼───────────────────────────────┐
         ▼                               ▼                               ▼
┌─────────────────┐            ┌─────────────────┐            ┌─────────────────┐
│ Object Detector │            │   Navigation    │            │   Scan-to-Image │
│   (Clustering)  │            │   Controller    │            │   (Mapping)     │
└─────────────────┘            └─────────────────┘            └─────────────────┘
```

## 📋 Requirements

| Component | Version |
|-----------|---------|
| Ubuntu | 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic |

### Install Dependencies

```bash
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-rviz2
```

## 🚀 Quick Start

### Build

```bash
cd <your_workspace>
git clone https://github.com/Flemming95/Project_Flembot.git src/Project_Flembot
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Launch

```bash
# Basic robot
ros2 launch gazebo_differential_drive_robot robot.launch.py

# Robot + LiDAR object detection
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py

# Full navigation stack with RViz2
ros2 launch gazebo_differential_drive_robot robot_navigation.launch.py

# Reactive navigation (command-based)
ros2 launch gazebo_differential_drive_robot robot_reactive_navigation.launch.py
```

### Control

```bash
# Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Xbox controller
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

## 🌍 Worlds

Launch in different environments using shortcuts:

| # | Name | Description |
|---|------|-------------|
| 1 | `empty` | Empty world |
| 2 | `boxes` | Box obstacles |
| 3 | `cylinders` | Cylinder obstacles |
| 4 | `mixed` | Mixed obstacles |
| 5 | `corridor` | Corridor environment |
| 6 | `scattered` | Scattered obstacles |
| 7 | `zigzag` | Zigzag path |
| 8 | `corners` | Corner obstacles |
| 9 | `slalom` | Slalom course |
| 10 | `challenge` | Challenge course |

```bash
# By number
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=5

# By name
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=corridor

# With custom pose (x, y, z position and Y=yaw rotation)
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=boxes x:=1.0 y:=2.0 Y:=1.57
```

## 🧭 Reactive Navigation

Send commands to navigate based on sensor feedback:

```bash
# Move forward until wall detected, then turn
ros2 topic pub /navigation/command std_msgs/msg/String \
    "data: 'forward_until_close:front:0.5:turn_left'" --once

# Chain commands
ros2 topic pub /navigation/command std_msgs/msg/String \
    "data: 'forward:1.0;turn_left:90;forward:2.0'" --once
```

### Command Reference

| Command | Example |
|---------|---------|
| Move forward | `forward:2.0` |
| Turn left | `turn_left:90` |
| Until obstacle | `forward_until_close:front:0.5` |
| Wall follow | `follow_wall:left:0.5:10.0` |

## 📊 Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Velocity commands |
| `/lidar/scan` | `LaserScan` | Raw LiDAR data |
| `/detected_objects` | `MarkerArray` | Detected obstacles |
| `/navigation/command` | `String` | Navigation commands |
| `/navigation/status` | `String` | Navigation state (JSON) |

## 🐳 Docker

No local ROS 2 installation? No problem.

```bash
cd src/docker

# Windows (sets up X11 display forwarding)
.\run.ps1

# Linux/macOS
docker compose up -d

# Enter container and launch
docker exec -it gz_diff_drive_robot bash
ros2 launch gazebo_differential_drive_robot robot.launch.py
```

## 📁 Project Structure

```
Project_Flembot/
├── src/
│   ├── model/           # Robot URDF/Xacro
│   ├── launch/          # Launch files
│   ├── scripts/         # Python nodes
│   ├── config/          # Bridge configs
│   ├── worlds/          # Simulation worlds
│   └── docker/          # Docker setup
├── README.md		 # README file
```

## 📚 Documentation

- [LiDAR Detection](tutorials/LIDAR_DETECTION.md) — Technical details on object detection
- [LiDAR Tutorial](tutorials/LIDAR_TUTORIAL.md) — Getting started with LiDAR
- [Reactive Navigation](tutorials/REACTIVE_NAVIGATION.md) — Command-based navigation


## 🛠️ Tech Stack

- **ROS 2 Jazzy** — Robot middleware
- **Gazebo Harmonic** — Physics simulation
- **Python** — Node implementations
- **Xacro** — Robot model templating

## 📜 License

Apache License 2.0

---

*Built with ☕ and ROS 2*
