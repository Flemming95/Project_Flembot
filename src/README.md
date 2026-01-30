# Differential Drive Robot Simulation

## About

This package provides a simple differential drive robot model designed for use in Gazebo Harmonic simulation with ROS 2 Jazzy Jalisco. The robot is equipped with a lidar sensor for object detection capabilities.

## Features

- Differential drive robot with two wheels and a caster
- 360-degree lidar sensor for environment scanning
- Object detection using lidar data clustering
- Gazebo Harmonic simulation integration
- ROS 2 Jazzy Jalisco compatibility
- Teleoperation support (keyboard and joystick)

## Requirements

To run this package, you'll need the following:

- [Linux Ubuntu 24.04](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)
- [ROS2 Jazzy Jalisco](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/) 


#### Install Required ROS 2 Packages

Make sure to install the following ROS 2 Jazzy Jalisco packages:

```bash
sudo apt install -y                         \
    ros-jazzy-ros-gz                        \
    ros-jazzy-ros-gz-bridge                 \
    ros-jazzy-joint-state-publisher         \
    ros-jazzy-xacro                         \
    ros-jazzy-teleop-twist-keyboard         \
    ros-jazzy-teleop-twist-joy              \
    ros-jazzy-rviz2
```

## Usage

### Clone the Repository

Clone this repository into your ``workspace/src`` folder. If you don't have a workspace set up, you can learn more about creating one in the [ROS 2 workspace tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).


```bash
cd <path_to_your_workspace>/src
git clone git@github.com:lucasmazz/gazebo_differential_drive_robot.git
cd ..
```

### Build the Package

Source the ROS 2 environment and build the package:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

### Launch the Robot

After building the package, launch the ```robot.launch.py``` file from the ```gazebo_differential_drive_robot``` package:

```bash
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot.launch.py
```

### Launch the Robot with Lidar Object Detection

To launch the robot with lidar-based object detection enabled:

```bash
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py
```

For detailed information about lidar object detection, see [LIDAR_DETECTION.md](../LIDAR_DETECTION.md).

### Launch the Robot with Full Navigation Support

To launch the robot with full navigation support (includes lidar detection, scan-to-image, navigation support, and RViz2):

```bash
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot_navigation.launch.py
```

This launch file provides:
- Object detection using lidar
- Navigation zone analysis and safe direction recommendations
- Scan-to-image map generation
- Pre-configured RViz2 visualization
- Ready for velocity commands via `/cmd_vel` topic

### Launch the Robot with Reactive Navigation (Object Detection-Based Navigation)

For autonomous navigation based on object detection (e.g., "move forward until close to wall, then turn"):

```bash
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot_reactive_navigation.launch.py
```

Then send navigation commands:

```bash
# Move forward until wall is detected within 0.5m, then turn left
ros2 topic pub /navigation/command std_msgs/msg/String "data: 'forward_until_close:front:0.5:turn_left'" --once

# Or run the examples script
ros2 run gazebo_differential_drive_robot navigation_examples.py
```

For detailed information about reactive navigation, see [REACTIVE_NAVIGATION.md](../REACTIVE_NAVIGATION.md).

To launch the robot in a specified world with a custom initial pose, run the `robot.launch.py` file and specify the world and robot pose arguments.


- **world**: World to load. Supports several formats:
  - **By number**: `1` through `10` (e.g., `world:=1` for empty, `world:=5` for corridor)
  - **By name**: `empty`, `boxes`, `cylinders`, `mixed`, `corridor`, `scattered`, `zigzag`, `corners`, `slalom`, `challenge`
  - **By filename**: `world_01_empty.sdf` or `world_01_empty`
  - **By full path**: `/path_to_world/world.sdf`
- **x**: Initial x-coordinate of the robot
- **y**: Initial y-coordinate of the robot
- **z**: Initial z-coordinate of the robot
- **R**: Initial roll orientation
- **P**: Initial pitch orientation
- **Y**: Initial yaw orientation

#### World Shortcuts

| Number | Name       | Description                    |
|--------|------------|--------------------------------|
| 1      | empty      | Empty world                    |
| 2      | boxes      | World with box obstacles       |
| 3      | cylinders  | World with cylinder obstacles  |
| 4      | mixed      | Mixed obstacles                |
| 5      | corridor   | Corridor environment           |
| 6      | scattered  | Scattered obstacles            |
| 7      | zigzag     | Zigzag path                    |
| 8      | corners    | Corner obstacles               |
| 9      | slalom     | Slalom course                  |
| 10     | challenge  | Challenge course               |

Examples:

```bash
# Using number shortcut
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=5

# Using name shortcut
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=corridor

# Using filename
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=world_05_corridor

# With custom initial pose
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=boxes x:=1.0 y:=2.0 z:=0.5 Y:=1.57

# Using full path (still supported)
ros2 launch gazebo_differential_drive_robot robot.launch.py world:=/path_to_world/world.sdf x:=1.0 y:=2.0 z:=0.5 R:=0.0 P:=0.0 Y:=1.57
```

### Control the Robot

#### Using a Joystick

In a new terminal, source the environment and launch the ```teleop-launch.py``` file from the ```teleop_twist_joy``` package. Adjust the joy_config parameter to match your joystick controller (e.g., xbox).

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

#### Using a Keyboard

If you don't have a joystick, you can control the robot using the ```teleop_twist_keyboard``` package. Run the following command:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Running with Docker

If you don't already have docker installed, you can install it
using the [docker installation instructions](https://docs.docker.com/engine/install/) for your operating system.
Be sure to follow the post-install instructions.

### Create and run the container

For Windows there is a handy `run.ps1` script for setting up the display service and running the container:
```powershell
cd docker
.\run.ps1
```

Otherwise, run docker compose (you might have to set the display output to see the UI, I only tested on Windows):
```bash
cd docker
docker compose up -d
```

### Launch Gazebo and see the robot

Enter the container:
```powershell
docker exec -it gz_diff_drive_robot bash
```

Launch ROS2:
```bash
ros2 launch gazebo_differential_drive_robot robot.launch.py
```

### Control the robot

Open another terminal and enter the container:
```powershell
docker exec -it gz_diff_drive_robot bash
```

Run the following command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
