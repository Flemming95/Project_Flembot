# Lidar Object Detection

This document explains how to use the lidar-based object detection functionality added to the differential drive robot.

## Overview

The robot has been equipped with a lidar sensor that can detect objects in its environment. The lidar sensor performs 360-degree scans and publishes laser scan data which is then processed by a Python node to detect and track objects.

## Architecture

### Hardware (Simulation)

1. **Lidar Sensor**: A GPU-based lidar sensor mounted on top of the robot body
   - Type: `gpu_lidar`
   - Range: 0.1m to 30.0m
   - Resolution: 360 samples per scan
   - Update rate: 10 Hz
   - Field of view: 360 degrees

### Software Components

1. **Lidar Sensor Plugin** (`robot.xacro`)
   - Gazebo plugin that simulates lidar scanning
   - Publishes to `/lidar` topic in Gazebo

2. **ROS-Gazebo Bridge** (`gz_bridge.yaml`)
   - Bridges lidar data from Gazebo to ROS2
   - Maps `/lidar` (Gazebo) → `/lidar/scan` (ROS2)

3. **Lidar Object Detector Node** (`lidar_object_detector.py`)
   - Subscribes to `/lidar/scan` topic (LaserScan messages)
   - Clusters nearby points to identify objects
   - Publishes detected objects to `/detected_objects` topic (MarkerArray)

## Object Detection Algorithm

The detection algorithm uses a simple clustering approach:

1. **Scan to Cartesian Conversion**: Converts polar coordinates (range, angle) to Cartesian (x, y)
2. **Point Filtering**: Removes invalid points (inf, nan)
3. **Clustering**: Groups points that are within a distance threshold using nearest-neighbor clustering
4. **Object Identification**: Clusters with sufficient points are considered detected objects
5. **Visualization**: Publishes sphere markers at object centroids with labels

### Configurable Parameters

- `distance_threshold` (default: 0.2m): Maximum distance between points in the same cluster
- `min_cluster_size` (default: 5 points): Minimum points required to form an object
- `max_cluster_size` (default: 100 points): Maximum points in a single cluster

## Usage

### Basic Launch

To launch the robot with lidar object detection:

```bash
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py
```

### Launch with Custom Parameters

You can customize the object detection parameters:

```bash
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py \
    distance_threshold:=0.3 \
    min_cluster_size:=10 \
    max_cluster_size:=50
```

### Launch in a Custom World

To test object detection in a custom world:

```bash
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py \
    world:=/path/to/your/world.sdf \
    x:=0.0 y:=0.0 z:=0.5
```

### Running Components Separately

If you want to launch the robot and object detector separately:

1. Launch the robot:
```bash
ros2 launch gazebo_differential_drive_robot robot.launch.py
```

2. In a separate terminal, launch the object detector:
```bash
ros2 run gazebo_differential_drive_robot lidar_object_detector.py \
    --ros-args \
    -p distance_threshold:=0.2 \
    -p min_cluster_size:=5 \
    -p max_cluster_size:=100
```

## Visualizing Detected Objects

### Using RViz2

To visualize the detected objects and lidar scan in RViz2:

1. Launch RViz2:
```bash
rviz2
```

2. Configure the display:
   - Set "Fixed Frame" to `body_link`
   - Add display: `LaserScan`
     - Topic: `/lidar/scan`
   - Add display: `MarkerArray`
     - Topic: `/detected_objects`

### Viewing in Gazebo

The lidar sensor visualization is enabled by default in Gazebo. You should see the lidar rays when the simulation is running.

## Topics

### Subscribed Topics

- `/lidar/scan` ([sensor_msgs/LaserScan](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))
  - Lidar scan data from the robot

### Published Topics

- `/detected_objects` ([visualization_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html))
  - Visualization markers for detected objects
  - Green spheres represent object centroids
  - White text labels show object ID and point count

## Testing Object Detection

### Simple Test in Empty World

1. Launch the robot:
```bash
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py
```

2. In Gazebo, add objects manually:
   - Click "Insert" tab
   - Add boxes, cylinders, or other models near the robot

3. Monitor detected objects:
```bash
ros2 topic echo /detected_objects
```

### Moving the Robot

To move the robot and scan different areas:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or with a joystick:
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

## Troubleshooting

### No Objects Detected

- Check if lidar data is being published:
  ```bash
  ros2 topic hz /lidar/scan
  ```
- Verify objects are within lidar range (0.1m - 30m)
- Adjust `min_cluster_size` parameter to a lower value
- Increase `distance_threshold` for objects further apart

### Too Many False Positives

- Increase `min_cluster_size` to filter small clusters
- Decrease `distance_threshold` to require tighter grouping
- Check for noise in the lidar data

### Performance Issues

- Reduce lidar update rate in `robot.xacro`
- Increase `distance_threshold` to create larger clusters
- Limit `max_cluster_size` to reduce processing time

## Code Structure

```
src/
├── model/
│   └── robot.xacro              # Robot model with lidar sensor
├── config/
│   └── gz_bridge.yaml           # ROS-Gazebo bridge configuration
├── launch/
│   ├── robot.launch.py          # Base robot launch file
│   └── robot_with_lidar.launch.py  # Robot + object detection
└── scripts/
    └── lidar_object_detector.py # Object detection node
```

## Navigation Support

The lidar sensor is well-suited for object detection-based navigation. The package now includes additional nodes for navigation support:

### Navigation Components

1. **Lidar Navigation Support Node** (`lidar_navigation_support.py`)
   - Analyzes lidar scans and provides zone-based obstacle detection
   - Publishes sector distances and obstacle zones
   - Provides safe direction recommendations
   - Publishes recommended velocities (for reference)

2. **Scan to Image Node** (`scan_to_image.py`)
   - Converts lidar scans to image maps
   - Publishes images for visualization
   - Optionally saves images to files

3. **RViz Configuration** (`robot_navigation.rviz`)
   - Pre-configured to display robot, lidar scans, detected objects, and odometry
   - Ready for tracking robot movement

### Navigation Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/navigation/sector_distances` | `std_msgs/msg/Float32MultiArray` | Minimum distance in each angular sector |
| `/navigation/obstacle_zones` | `std_msgs/msg/String` | JSON with obstacle status for each zone |
| `/navigation/safe_direction` | `std_msgs/msg/Float32MultiArray` | Recommended safe direction (angle, confidence) |
| `/navigation/recommended_velocity` | `geometry_msgs/msg/Twist` | Conservative velocity recommendation |
| `/navigation/markers` | `visualization_msgs/msg/MarkerArray` | Visual markers for navigation data |
| `/scan_image` | `sensor_msgs/msg/Image` | Visual image of the lidar scan |

### Sending Velocity Commands

Velocity commands can be sent via a separate publisher to the `/cmd_vel` topic:

```bash
# Using teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### Launching with Full Navigation Support

```bash
ros2 launch gazebo_differential_drive_robot robot_navigation.launch.py
```

This will start:
- Robot simulation in Gazebo
- Lidar object detection
- Navigation support node
- Scan-to-image converter
- RViz2 with pre-configured visualization

## Future Enhancements

Potential improvements to the object detection system:

1. **Advanced Clustering**: Implement DBSCAN or other sophisticated clustering algorithms
2. **Object Classification**: Classify detected objects by shape or size
3. **Object Tracking**: Track objects over time with unique IDs
4. **Obstacle Avoidance**: Integrate with navigation stack for autonomous obstacle avoidance
5. **3D Detection**: Use 3D lidar data for height estimation
6. **Machine Learning**: Use trained models for object recognition

## References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [LaserScan Message](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)
- [Visualization Markers](https://wiki.ros.org/rviz/DisplayTypes/Marker)
