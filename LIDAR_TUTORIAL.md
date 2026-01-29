# Step-by-Step Tutorial: Using Lidar Object Detection

This tutorial will guide you through using the lidar-based object detection functionality in the differential drive robot.

## Prerequisites

Before starting, ensure you have:
- ROS 2 Jazzy Jalisco installed
- Gazebo Harmonic installed
- The `gazebo_differential_drive_robot` package built
- Basic understanding of ROS 2 concepts

## Step 1: Understanding What Was Added

The lidar functionality includes:

1. **Hardware (in simulation)**:
   - A 360-degree lidar sensor mounted on top of the robot
   - Visible as a blue cylinder in Gazebo

2. **Software**:
   - `lidar_object_detector.py`: Detects objects by clustering lidar points
   - `simple_lidar_reader.py`: Example script for reading lidar data
   - Bridge configuration to connect Gazebo to ROS 2

## Step 2: Launch the Robot with Lidar

Open a terminal and run:

```bash
cd /path/to/your/workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py
```

You should see:
- Gazebo window with the robot
- A blue cylinder (lidar sensor) on top of the robot
- Terminal output from the lidar object detector

## Step 3: Verify Lidar is Working

In a new terminal, check if lidar data is being published:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Check topic list
ros2 topic list | grep lidar

# View lidar scan rate
ros2 topic hz /lidar/scan

# Echo a few messages
ros2 topic echo /lidar/scan --once
```

Expected output:
- `/lidar/scan` topic should be listed
- Update rate should be around 10 Hz
- LaserScan messages with 360 range values

## Step 4: Add Objects to Detect

In Gazebo:
1. Click on the "Insert" tab on the left panel
2. Scroll down and select a model (e.g., "Box" or "Cylinder")
3. Click in the simulation to place it near the robot (within 5 meters)
4. Repeat to add 2-3 objects around the robot

## Step 5: View Detected Objects

In a new terminal, subscribe to the detected objects topic:

```bash
ros2 topic echo /detected_objects
```

You should see MarkerArray messages with information about detected objects.

## Step 6: Visualize with RViz2

Launch RViz2 to see a visual representation:

```bash
rviz2
```

Configure RViz2:
1. In "Displays" panel, click "Add"
2. Add "LaserScan" display:
   - Set Topic: `/lidar/scan`
   - Set Size: 0.05
   - Set Color: Red
3. Add "MarkerArray" display:
   - Set Topic: `/detected_objects`
4. Set "Fixed Frame" in Global Options to `body_link`

You should now see:
- Red points showing lidar rays
- Green spheres at detected object locations
- Text labels showing object information

## Step 7: Move the Robot

While the simulation is running, open a new terminal and control the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keys shown to move the robot around. Watch how:
- The lidar scans update in RViz2
- Detected objects change as the robot moves
- New objects are detected when they come into range

## Step 8: Experiment with Parameters

Try adjusting the detection parameters. Stop the simulation (Ctrl+C) and restart with:

```bash
ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py \
    distance_threshold:=0.3 \
    min_cluster_size:=10
```

This will:
- Use a larger distance threshold (0.3m instead of 0.2m)
- Require more points per object (10 instead of 5)
- Result in fewer but more reliable detections

## Step 9: Read Lidar Data Programmatically

Run the example lidar reader script:

```bash
ros2 run gazebo_differential_drive_robot simple_lidar_reader.py
```

This script demonstrates:
- How to subscribe to lidar scan data
- Basic processing of scan data
- Calculating statistics (min, max, average distance)
- Detecting obstacles in specific directions

Watch the terminal output to see real-time lidar statistics.

## Step 10: Create Your Own Lidar Application

Use the provided scripts as templates to create your own application:

1. Copy `simple_lidar_reader.py`:
```bash
cd src/gazebo_differential_drive_robot/scripts
cp simple_lidar_reader.py my_lidar_app.py
```

2. Edit `my_lidar_app.py` to add your custom logic

3. Add it to `CMakeLists.txt`:
```cmake
install(
  PROGRAMS scripts/lidar_object_detector.py scripts/simple_lidar_reader.py scripts/my_lidar_app.py
  DESTINATION lib/${PROJECT_NAME}
)
```

4. Rebuild and test:
```bash
colcon build --packages-select gazebo_differential_drive_robot
source install/setup.bash
ros2 run gazebo_differential_drive_robot my_lidar_app.py
```

## Common Use Cases

### Obstacle Avoidance

Modify the object detector to publish obstacle warnings:
```python
if any(cluster_distance < 0.5 for cluster_distance in detected_distances):
    self.get_logger().warn('Obstacle too close!')
```

### Wall Following

Use lidar data to maintain a constant distance from walls:
```python
right_side_distances = scan.ranges[270:360]  # Right 90 degrees
target_distance = 1.0  # 1 meter from wall
# Implement control logic
```

### Environment Mapping

Accumulate lidar scans over time to build a map:
```python
# Store scan data with robot pose
self.scan_history.append((current_pose, scan_data))
# Use for SLAM or mapping
```

## Troubleshooting

### Problem: No lidar data appearing

**Solution:**
- Check if Gazebo is running: `ps aux | grep gz`
- Verify bridge is active: `ros2 topic list | grep lidar`
- Check for errors in terminal output

### Problem: Objects not being detected

**Solution:**
- Ensure objects are within lidar range (0.1m - 30m)
- Try lowering `min_cluster_size` parameter
- Add more/larger objects in Gazebo
- Check if lidar sensor is obscured

### Problem: Too many false detections

**Solution:**
- Increase `min_cluster_size` parameter
- Decrease `distance_threshold` parameter
- Check for noise in lidar data: `ros2 topic echo /lidar/scan`

### Problem: Performance issues

**Solution:**
- Reduce lidar update rate in `robot.xacro` (change `<update_rate>10</update_rate>`)
- Increase clustering thresholds
- Close unnecessary visualization windows

## Next Steps

Now that you understand the basics:

1. **Integrate with Navigation**: Connect lidar to ROS 2 Navigation stack
2. **Add Object Classification**: Classify detected objects by size/shape
3. **Implement Tracking**: Track objects over time with unique IDs
4. **Create Autonomous Behaviors**: Use object detection for autonomous navigation
5. **Optimize Performance**: Profile and optimize the clustering algorithm

## Additional Resources

- [ROS 2 LaserScan Documentation](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)
- [ROS 2 Visualization Markers](https://wiki.ros.org/rviz/DisplayTypes/Marker)
- [Gazebo Sensor Plugins](https://gazebosim.org/api/gazebo/7/classgazebo_1_1sensors_1_1GpuRaySensor.html)
- [Point Cloud Library (PCL) for advanced processing](https://pointclouds.org/)

## Summary

You have learned how to:
- ✅ Launch the robot with lidar functionality
- ✅ Verify lidar data is being published
- ✅ Add objects and detect them
- ✅ Visualize results in RViz2
- ✅ Control the robot while scanning
- ✅ Adjust detection parameters
- ✅ Create custom lidar applications

The lidar system is now ready for your robotics projects!
