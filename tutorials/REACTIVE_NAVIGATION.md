# Reactive Navigation with Object Detection

This document explains how to use the reactive navigation capabilities added to the differential drive robot. The reactive navigation controller enables object detection-based navigation, allowing commands such as "move forward until you get close to a wall, then turn left."

## Overview

The reactive navigation system consists of:

1. **Reactive Navigation Controller** (`reactive_navigation_controller.py`)
   - Core node that executes navigation commands
   - Monitors LiDAR data for obstacle detection
   - Supports condition-based movement
   - Publishes velocity commands to `/cmd_vel`

2. **Navigation Examples** (`navigation_examples.py`)
   - Demonstration script with example commands
   - Reference implementation for integrating navigation

## Quick Start

### Launch the Robot with Reactive Navigation

```bash
# Build the package
source /opt/ros/jazzy/setup.bash
cd <path_to_your_workspace>
colcon build

# Launch robot with reactive navigation
source install/setup.bash
ros2 launch gazebo_differential_drive_robot robot_reactive_navigation.launch.py
```

### Send a Navigation Command

In a separate terminal:

```bash
# Move forward until front obstacle is within 0.5 meters, then turn left
ros2 topic pub /navigation/command std_msgs/msg/String "data: 'forward_until_close:front:0.5:turn_left'" --once
```

### Run the Example Script

```bash
ros2 run gazebo_differential_drive_robot navigation_examples.py
```

## Command Reference

### Basic Movement Commands

| Command | Description | Example |
|---------|-------------|---------|
| `stop` | Stop all movement | `stop` |
| `forward` | Move forward indefinitely | `forward` |
| `forward:<distance>` | Move forward for specified meters | `forward:2.0` |
| `backward` | Move backward indefinitely | `backward` |
| `backward:<distance>` | Move backward for specified meters | `backward:1.0` |
| `turn_left` | Turn left 90 degrees | `turn_left` |
| `turn_left:<angle>` | Turn left by specified degrees | `turn_left:45` |
| `turn_right` | Turn right 90 degrees | `turn_right` |
| `turn_right:<angle>` | Turn right by specified degrees | `turn_right:90` |

### Condition-Based Commands

These commands are the key to reactive navigation based on object detection:

| Command | Description | Example |
|---------|-------------|---------|
| `forward_until_close:<direction>:<distance>` | Move forward until obstacle in direction is within distance | `forward_until_close:front:0.5` |
| `forward_until_close:<direction>:<distance>:<next_action>` | Same as above, then execute next action | `forward_until_close:front:0.5:turn_left` |
| `backward_until_close:<direction>:<distance>` | Move backward until condition met | `backward_until_close:back:0.5` |
| `turn_until_clear:<direction>:<distance>` | Turn until front is clear | `turn_until_clear:left:1.0` |
| `follow_wall:<side>:<distance>` | Follow wall maintaining distance | `follow_wall:left:0.5` |
| `follow_wall:<side>:<distance>:<duration>` | Follow wall for specified distance | `follow_wall:left:0.5:10.0` |

### Directions

The following directions are supported for obstacle detection:

- `front` - Directly in front of the robot
- `front_left` - Front-left quadrant
- `front_right` - Front-right quadrant
- `left` - Directly to the left
- `right` - Directly to the right
- `back` - Directly behind
- `back_left` - Back-left quadrant
- `back_right` - Back-right quadrant
- `any` - Minimum distance in any direction

### Chaining Commands

Multiple commands can be chained using semicolons:

```bash
# Move forward, turn, move forward again
ros2 topic pub /navigation/command std_msgs/msg/String "data: 'forward:1.0;turn_left:90;forward:2.0'" --once
```

### JSON Format

Commands can also be sent in JSON format for more structured specification:

```bash
# Single command
ros2 topic pub /navigation/command std_msgs/msg/String 'data: "{\"action\": \"forward\", \"distance\": 2.0}"' --once

# Multiple commands
ros2 topic pub /navigation/command std_msgs/msg/String 'data: "[{\"action\": \"forward\", \"distance\": 1.0}, {\"action\": \"turn_left\", \"angle\": 90}]"' --once
```

JSON command fields:
- `action` (required): The command action
- `direction`: Direction for turns or wall following (left/right)
- `distance`: Distance in meters for movement commands
- `angle`: Angle in degrees for turn commands
- `condition_direction`: Direction for condition checking
- `condition_distance`: Distance threshold for conditions
- `next_action`: Action to execute when condition is met

## Example Use Cases

### 1. Move Forward Until Close to Wall, Then Turn

This is the primary use case for reactive navigation:

```bash
ros2 topic pub /navigation/command std_msgs/msg/String "data: 'forward_until_close:front:0.5:turn_left'" --once
```

The robot will:
1. Move forward
2. Continuously monitor the front LiDAR readings
3. Stop when front distance is ≤ 0.5 meters
4. Turn left 90 degrees

### 2. Navigate Around an Obstacle

```bash
ros2 topic pub /navigation/command std_msgs/msg/String "data: 'forward_until_close:front:0.5;turn_left:90;forward:1.5;turn_right:90;forward:2.0'" --once
```

This creates a sequence to:
1. Move forward until obstacle detected
2. Turn left 90°
3. Move forward 1.5 meters
4. Turn right 90°
5. Move forward 2 meters

### 3. Wall Following

```bash
# Follow left wall at 0.5m distance for 10 meters
ros2 topic pub /navigation/command std_msgs/msg/String "data: 'follow_wall:left:0.5:10.0'" --once
```

### 4. Find a Clear Path

When in a tight space, turn until a clear path is found:

```bash
# Turn left until front clearance is at least 2 meters
ros2 topic pub /navigation/command std_msgs/msg/String "data: 'turn_until_clear:left:2.0'" --once
```

### 5. Programmatic Control

For integration with your own applications, here's example Python code:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MyNavigationApp(Node):
    def __init__(self):
        super().__init__('my_navigation_app')
        
        # Publisher for navigation commands
        self.command_pub = self.create_publisher(
            String,
            '/navigation/command',
            10
        )
        
        # Subscribe to navigation status
        self.status_sub = self.create_subscription(
            String,
            '/navigation/status',
            self.status_callback,
            10
        )
        
        # Subscribe to condition triggers
        self.trigger_sub = self.create_subscription(
            String,
            '/navigation/condition_triggered',
            self.trigger_callback,
            10
        )
    
    def send_command(self, command: str):
        """Send a navigation command."""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
    
    def status_callback(self, msg: String):
        """Handle navigation status updates."""
        status = json.loads(msg.data)
        print(f"State: {status['state']}, Current: {status['current_command']}")
    
    def trigger_callback(self, msg: String):
        """Handle condition trigger events."""
        data = json.loads(msg.data)
        print(f"Condition triggered: {data['condition']} = {data['value']:.2f}m")
        
        # React to the condition
        if data['condition'] == 'front':
            # Obstacle detected in front, decide what to do next
            self.send_command('turn_left:90')

def main():
    rclpy.init()
    node = MyNavigationApp()
    
    # Send initial navigation command
    node.send_command('forward_until_close:front:0.5')
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan data for obstacle detection |
| `/navigation/command` | `std_msgs/msg/String` | Navigation commands |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands for robot movement |
| `/navigation/status` | `std_msgs/msg/String` | Current navigation status (JSON) |
| `/navigation/state` | `std_msgs/msg/String` | Navigation state changes |
| `/navigation/condition_triggered` | `std_msgs/msg/String` | Condition trigger events (JSON) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/navigation/stop` | `std_srvs/srv/Trigger` | Stop all navigation |
| `/navigation/pause` | `std_srvs/srv/Trigger` | Pause current navigation |
| `/navigation/resume` | `std_srvs/srv/Trigger` | Resume paused navigation |

## Parameters

The reactive navigation controller can be configured via parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_speed` | 0.3 | Linear velocity (m/s) |
| `angular_speed` | 0.5 | Angular velocity (rad/s) |
| `default_obstacle_threshold` | 0.5 | Default obstacle distance threshold (m) |
| `safety_threshold` | 0.3 | Emergency stop distance (m) |
| `min_valid_range` | 0.1 | Minimum valid LiDAR range (m) |
| `max_valid_range` | 10.0 | Maximum valid LiDAR range (m) |
| `front_cone_angle` | 45.0 | Front detection cone angle (degrees) |
| `side_cone_angle` | 60.0 | Side detection cone angle (degrees) |

### Setting Parameters at Launch

```bash
ros2 launch gazebo_differential_drive_robot robot_reactive_navigation.launch.py \
    linear_speed:=0.5 \
    angular_speed:=0.8 \
    obstacle_threshold:=1.0
```

## Integration with LLM

If you want to use a Large Language Model (LLM) to generate navigation commands from natural language, here's a suggested approach:

### 1. Command Generation Prompt

Provide the LLM with the command reference and ask it to generate commands:

```
Given the following navigation command syntax:
- forward_until_close:<direction>:<distance>:<next_action>
- turn_left:<angle>
- turn_right:<angle>
- follow_wall:<side>:<distance>

Available directions: front, front_left, front_right, left, right, back

User request: "Move forward until you get close to a wall, then turn left and continue for 2 meters"

Generate the navigation command:
```

### 2. Example LLM Response

```
forward_until_close:front:0.5:turn_left;forward:2.0
```

### 3. Integration Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  User Input     │────▶│  LLM           │────▶│  Navigation     │
│  (Natural       │     │  (Command       │     │  Controller     │
│   Language)     │     │   Generator)    │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                        │
                                                        ▼
                                               ┌─────────────────┐
                                               │  Robot          │
                                               │  (Executes      │
                                               │   Commands)     │
                                               └─────────────────┘
```

The LLM can be hosted separately and communicate with your ROS2 application via:
- REST API
- WebSocket
- ROS2 action/service

## Troubleshooting

### Robot Doesn't Move

1. Check if LiDAR data is being published:
   ```bash
   ros2 topic hz /lidar/scan
   ```

2. Check navigation controller status:
   ```bash
   ros2 topic echo /navigation/status
   ```

3. Verify command was received:
   ```bash
   ros2 topic echo /navigation/state
   ```

### Robot Stops Unexpectedly

The safety threshold may be triggering emergency stops:
- Check `/navigation/condition_triggered` for `emergency_stop` events
- Increase `safety_threshold` if the environment is safe but tight
- Verify obstacles aren't too close to the robot

### Commands Not Executing

1. Check if the navigation state is `idle`:
   ```bash
   ros2 topic echo /navigation/status --once
   ```

2. Stop any pending navigation:
   ```bash
   ros2 service call /navigation/stop std_srvs/srv/Trigger
   ```

### Wall Following Oscillates

Adjust the following:
- Decrease linear speed for more stable control
- Ensure the target wall distance is achievable in your environment
- Check LiDAR data quality for the specified side

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Gazebo Simulation                            │
│  ┌──────────────┐                    ┌──────────────┐              │
│  │   Robot      │◀───────────────────│    LiDAR     │              │
│  │              │                    │   Sensor     │              │
│  └──────────────┘                    └──────┬───────┘              │
└─────────────────────────────────────────────┼───────────────────────┘
                                              │ /lidar (gz.msgs)
                                              ▼
                                    ┌──────────────────┐
                                    │   ROS-Gazebo     │
                                    │     Bridge       │
                                    └────────┬─────────┘
                                             │ /lidar/scan (LaserScan)
                                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     Reactive Navigation Controller                   │
│                                                                      │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐          │
│  │   Command    │───▶│   Control    │───▶│   Velocity   │          │
│  │   Parser     │    │    Logic     │    │   Publisher  │          │
│  └──────────────┘    └──────────────┘    └──────────────┘          │
│         ▲                   │                    │                   │
│         │                   │                    │ /cmd_vel          │
│         │                   ▼                    ▼                   │
│  /navigation/command   ┌──────────────┐    ┌──────────────┐         │
│                        │   Distance   │    │   Status     │         │
│                        │   Monitor    │    │   Publisher  │         │
│                        └──────────────┘    └──────────────┘         │
│                              ▲                   │                   │
│                              │                   │ /navigation/status│
│                     /lidar/scan                  ▼                   │
└─────────────────────────────────────────────────────────────────────┘
                                              
```

## Files Added

| File | Description |
|------|-------------|
| `src/scripts/reactive_navigation_controller.py` | Main reactive navigation node |
| `src/scripts/navigation_examples.py` | Example script demonstrating usage |
| `src/launch/robot_reactive_navigation.launch.py` | Launch file for reactive navigation |
| `REACTIVE_NAVIGATION.md` | This documentation file |

## Next Steps

1. **Test Basic Commands**: Start with simple forward/turn commands
2. **Test Condition-Based Navigation**: Try `forward_until_close` commands
3. **Build Complex Sequences**: Chain commands for complex behaviors
4. **Integrate with Your Application**: Use the example code as reference
5. **Consider LLM Integration**: For natural language command generation

## References

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [LaserScan Message](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)
- [Twist Message](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)
- [LiDAR Object Detection](LIDAR_DETECTION.md)
- [LiDAR Tutorial](LIDAR_TUTORIAL.md)
