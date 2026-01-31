#!/usr/bin/env python3

"""
Reactive Navigation Controller

This ROS2 node provides reactive navigation capabilities based on LiDAR object detection.
It allows the robot to execute navigation behaviors with distance-based conditions,
such as "move forward until you get close to a wall, then turn left."

The controller supports:
- Condition-based movement (distance triggers)
- Sequential command execution
- Real-time obstacle monitoring
- Multiple navigation behaviors

Usage:
    The controller subscribes to /lidar/scan for obstacle detection and publishes
    velocity commands to /cmd_vel. Navigation commands can be sent via services
    or the /navigation/command topic.

Example commands:
    - "forward_until_close:front:0.5:turn_left"
    - "turn_right:90"
    - "forward:2.0"
    - "stop"
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import math
import json
import threading
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Dict


class NavigationState(Enum):
    """Enum representing the current state of the navigation controller."""
    IDLE = "idle"
    EXECUTING = "executing"
    PAUSED = "paused"
    COMPLETED = "completed"
    ERROR = "error"


class Direction(Enum):
    """Enum representing directions for obstacle detection."""
    FRONT = "front"
    FRONT_LEFT = "front_left"
    FRONT_RIGHT = "front_right"
    LEFT = "left"
    RIGHT = "right"
    BACK = "back"
    BACK_LEFT = "back_left"
    BACK_RIGHT = "back_right"
    ANY = "any"


@dataclass
class NavigationCommand:
    """Data class representing a navigation command."""
    action: str
    direction: Optional[str] = None
    distance: Optional[float] = None
    angle: Optional[float] = None
    condition_direction: Optional[str] = None
    condition_distance: Optional[float] = None
    next_action: Optional[str] = None


class ReactiveNavigationController(Node):
    """
    A ROS2 node that provides reactive navigation based on LiDAR object detection.
    
    This controller enables condition-based navigation where the robot can execute
    movements until certain distance conditions are met, then transition to new
    behaviors.
    
    Attributes:
        state: Current navigation state
        current_command: Currently executing navigation command
        command_queue: Queue of pending navigation commands
    """

    def __init__(self):
        super().__init__('reactive_navigation_controller')
        
        # Callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.3)  # m/s
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('default_obstacle_threshold', 0.5)  # meters
        self.declare_parameter('safety_threshold', 0.3)  # meters - emergency stop
        self.declare_parameter('min_valid_range', 0.1)  # meters
        self.declare_parameter('max_valid_range', 10.0)  # meters
        self.declare_parameter('front_cone_angle', 45.0)  # degrees
        self.declare_parameter('side_cone_angle', 60.0)  # degrees
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.default_obstacle_threshold = self.get_parameter('default_obstacle_threshold').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.min_valid_range = self.get_parameter('min_valid_range').value
        self.max_valid_range = self.get_parameter('max_valid_range').value
        self.front_cone_angle = math.radians(self.get_parameter('front_cone_angle').value)
        self.side_cone_angle = math.radians(self.get_parameter('side_cone_angle').value)
        
        # Navigation state
        self.state = NavigationState.IDLE
        self.current_command: Optional[NavigationCommand] = None
        self.command_queue: List[NavigationCommand] = []
        self.state_lock = threading.Lock()
        
        # Sensor data storage
        self.latest_scan: Optional[LaserScan] = None
        self.direction_distances: Dict[str, float] = {}
        
        # Movement tracking
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        self.last_control_time = None  # For actual elapsed time tracking
        self.current_velocity = Twist()
        
        # Create subscriber to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create subscriber for navigation commands
        self.command_sub = self.create_subscription(
            String,
            '/navigation/command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create publisher for navigation status
        self.status_pub = self.create_publisher(
            String,
            '/navigation/status',
            10
        )
        
        # Create publisher for navigation state
        self.state_pub = self.create_publisher(
            String,
            '/navigation/state',
            10
        )
        
        # Create publisher for condition triggered events
        self.condition_triggered_pub = self.create_publisher(
            String,
            '/navigation/condition_triggered',
            10
        )
        
        # Create services
        self.stop_srv = self.create_service(
            Trigger,
            '/navigation/stop',
            self.stop_service_callback,
            callback_group=self.callback_group
        )
        
        self.pause_srv = self.create_service(
            Trigger,
            '/navigation/pause',
            self.pause_service_callback,
            callback_group=self.callback_group
        )
        
        self.resume_srv = self.create_service(
            Trigger,
            '/navigation/resume',
            self.resume_service_callback,
            callback_group=self.callback_group
        )
        
        # Create timer for navigation control loop
        self.control_timer = self.create_timer(
            0.05,  # 20 Hz
            self.control_loop,
            callback_group=self.callback_group
        )
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_status,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Reactive Navigation Controller initialized')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'Default obstacle threshold: {self.default_obstacle_threshold} m')
        self.get_logger().info(f'Safety threshold: {self.safety_threshold} m')
        self.get_logger().info('Listening for commands on /navigation/command')

    def scan_callback(self, msg: LaserScan):
        """
        Callback function for processing lidar scan data.
        
        Updates the direction distances based on the latest scan.
        
        Args:
            msg: The incoming laser scan message
        """
        self.latest_scan = msg
        self.direction_distances = self._calculate_direction_distances(msg)

    def _calculate_direction_distances(self, msg: LaserScan) -> Dict[str, float]:
        """
        Calculate minimum distance in each direction from the scan data.
        
        Args:
            msg: The laser scan message
            
        Returns:
            Dictionary mapping direction names to minimum distances
        """
        distances = {
            Direction.FRONT.value: float('inf'),
            Direction.FRONT_LEFT.value: float('inf'),
            Direction.FRONT_RIGHT.value: float('inf'),
            Direction.LEFT.value: float('inf'),
            Direction.RIGHT.value: float('inf'),
            Direction.BACK.value: float('inf'),
            Direction.BACK_LEFT.value: float('inf'),
            Direction.BACK_RIGHT.value: float('inf'),
            Direction.ANY.value: float('inf'),
        }
        
        # Define angle ranges for each direction (in radians, 0 = front)
        # Angles follow ROS convention: positive = left (counter-clockwise)
        # Note: BACK and BACK_RIGHT require special handling due to angle wrapping at ±pi
        direction_ranges = {
            Direction.FRONT.value: (-self.front_cone_angle / 2, self.front_cone_angle / 2),
            Direction.FRONT_LEFT.value: (self.front_cone_angle / 2, math.pi / 2),
            Direction.LEFT.value: (math.pi / 2 - self.side_cone_angle / 2, math.pi / 2 + self.side_cone_angle / 2),
            Direction.BACK_LEFT.value: (math.pi / 2, math.pi - self.front_cone_angle / 2),
            Direction.RIGHT.value: (-math.pi / 2 - self.side_cone_angle / 2, -math.pi / 2 + self.side_cone_angle / 2),
            Direction.FRONT_RIGHT.value: (-math.pi / 2, -self.front_cone_angle / 2),
        }
        
        angle = msg.angle_min
        for r in msg.ranges:
            if self.min_valid_range <= r <= self.max_valid_range and math.isfinite(r):
                # Normalize angle to [-pi, pi]
                normalized_angle = math.atan2(math.sin(angle), math.cos(angle))
                
                # Check each direction
                for direction, (min_angle, max_angle) in direction_ranges.items():
                    if min_angle <= normalized_angle <= max_angle:
                        distances[direction] = min(distances[direction], r)
                
                # Update "any" direction with minimum overall distance
                distances[Direction.ANY.value] = min(distances[Direction.ANY.value], r)
            
            angle += msg.angle_increment
        
        # Handle back direction which wraps around ±pi
        back_min = float('inf')
        back_right_min = float('inf')
        angle = msg.angle_min
        for r in msg.ranges:
            if self.min_valid_range <= r <= self.max_valid_range and math.isfinite(r):
                normalized_angle = math.atan2(math.sin(angle), math.cos(angle))
                # Back: angles close to ±pi
                if abs(normalized_angle) > math.pi - self.front_cone_angle / 2:
                    back_min = min(back_min, r)
                # Back-right: negative angles between -pi and -pi/2
                if -math.pi + self.front_cone_angle / 2 < normalized_angle < -math.pi / 2:
                    back_right_min = min(back_right_min, r)
            angle += msg.angle_increment
        distances[Direction.BACK.value] = back_min
        distances[Direction.BACK_RIGHT.value] = back_right_min
        
        # Replace inf with max_valid_range for directions with no readings
        for direction in distances:
            if distances[direction] == float('inf'):
                distances[direction] = self.max_valid_range
        
        return distances

    def command_callback(self, msg: String):
        """
        Callback for receiving navigation commands.
        
        Parses the command string and queues it for execution.
        
        Args:
            msg: The command string message
        """
        try:
            commands = self.parse_command(msg.data)
            if commands:
                with self.state_lock:
                    for cmd in commands:
                        self.command_queue.append(cmd)
                        self.get_logger().info(f'Command queued: {cmd.action}')
                    
                    # Start executing if we're idle or if previous commands completed
                    if self.state in (NavigationState.IDLE, NavigationState.COMPLETED):
                        self._start_next_command()
        except ValueError as e:
            self.get_logger().error(f'Failed to parse command: {e}')
            self._publish_error(str(e))

    def parse_command(self, command_str: str) -> List[NavigationCommand]:
        """
        Parse a command string into NavigationCommand objects.
        
        Command formats:
        - "stop" - Stop all movement
        - "forward" - Move forward indefinitely
        - "forward:2.0" - Move forward for 2.0 meters
        - "forward_until_close:front:0.5" - Move forward until front is within 0.5m
        - "forward_until_close:front:0.5:turn_left" - Move forward until close, then turn left
        - "turn_left" or "turn_left:90" - Turn left (optionally specify degrees)
        - "turn_right" or "turn_right:90" - Turn right
        - "backward" or "backward:1.0" - Move backward
        - Multiple commands separated by ";" are executed sequentially
        - JSON format: {"action": "forward", "distance": 2.0, ...}
        
        Args:
            command_str: The command string to parse
            
        Returns:
            List of NavigationCommand objects
        """
        commands = []
        
        # Try JSON format first
        if command_str.strip().startswith('{') or command_str.strip().startswith('['):
            return self._parse_json_command(command_str)
        
        # Split multiple commands by semicolon
        command_parts = command_str.strip().split(';')
        
        for part in command_parts:
            part = part.strip()
            if not part:
                continue
            
            tokens = part.split(':')
            action = tokens[0].lower().strip()
            
            cmd = NavigationCommand(action=action)
            
            if action == 'stop':
                pass  # No additional parameters needed
                
            elif action in ['forward', 'backward']:
                if len(tokens) > 1:
                    cmd.distance = float(tokens[1])
                    
            elif action in ['turn_left', 'turn_right']:
                if len(tokens) > 1:
                    cmd.angle = float(tokens[1])
                else:
                    cmd.angle = 90.0  # Default to 90 degrees
                    
            elif action == 'forward_until_close':
                if len(tokens) < 3:
                    raise ValueError(
                        "forward_until_close requires format: "
                        "forward_until_close:direction:distance[:next_action]"
                    )
                cmd.condition_direction = tokens[1].lower()
                cmd.condition_distance = float(tokens[2])
                if len(tokens) > 3:
                    cmd.next_action = tokens[3]
                    
            elif action == 'backward_until_close':
                if len(tokens) < 3:
                    raise ValueError(
                        "backward_until_close requires format: "
                        "backward_until_close:direction:distance[:next_action]"
                    )
                cmd.condition_direction = tokens[1].lower()
                cmd.condition_distance = float(tokens[2])
                if len(tokens) > 3:
                    cmd.next_action = tokens[3]
                    
            elif action == 'turn_until_clear':
                if len(tokens) < 3:
                    raise ValueError(
                        "turn_until_clear requires format: "
                        "turn_until_clear:direction_to_turn:min_clear_distance"
                    )
                cmd.direction = tokens[1].lower()  # left or right
                cmd.condition_distance = float(tokens[2])
                
            elif action == 'follow_wall':
                if len(tokens) < 3:
                    raise ValueError(
                        "follow_wall requires format: "
                        "follow_wall:side:desired_distance"
                    )
                cmd.direction = tokens[1].lower()  # left or right
                cmd.distance = float(tokens[2])
                if len(tokens) > 3:
                    cmd.condition_distance = float(tokens[3])  # duration or distance limit
                    
            else:
                raise ValueError(f"Unknown command action: {action}")
            
            commands.append(cmd)
        
        return commands

    def _parse_json_command(self, json_str: str) -> List[NavigationCommand]:
        """
        Parse a JSON-formatted command string.
        
        Args:
            json_str: JSON string or array of command objects
            
        Returns:
            List of NavigationCommand objects
        """
        data = json.loads(json_str)
        
        if isinstance(data, list):
            return [self._json_to_command(item) for item in data]
        else:
            return [self._json_to_command(data)]

    def _json_to_command(self, data: dict) -> NavigationCommand:
        """
        Convert a JSON dictionary to a NavigationCommand.
        
        Args:
            data: Dictionary with command parameters
            
        Returns:
            NavigationCommand object
        """
        return NavigationCommand(
            action=data.get('action', 'stop'),
            direction=data.get('direction'),
            distance=data.get('distance'),
            angle=data.get('angle'),
            condition_direction=data.get('condition_direction'),
            condition_distance=data.get('condition_distance'),
            next_action=data.get('next_action')
        )

    def control_loop(self):
        """
        Main control loop that executes navigation commands.
        
        Called periodically by the control timer.
        """
        with self.state_lock:
            if self.state != NavigationState.EXECUTING:
                return
            
            if self.current_command is None:
                self._start_next_command()
                return
            
            # Safety check - emergency stop if obstacle too close
            if self._check_emergency_stop():
                self._emergency_stop()
                return
            
            # Execute current command
            completed = self._execute_current_command()
            
            if completed:
                self._handle_command_completion()

    def _check_emergency_stop(self) -> bool:
        """
        Check if an emergency stop is needed due to obstacle proximity.
        
        Returns:
            True if emergency stop should be triggered
        """
        if not self.direction_distances:
            return False
        
        # Check front direction when moving forward
        if self.current_velocity.linear.x > 0:
            front_dist = self.direction_distances.get(Direction.FRONT.value, float('inf'))
            if front_dist < self.safety_threshold:
                return True
        
        # Check back direction when moving backward
        if self.current_velocity.linear.x < 0:
            back_dist = self.direction_distances.get(Direction.BACK.value, float('inf'))
            if back_dist < self.safety_threshold:
                return True
        
        return False

    def _emergency_stop(self):
        """Execute an emergency stop and pause navigation."""
        self.get_logger().warn('Emergency stop triggered - obstacle too close!')
        self._stop_robot()
        self.state = NavigationState.PAUSED
        self._publish_condition_triggered('emergency_stop', 0.0)

    def _execute_current_command(self) -> bool:
        """
        Execute the current navigation command.
        
        Returns:
            True if command is completed
        """
        cmd = self.current_command
        if cmd is None:
            return True
        
        action = cmd.action
        
        if action == 'stop':
            self._stop_robot()
            return True
        
        elif action == 'forward':
            return self._execute_forward(cmd)
        
        elif action == 'backward':
            return self._execute_backward(cmd)
        
        elif action == 'turn_left':
            return self._execute_turn_left(cmd)
        
        elif action == 'turn_right':
            return self._execute_turn_right(cmd)
        
        elif action == 'forward_until_close':
            return self._execute_forward_until_close(cmd)
        
        elif action == 'backward_until_close':
            return self._execute_backward_until_close(cmd)
        
        elif action == 'turn_until_clear':
            return self._execute_turn_until_clear(cmd)
        
        elif action == 'follow_wall':
            return self._execute_follow_wall(cmd)
        
        else:
            self.get_logger().error(f'Unknown action: {action}')
            return True

    def _get_elapsed_time(self) -> float:
        """
        Get elapsed time since last control loop iteration.
        
        Uses actual timestamps for accurate tracking instead of assuming
        a fixed control loop frequency.
        
        Returns:
            Elapsed time in seconds
        """
        current_time = self.get_clock().now()
        if self.last_control_time is None:
            self.last_control_time = current_time
            return 0.05  # Default to expected 20 Hz on first call
        
        elapsed = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time
        
        # Clamp to reasonable range to handle pauses or delays
        return min(max(elapsed, 0.001), 0.5)

    def _execute_forward(self, cmd: NavigationCommand) -> bool:
        """Execute forward movement command."""
        vel = Twist()
        vel.linear.x = self.linear_speed
        self._publish_velocity(vel)
        
        if cmd.distance is not None:
            dt = self._get_elapsed_time()
            self.distance_traveled += self.linear_speed * dt
            if self.distance_traveled >= cmd.distance:
                self._stop_robot()
                return True
        
        return False

    def _execute_backward(self, cmd: NavigationCommand) -> bool:
        """Execute backward movement command."""
        vel = Twist()
        vel.linear.x = -self.linear_speed
        self._publish_velocity(vel)
        
        if cmd.distance is not None:
            dt = self._get_elapsed_time()
            self.distance_traveled += self.linear_speed * dt
            if self.distance_traveled >= cmd.distance:
                self._stop_robot()
                return True
        
        return False

    def _execute_turn_left(self, cmd: NavigationCommand) -> bool:
        """Execute left turn command."""
        vel = Twist()
        vel.angular.z = self.angular_speed
        self._publish_velocity(vel)
        
        if cmd.angle is not None:
            angle_rad = math.radians(cmd.angle)
            dt = self._get_elapsed_time()
            self.angle_turned += self.angular_speed * dt
            if self.angle_turned >= angle_rad:
                self._stop_robot()
                return True
        
        return False

    def _execute_turn_right(self, cmd: NavigationCommand) -> bool:
        """Execute right turn command."""
        vel = Twist()
        vel.angular.z = -self.angular_speed
        self._publish_velocity(vel)
        
        if cmd.angle is not None:
            angle_rad = math.radians(cmd.angle)
            dt = self._get_elapsed_time()
            self.angle_turned += self.angular_speed * dt
            if self.angle_turned >= angle_rad:
                self._stop_robot()
                return True
        
        return False

    def _execute_forward_until_close(self, cmd: NavigationCommand) -> bool:
        """
        Execute forward movement until obstacle is close.
        
        This is the key behavior for reactive navigation based on object detection.
        """
        direction = cmd.condition_direction or Direction.FRONT.value
        threshold = cmd.condition_distance or self.default_obstacle_threshold
        
        # Check if condition is met
        current_dist = self.direction_distances.get(direction, float('inf'))
        
        if current_dist <= threshold:
            self.get_logger().info(
                f'Condition triggered: {direction} distance {current_dist:.2f}m <= {threshold}m'
            )
            self._stop_robot()
            self._publish_condition_triggered(direction, current_dist)
            
            # Queue next action if specified
            if cmd.next_action:
                next_cmds = self.parse_command(cmd.next_action)
                # Insert in reverse order to maintain execution sequence
                for nc in reversed(next_cmds):
                    self.command_queue.insert(0, nc)
            
            return True
        
        # Continue moving forward
        vel = Twist()
        vel.linear.x = self.linear_speed
        self._publish_velocity(vel)
        return False

    def _execute_backward_until_close(self, cmd: NavigationCommand) -> bool:
        """Execute backward movement until obstacle is close."""
        direction = cmd.condition_direction or Direction.BACK.value
        threshold = cmd.condition_distance or self.default_obstacle_threshold
        
        current_dist = self.direction_distances.get(direction, float('inf'))
        
        if current_dist <= threshold:
            self.get_logger().info(
                f'Condition triggered: {direction} distance {current_dist:.2f}m <= {threshold}m'
            )
            self._stop_robot()
            self._publish_condition_triggered(direction, current_dist)
            
            if cmd.next_action:
                next_cmds = self.parse_command(cmd.next_action)
                # Insert in reverse order to maintain execution sequence
                for nc in reversed(next_cmds):
                    self.command_queue.insert(0, nc)
            
            return True
        
        vel = Twist()
        vel.linear.x = -self.linear_speed
        self._publish_velocity(vel)
        return False

    def _execute_turn_until_clear(self, cmd: NavigationCommand) -> bool:
        """
        Execute turn until the front is clear of obstacles.
        
        Useful for finding a clear path after encountering an obstacle.
        """
        turn_direction = cmd.direction or 'left'
        clear_threshold = cmd.condition_distance or self.default_obstacle_threshold
        
        # Use inf as default when sensor data unavailable to avoid false positives
        front_dist = self.direction_distances.get(Direction.FRONT.value, float('inf'))
        
        # Only consider path clear if we have valid sensor data and it exceeds threshold
        if self.direction_distances and front_dist >= clear_threshold:
            self.get_logger().info(
                f'Clear path found: front distance {front_dist:.2f}m >= {clear_threshold}m'
            )
            self._stop_robot()
            self._publish_condition_triggered('clear_path', front_dist)
            return True
        
        vel = Twist()
        if turn_direction == 'left':
            vel.angular.z = self.angular_speed
        else:
            vel.angular.z = -self.angular_speed
        self._publish_velocity(vel)
        return False

    def _execute_follow_wall(self, cmd: NavigationCommand) -> bool:
        """
        Execute wall following behavior.
        
        Maintains a specified distance from a wall on the given side.
        """
        side = cmd.direction or 'left'
        desired_distance = cmd.distance or 1.0
        
        # Get distance to wall on specified side
        if side == 'left':
            wall_dist = self.direction_distances.get(Direction.LEFT.value, float('inf'))
        else:
            wall_dist = self.direction_distances.get(Direction.RIGHT.value, float('inf'))
        
        front_dist = self.direction_distances.get(Direction.FRONT.value, float('inf'))
        
        # Calculate error from desired distance
        error = wall_dist - desired_distance
        
        vel = Twist()
        
        # Check for obstacle in front
        if front_dist < self.default_obstacle_threshold:
            # Turn away from wall
            if side == 'left':
                vel.angular.z = -self.angular_speed
            else:
                vel.angular.z = self.angular_speed
            vel.linear.x = 0.0
        else:
            # Move forward while adjusting for wall distance
            vel.linear.x = self.linear_speed
            
            # P controller for angular velocity
            kp = 0.5
            if side == 'left':
                vel.angular.z = -kp * error  # Turn right if too close to left wall
            else:
                vel.angular.z = kp * error  # Turn left if too close to right wall
            
            # Limit angular velocity
            vel.angular.z = max(-self.angular_speed, min(self.angular_speed, vel.angular.z))
        
        self._publish_velocity(vel)
        
        # This command runs continuously until stopped or a distance limit is reached
        if cmd.condition_distance is not None:
            dt = self._get_elapsed_time()
            self.distance_traveled += self.linear_speed * dt
            if self.distance_traveled >= cmd.condition_distance:
                self._stop_robot()
                return True
        
        return False

    def _handle_command_completion(self):
        """Handle completion of the current command."""
        self.get_logger().info(f'Command completed: {self.current_command.action if self.current_command else "unknown"}')
        self.current_command = None
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        self.last_control_time = None  # Reset for next command
        
        if self.command_queue:
            self._start_next_command()
        else:
            self.state = NavigationState.COMPLETED
            self._publish_state_change('completed')

    def _start_next_command(self):
        """Start executing the next command in the queue."""
        if not self.command_queue:
            self.state = NavigationState.IDLE
            return
        
        self.current_command = self.command_queue.pop(0)
        self.state = NavigationState.EXECUTING
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        
        self.get_logger().info(f'Starting command: {self.current_command.action}')
        self._publish_state_change('executing')

    def _publish_velocity(self, vel: Twist):
        """Publish velocity command."""
        self.current_velocity = vel
        self.cmd_vel_pub.publish(vel)

    def _stop_robot(self):
        """Stop all robot movement."""
        vel = Twist()
        self._publish_velocity(vel)

    def _publish_condition_triggered(self, condition: str, value: float):
        """Publish a condition triggered event."""
        msg = String()
        msg.data = json.dumps({
            'condition': condition,
            'value': value,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.condition_triggered_pub.publish(msg)

    def _publish_state_change(self, new_state: str):
        """Publish a state change notification."""
        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)

    def _publish_error(self, error_msg: str):
        """Publish an error message."""
        msg = String()
        msg.data = json.dumps({
            'error': error_msg,
            'timestamp': self.get_clock().now().nanoseconds
        })
        self.status_pub.publish(msg)

    def publish_status(self):
        """Publish current navigation status."""
        status = {
            'state': self.state.value,
            'current_command': self.current_command.action if self.current_command else None,
            'queue_length': len(self.command_queue),
            'direction_distances': self.direction_distances,
            'distance_traveled': self.distance_traveled,
            'angle_turned': math.degrees(self.angle_turned)
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def stop_service_callback(self, request, response):
        """Service callback to stop navigation."""
        with self.state_lock:
            self._stop_robot()
            self.command_queue.clear()
            self.current_command = None
            self.state = NavigationState.IDLE
        
        response.success = True
        response.message = 'Navigation stopped'
        self.get_logger().info('Navigation stopped via service')
        return response

    def pause_service_callback(self, request, response):
        """Service callback to pause navigation."""
        with self.state_lock:
            if self.state == NavigationState.EXECUTING:
                self._stop_robot()
                self.state = NavigationState.PAUSED
                response.success = True
                response.message = 'Navigation paused'
            else:
                response.success = False
                response.message = f'Cannot pause from state: {self.state.value}'
        
        return response

    def resume_service_callback(self, request, response):
        """Service callback to resume navigation."""
        with self.state_lock:
            if self.state == NavigationState.PAUSED:
                self.state = NavigationState.EXECUTING
                response.success = True
                response.message = 'Navigation resumed'
            else:
                response.success = False
                response.message = f'Cannot resume from state: {self.state.value}'
        
        return response


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    
    node = ReactiveNavigationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
