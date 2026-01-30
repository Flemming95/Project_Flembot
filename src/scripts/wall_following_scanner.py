#!/usr/bin/env python3

"""
Wall Following Scanner

This ROS2 node drives the robot along the 4 outer walls of the arena
while the lidar_gridmap_generator collects scan data to build a gridmap.

The robot follows a simple wall-following behavior to traverse the perimeter
of the arena, ensuring full coverage of the walls.

Usage:
    First, start the lidar gridmap generator:
    ros2 run gazebo_differential_drive_robot lidar_gridmap_generator.py
    
    Then start the wall follower:
    ros2 run gazebo_differential_drive_robot wall_following_scanner.py

Parameters:
    - wall_distance: Target distance from wall (default: 0.5m)
    - linear_speed: Forward speed (default: 0.3 m/s)
    - angular_speed: Turn speed (default: 0.5 rad/s)
    - laps: Number of laps around the arena (default: 1)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import math
import time
import json
from enum import Enum
from typing import Optional, Dict


class WallFollowerState(Enum):
    """States for the wall following state machine."""
    IDLE = "idle"
    FINDING_WALL = "finding_wall"
    FOLLOWING_WALL = "following_wall"
    TURNING_CORNER = "turning_corner"
    COMPLETED = "completed"


class WallFollowingScanner(Node):
    """
    A ROS2 node that drives the robot along walls for gridmap generation.
    
    Uses a simple wall-following algorithm to traverse the perimeter
    of an arena while the lidar gridmap generator collects data.
    """
    
    def __init__(self):
        super().__init__('wall_following_scanner')
        
        # Declare parameters
        self.declare_parameter('wall_distance', 0.5)  # meters
        self.declare_parameter('linear_speed', 0.3)  # m/s
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('laps', 1)  # number of laps
        self.declare_parameter('front_obstacle_threshold', 0.6)  # meters
        self.declare_parameter('side_cone_angle', 45.0)  # degrees
        self.declare_parameter('auto_start', True)  # Start automatically
        self.declare_parameter('auto_save', True)  # Save gridmap when done
        self.declare_parameter('wall_follow_kp', 1.5)  # Proportional gain for wall following
        
        # Get parameters
        self.wall_distance = self.get_parameter('wall_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.laps = self.get_parameter('laps').value
        self.front_threshold = self.get_parameter('front_obstacle_threshold').value
        self.side_cone_angle = math.radians(self.get_parameter('side_cone_angle').value)
        self.auto_start = self.get_parameter('auto_start').value
        self.auto_save = self.get_parameter('auto_save').value
        self.wall_follow_kp = self.get_parameter('wall_follow_kp').value
        
        # State machine
        self.state = WallFollowerState.IDLE
        self.corners_turned = 0
        self.current_lap = 0
        
        # Sensor data
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.front_right_distance = float('inf')
        self.left_distance = float('inf')
        
        # Robot pose for lap detection
        self.start_x = None
        self.start_y = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.lap_start_recorded = False
        
        # Create subscriber to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10
        )
        
        # Create subscriber to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create publisher for status
        self.status_pub = self.create_publisher(
            String,
            '/wall_follower/status',
            10
        )
        
        # Service clients for gridmap generator
        self.start_scanning_client = self.create_client(
            Trigger, '/gridmap/start_scanning'
        )
        self.stop_scanning_client = self.create_client(
            Trigger, '/gridmap/stop_scanning'
        )
        self.save_gridmap_client = self.create_client(
            Trigger, '/gridmap/save'
        )
        
        # Service to start/stop wall following
        self.start_srv = self.create_service(
            Trigger,
            '/wall_follower/start',
            self.start_callback
        )
        
        self.stop_srv = self.create_service(
            Trigger,
            '/wall_follower/stop',
            self.stop_callback
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Auto-start timer (will be set if auto_start is True)
        self.auto_start_timer = None
        
        self.get_logger().info('Wall Following Scanner initialized')
        self.get_logger().info(f'Wall distance: {self.wall_distance}m')
        self.get_logger().info(f'Laps to complete: {self.laps}')
        self.get_logger().info('Services: /wall_follower/start, /wall_follower/stop')
        
        if self.auto_start:
            # Give time for other nodes to start
            self.auto_start_timer = self.create_timer(2.0, self.auto_start_callback)
    
    def auto_start_callback(self):
        """Automatically start wall following after a delay."""
        if self.state == WallFollowerState.IDLE:
            self.get_logger().info('Auto-starting wall following...')
            self.start_wall_following()
        # Cancel the timer after first execution
        if self.auto_start_timer is not None:
            self.destroy_timer(self.auto_start_timer)
            self.auto_start_timer = None
    
    def odom_callback(self, msg: Odometry):
        """Callback for odometry to track robot position."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def scan_callback(self, msg: LaserScan):
        """
        Callback for lidar scan to update distance measurements.
        """
        # Calculate distances in different directions
        self.front_distance = self._get_min_distance_in_cone(msg, 0.0, math.radians(30))
        self.right_distance = self._get_min_distance_in_cone(msg, -math.pi/2, self.side_cone_angle)
        self.front_right_distance = self._get_min_distance_in_cone(msg, -math.pi/4, math.radians(30))
        self.left_distance = self._get_min_distance_in_cone(msg, math.pi/2, self.side_cone_angle)
    
    def _get_min_distance_in_cone(self, msg: LaserScan, center_angle: float, 
                                   half_cone: float) -> float:
        """
        Get minimum distance within a cone of angles.
        """
        min_dist = float('inf')
        angle = msg.angle_min
        
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max and math.isfinite(r):
                # Normalize angle to [-pi, pi]
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))
                
                # Check if within cone
                angle_diff = abs(math.atan2(math.sin(norm_angle - center_angle),
                                            math.cos(norm_angle - center_angle)))
                if angle_diff <= half_cone:
                    min_dist = min(min_dist, r)
            
            angle += msg.angle_increment
        
        return min_dist
    
    def control_loop(self):
        """Main control loop for wall following."""
        if self.state == WallFollowerState.IDLE:
            self.stop_robot()
            return
        
        if self.state == WallFollowerState.COMPLETED:
            self.stop_robot()
            return
        
        if self.state == WallFollowerState.FINDING_WALL:
            self.find_wall()
            return
        
        if self.state == WallFollowerState.FOLLOWING_WALL:
            self.follow_wall()
            return
        
        if self.state == WallFollowerState.TURNING_CORNER:
            self.turn_corner()
            return
    
    def find_wall(self):
        """State: Find a wall to start following."""
        vel = Twist()
        
        # Check if there's a wall on the right
        if self.right_distance < self.wall_distance * 2:
            # Found wall on right, start following
            self.state = WallFollowerState.FOLLOWING_WALL
            self.get_logger().info('Wall found, starting to follow')
            
            # Record start position for lap detection
            if not self.lap_start_recorded:
                self.start_x = self.robot_x
                self.start_y = self.robot_y
                self.lap_start_recorded = True
            return
        
        # No wall on right, check front
        if self.front_distance < self.front_threshold:
            # Wall in front, turn left
            vel.angular.z = self.angular_speed
        else:
            # Move forward to find wall
            vel.linear.x = self.linear_speed
        
        self.cmd_vel_pub.publish(vel)
    
    def follow_wall(self):
        """State: Follow wall on the right side."""
        vel = Twist()
        
        # Check for corner (wall in front)
        if self.front_distance < self.front_threshold:
            self.state = WallFollowerState.TURNING_CORNER
            self.corners_turned += 1
            self.get_logger().info(f'Corner detected, turning ({self.corners_turned} corners)')
            
            # Check for lap completion (4 corners = 1 lap)
            if self.corners_turned >= 4:
                self._check_lap_complete()
            return
        
        # Wall following control
        # Error: positive means too far from wall, negative means too close
        error = self.right_distance - self.wall_distance
        
        # Proportional control for angular velocity
        vel.angular.z = -self.wall_follow_kp * error
        
        # Limit angular velocity
        vel.angular.z = max(-self.angular_speed, min(self.angular_speed, vel.angular.z))
        
        # Reduce forward speed when correcting
        vel.linear.x = self.linear_speed * (1.0 - abs(vel.angular.z) / self.angular_speed * 0.5)
        
        # If lost wall completely, turn right to find it
        if self.right_distance > self.wall_distance * 3:
            vel.angular.z = -self.angular_speed * 0.5
            vel.linear.x = self.linear_speed * 0.5
        
        self.cmd_vel_pub.publish(vel)
    
    def turn_corner(self):
        """State: Turn left at a corner."""
        vel = Twist()
        
        # Turn left
        vel.angular.z = self.angular_speed
        vel.linear.x = 0.0
        
        # Check if turn is complete (no wall in front and wall on right)
        if self.front_distance > self.front_threshold * 1.2:
            # Wait a bit more if no wall on right yet
            if self.right_distance < self.wall_distance * 2:
                self.state = WallFollowerState.FOLLOWING_WALL
                self.get_logger().info('Corner turn complete, resuming wall following')
        
        self.cmd_vel_pub.publish(vel)
    
    def _check_lap_complete(self):
        """Check if a lap has been completed."""
        if self.start_x is None or self.start_y is None:
            return
        
        # Check if back near start position
        dist_to_start = math.sqrt(
            (self.robot_x - self.start_x)**2 + 
            (self.robot_y - self.start_y)**2
        )
        
        if dist_to_start < 1.0:  # Within 1 meter of start
            self.current_lap += 1
            self.corners_turned = 0  # Reset for next lap
            self.get_logger().info(f'Lap {self.current_lap} completed')
            
            if self.current_lap >= self.laps:
                self.complete_scanning()
    
    def complete_scanning(self):
        """Complete the scanning process."""
        self.state = WallFollowerState.COMPLETED
        self.stop_robot()
        self.get_logger().info('Wall following complete!')
        
        # Stop scanning
        if self.stop_scanning_client.wait_for_service(timeout_sec=1.0):
            future = self.stop_scanning_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        # Save gridmap if auto_save is enabled
        if self.auto_save:
            self.get_logger().info('Saving gridmap...')
            if self.save_gridmap_client.wait_for_service(timeout_sec=1.0):
                future = self.save_gridmap_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.result():
                    self.get_logger().info(f'Gridmap saved: {future.result().message}')
    
    def stop_robot(self):
        """Stop the robot."""
        vel = Twist()
        self.cmd_vel_pub.publish(vel)
    
    def start_wall_following(self):
        """Start the wall following process."""
        if self.state != WallFollowerState.IDLE and self.state != WallFollowerState.COMPLETED:
            return False
        
        # Reset state
        self.state = WallFollowerState.FINDING_WALL
        self.corners_turned = 0
        self.current_lap = 0
        self.lap_start_recorded = False
        self.start_x = None
        self.start_y = None
        
        # Start scanning
        if self.start_scanning_client.wait_for_service(timeout_sec=1.0):
            future = self.start_scanning_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result():
                self.get_logger().info(f'Scanning started: {future.result().message}')
        else:
            self.get_logger().warn('Gridmap generator not available, proceeding anyway')
        
        self.get_logger().info('Wall following started')
        return True
    
    def start_callback(self, request, response):
        """Service callback to start wall following."""
        if self.start_wall_following():
            response.success = True
            response.message = 'Wall following started'
        else:
            response.success = False
            response.message = 'Wall following already in progress'
        return response
    
    def stop_callback(self, request, response):
        """Service callback to stop wall following."""
        self.state = WallFollowerState.IDLE
        self.stop_robot()
        
        # Stop scanning
        if self.stop_scanning_client.wait_for_service(timeout_sec=1.0):
            self.stop_scanning_client.call_async(Trigger.Request())
        
        response.success = True
        response.message = 'Wall following stopped'
        return response
    
    def publish_status(self):
        """Publish current status."""
        status = {
            'state': self.state.value,
            'corners_turned': self.corners_turned,
            'current_lap': self.current_lap,
            'target_laps': self.laps,
            'distances': {
                'front': round(self.front_distance, 2),
                'right': round(self.right_distance, 2),
                'front_right': round(self.front_right_distance, 2),
                'left': round(self.left_distance, 2)
            },
            'robot_pose': {
                'x': round(self.robot_x, 2),
                'y': round(self.robot_y, 2),
                'yaw': round(math.degrees(self.robot_yaw), 1)
            }
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    
    node = WallFollowingScanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
