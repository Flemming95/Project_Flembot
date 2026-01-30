#!/usr/bin/env python3

"""
Lidar Gridmap Generator

This ROS2 node generates gridmaps from lidar scan data while the robot
drives along the walls of an arena. It accumulates lidar readings and
builds an occupancy gridmap that can be compared to the "real" gridmap.

The gridmaps are saved to the gridmaps/created/ folder.

Usage:
    ros2 run gazebo_differential_drive_robot lidar_gridmap_generator.py

Parameters:
    - resolution: Grid cell size in meters (default: 0.05m)
    - world_size_x: World width in meters (default: 12.0m)
    - world_size_y: World height in meters (default: 10.0m)
    - origin_x: X origin of gridmap (default: -6.0m)
    - origin_y: Y origin of gridmap (default: -5.0m)
    - output_dir: Directory to save gridmaps
    - world_name: Name of the world being scanned
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import numpy as np
import math
import os
import json
from datetime import datetime
from typing import Optional, Tuple
import threading


class LidarGridmapGenerator(Node):
    """
    A ROS2 node that generates occupancy gridmaps from lidar scan data.
    
    The node subscribes to lidar scans and odometry to accumulate
    observations and build a gridmap. It provides services to save
    the gridmap and control the scanning process.
    """
    
    def __init__(self):
        super().__init__('lidar_gridmap_generator')
        
        # Declare parameters
        self.declare_parameter('resolution', 0.05)  # 5cm cells
        self.declare_parameter('world_size_x', 12.0)
        self.declare_parameter('world_size_y', 10.0)
        self.declare_parameter('origin_x', -6.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('output_dir', '')  # Empty = use default
        self.declare_parameter('world_name', 'unknown')
        self.declare_parameter('hit_threshold', 3)  # Hits needed to mark occupied
        self.declare_parameter('miss_threshold', 5)  # Misses needed to mark free
        
        # Get parameters
        self.resolution = self.get_parameter('resolution').value
        self.world_size_x = self.get_parameter('world_size_x').value
        self.world_size_y = self.get_parameter('world_size_y').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.output_dir = self.get_parameter('output_dir').value
        self.world_name = self.get_parameter('world_name').value
        self.hit_threshold = self.get_parameter('hit_threshold').value
        self.miss_threshold = self.get_parameter('miss_threshold').value
        
        # Calculate grid dimensions
        self.width = int(self.world_size_x / self.resolution)
        self.height = int(self.world_size_y / self.resolution)
        
        # Set default output directory if not specified
        if not self.output_dir:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            package_dir = os.path.dirname(script_dir)
            self.output_dir = os.path.join(package_dir, 'gridmaps', 'created')
        
        # Initialize gridmap data structures
        self._init_gridmap()
        
        # Robot pose tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pose_initialized = False
        
        # Scanning state
        self.is_scanning = False
        self.scan_count = 0
        # Lock for thread-safe gridmap updates when using multi-threaded executors
        # or when service calls modify state during scan callbacks
        self.lock = threading.Lock()
        
        # Create subscriber to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10
        )
        
        # Create subscriber to odometry for robot pose
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Create publisher for scanning status
        self.status_pub = self.create_publisher(
            String,
            '/gridmap/status',
            10
        )
        
        # Create publisher for gridmap complete notification
        self.complete_pub = self.create_publisher(
            Bool,
            '/gridmap/complete',
            10
        )
        
        # Create services
        self.start_srv = self.create_service(
            Trigger,
            '/gridmap/start_scanning',
            self.start_scanning_callback
        )
        
        self.stop_srv = self.create_service(
            Trigger,
            '/gridmap/stop_scanning',
            self.stop_scanning_callback
        )
        
        self.save_srv = self.create_service(
            Trigger,
            '/gridmap/save',
            self.save_gridmap_callback
        )
        
        self.reset_srv = self.create_service(
            Trigger,
            '/gridmap/reset',
            self.reset_gridmap_callback
        )
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Lidar Gridmap Generator initialized')
        self.get_logger().info(f'Grid size: {self.width}x{self.height}')
        self.get_logger().info(f'Resolution: {self.resolution}m')
        self.get_logger().info(f'World name: {self.world_name}')
        self.get_logger().info(f'Output directory: {self.output_dir}')
        self.get_logger().info('Services available:')
        self.get_logger().info('  /gridmap/start_scanning - Start collecting scan data')
        self.get_logger().info('  /gridmap/stop_scanning  - Stop collecting scan data')
        self.get_logger().info('  /gridmap/save           - Save the current gridmap')
        self.get_logger().info('  /gridmap/reset          - Reset the gridmap')
    
    def _init_gridmap(self):
        """Initialize or reset the gridmap data structures."""
        # Hit and miss counters for each cell
        self.hit_counts = np.zeros((self.height, self.width), dtype=np.int32)
        self.miss_counts = np.zeros((self.height, self.width), dtype=np.int32)
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            x: X coordinate in world frame
            y: Y coordinate in world frame
            
        Returns:
            Tuple of (grid_x, grid_y) indices
        """
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates.
        
        Args:
            grid_x: X index in grid
            grid_y: Y index in grid
            
        Returns:
            Tuple of (x, y) world coordinates
        """
        x = grid_x * self.resolution + self.origin_x
        y = grid_y * self.resolution + self.origin_y
        return (x, y)
    
    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry messages to track robot pose.
        
        Args:
            msg: Odometry message
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.pose_initialized = True
    
    def scan_callback(self, msg: LaserScan):
        """
        Callback for lidar scan messages.
        
        Processes the scan data and updates the gridmap.
        
        Args:
            msg: LaserScan message
        """
        if not self.is_scanning or not self.pose_initialized:
            return
        
        with self.lock:
            self._process_scan(msg)
            self.scan_count += 1
    
    def _process_scan(self, msg: LaserScan):
        """
        Process a lidar scan and update hit/miss counts.
        
        Uses raycasting to mark cells along each ray as free (miss)
        and the endpoint as occupied (hit) if the ray hits an obstacle.
        
        Args:
            msg: LaserScan message
        """
        angle = msg.angle_min
        
        for r in msg.ranges:
            # Skip invalid readings
            if not (msg.range_min <= r <= msg.range_max) or not math.isfinite(r):
                angle += msg.angle_increment
                continue
            
            # Calculate ray endpoint in world coordinates
            ray_angle = self.robot_yaw + angle
            end_x = self.robot_x + r * math.cos(ray_angle)
            end_y = self.robot_y + r * math.sin(ray_angle)
            
            # Raycast from robot to endpoint
            self._raycast(self.robot_x, self.robot_y, end_x, end_y, 
                          hit=(r < msg.range_max - 0.1))
            
            angle += msg.angle_increment
    
    def _raycast(self, start_x: float, start_y: float, 
                 end_x: float, end_y: float, hit: bool):
        """
        Perform raycasting from start to end, updating the gridmap.
        
        Uses Bresenham's line algorithm for efficient grid traversal.
        
        Args:
            start_x: Ray start X in world coordinates
            start_y: Ray start Y in world coordinates
            end_x: Ray end X in world coordinates
            end_y: Ray end Y in world coordinates
            hit: Whether the ray hit an obstacle
        """
        # Convert to grid coordinates
        start_gx, start_gy = self.world_to_grid(start_x, start_y)
        end_gx, end_gy = self.world_to_grid(end_x, end_y)
        
        # Bresenham's line algorithm
        dx = abs(end_gx - start_gx)
        dy = abs(end_gy - start_gy)
        sx = 1 if start_gx < end_gx else -1
        sy = 1 if start_gy < end_gy else -1
        err = dx - dy
        
        x, y = start_gx, start_gy
        
        while True:
            # Mark cell as free (if in bounds and not at endpoint)
            if 0 <= x < self.width and 0 <= y < self.height:
                if x == end_gx and y == end_gy:
                    # Endpoint - mark as hit if obstacle detected
                    if hit:
                        self.hit_counts[y, x] += 1
                else:
                    # Along the ray - mark as miss (free space)
                    self.miss_counts[y, x] += 1
            
            if x == end_gx and y == end_gy:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def get_occupancy_gridmap(self) -> np.ndarray:
        """
        Generate the occupancy gridmap from accumulated hit/miss data.
        
        Returns:
            numpy array with occupancy values:
            - 0: Free
            - 100: Occupied
            - -1: Unknown
        """
        gridmap = np.full((self.height, self.width), -1, dtype=np.int8)
        
        # Mark cells based on hit/miss counts
        for y in range(self.height):
            for x in range(self.width):
                hits = self.hit_counts[y, x]
                misses = self.miss_counts[y, x]
                
                if hits >= self.hit_threshold:
                    gridmap[y, x] = 100  # Occupied
                elif misses >= self.miss_threshold:
                    gridmap[y, x] = 0  # Free
                # Else remains -1 (unknown)
        
        return gridmap
    
    def publish_status(self):
        """Publish current scanning status."""
        gridmap = self.get_occupancy_gridmap()
        occupied = np.sum(gridmap == 100)
        free = np.sum(gridmap == 0)
        unknown = np.sum(gridmap == -1)
        
        status = {
            'is_scanning': self.is_scanning,
            'scan_count': self.scan_count,
            'robot_pose': {
                'x': self.robot_x,
                'y': self.robot_y,
                'yaw': self.robot_yaw
            },
            'gridmap': {
                'occupied': int(occupied),
                'free': int(free),
                'unknown': int(unknown),
                'total': self.width * self.height
            }
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def start_scanning_callback(self, request, response):
        """Service callback to start scanning."""
        self.is_scanning = True
        self.get_logger().info('Started scanning')
        response.success = True
        response.message = 'Scanning started'
        return response
    
    def stop_scanning_callback(self, request, response):
        """Service callback to stop scanning."""
        self.is_scanning = False
        self.get_logger().info('Stopped scanning')
        response.success = True
        response.message = f'Scanning stopped after {self.scan_count} scans'
        return response
    
    def reset_gridmap_callback(self, request, response):
        """Service callback to reset the gridmap."""
        with self.lock:
            self._init_gridmap()
            self.scan_count = 0
        self.get_logger().info('Gridmap reset')
        response.success = True
        response.message = 'Gridmap reset'
        return response
    
    def save_gridmap_callback(self, request, response):
        """Service callback to save the gridmap."""
        try:
            filepath = self.save_gridmap()
            response.success = True
            response.message = f'Gridmap saved to {filepath}'
            self.get_logger().info(response.message)
            
            # Publish completion notification
            complete_msg = Bool()
            complete_msg.data = True
            self.complete_pub.publish(complete_msg)
        except Exception as e:
            response.success = False
            response.message = f'Error saving gridmap: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def save_gridmap(self) -> str:
        """
        Save the current gridmap to a file.
        
        Returns:
            Path to the saved gridmap file
        """
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Generate gridmap
        gridmap = self.get_occupancy_gridmap()
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.world_name}_created_{timestamp}.npy"
        filepath = os.path.join(self.output_dir, filename)
        
        # Save gridmap
        np.save(filepath, gridmap)
        
        # Save metadata
        metadata = {
            'world_name': self.world_name,
            'resolution': self.resolution,
            'width': self.width,
            'height': self.height,
            'origin': [self.origin_x, self.origin_y],
            'world_size': [self.world_size_x, self.world_size_y],
            'timestamp': datetime.now().isoformat(),
            'type': 'created',
            'scan_count': self.scan_count,
            'hit_threshold': self.hit_threshold,
            'miss_threshold': self.miss_threshold
        }
        metadata_path = filepath.replace('.npy', '_metadata.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        # Save preview
        self._save_preview(gridmap, filepath.replace('.npy', '_preview.txt'))
        
        return filepath
    
    def _save_preview(self, gridmap: np.ndarray, filepath: str):
        """
        Save a text preview of the gridmap.
        
        Args:
            gridmap: The gridmap array
            filepath: Path to save the preview
        """
        scale = 4  # Show every 4th cell
        with open(filepath, 'w') as f:
            f.write(f"Gridmap: {self.world_name} (Created from Lidar)\n")
            f.write(f"Resolution: {self.resolution}m, Size: {self.width}x{self.height}\n")
            f.write(f"Scans processed: {self.scan_count}\n")
            f.write("-" * (self.width // scale + 2) + "\n")
            
            for y in range(self.height - 1, -1, -scale):
                row = "|"
                for x in range(0, self.width, scale):
                    if gridmap[y, x] == 100:
                        row += "#"
                    elif gridmap[y, x] == 0:
                        row += " "
                    else:
                        row += "."
                row += "|"
                f.write(row + "\n")
            
            f.write("-" * (self.width // scale + 2) + "\n")


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    
    node = LidarGridmapGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
