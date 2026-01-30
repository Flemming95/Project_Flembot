#!/usr/bin/env python3

"""
Lidar Navigation Support Node

This ROS2 node provides navigation support based on lidar object detection.
It processes detected objects and provides information useful for navigation,
such as nearest obstacle direction and safe navigation corridors.

This node is designed to work with a separate velocity command publisher,
providing the detection data and navigation recommendations that can be used
to generate appropriate velocity commands.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import math
import json


class LidarNavigationSupport(Node):
    """
    A ROS2 node that provides navigation support based on lidar data.
    
    This node:
    - Analyzes lidar scans to detect obstacles
    - Publishes navigation-relevant data for velocity command generation
    - Provides zone-based obstacle detection (front, left, right, back)
    - Publishes recommended safe directions for navigation
    """

    def __init__(self):
        super().__init__('lidar_navigation_support')
        
        # Declare parameters
        self.declare_parameter('obstacle_threshold', 1.0)  # meters
        self.declare_parameter('warning_threshold', 2.0)  # meters
        self.declare_parameter('num_sectors', 8)  # Number of angular sectors
        self.declare_parameter('min_valid_range', 0.1)  # meters
        self.declare_parameter('max_valid_range', 10.0)  # meters
        
        # Get parameters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.warning_threshold = self.get_parameter('warning_threshold').value
        self.num_sectors = self.get_parameter('num_sectors').value
        self.min_valid_range = self.get_parameter('min_valid_range').value
        self.max_valid_range = self.get_parameter('max_valid_range').value
        
        # Calculate sector size
        self.sector_size = 2 * math.pi / self.num_sectors
        
        # Create subscriber to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10
        )
        
        # Publishers for navigation data
        
        # Sector distances (min distance in each sector)
        self.sector_distances_pub = self.create_publisher(
            Float32MultiArray,
            '/navigation/sector_distances',
            10
        )
        
        # Obstacle zones (front, left, right, back status)
        self.obstacle_zones_pub = self.create_publisher(
            String,
            '/navigation/obstacle_zones',
            10
        )
        
        # Safe direction recommendation
        self.safe_direction_pub = self.create_publisher(
            Float32MultiArray,
            '/navigation/safe_direction',
            10
        )
        
        # Navigation markers for visualization
        self.nav_markers_pub = self.create_publisher(
            MarkerArray,
            '/navigation/markers',
            10
        )
        
        # Recommended velocity (optional, for reference)
        self.recommended_vel_pub = self.create_publisher(
            Twist,
            '/navigation/recommended_velocity',
            10
        )
        
        self.get_logger().info('Lidar Navigation Support node initialized')
        self.get_logger().info(f'Obstacle threshold: {self.obstacle_threshold}m')
        self.get_logger().info(f'Warning threshold: {self.warning_threshold}m')
        self.get_logger().info(f'Number of sectors: {self.num_sectors}')

    def scan_callback(self, msg):
        """
        Callback function for processing lidar scan data.
        
        Args:
            msg (LaserScan): The incoming laser scan message
        """
        # Calculate sector distances
        sector_distances = self._calculate_sector_distances(msg)
        
        # Determine obstacle zones
        obstacle_zones = self._determine_obstacle_zones(sector_distances)
        
        # Calculate safe direction
        safe_direction = self._calculate_safe_direction(sector_distances)
        
        # Generate recommended velocity (conservative, for reference)
        recommended_vel = self._generate_recommended_velocity(sector_distances, obstacle_zones)
        
        # Publish navigation data
        self._publish_sector_distances(sector_distances)
        self._publish_obstacle_zones(obstacle_zones)
        self._publish_safe_direction(safe_direction)
        self._publish_recommended_velocity(recommended_vel)
        self._publish_navigation_markers(sector_distances, obstacle_zones, msg.header)

    def _calculate_sector_distances(self, msg):
        """
        Calculate minimum distance in each angular sector.
        
        Args:
            msg (LaserScan): The laser scan message
            
        Returns:
            list: Minimum distance for each sector
        """
        sector_distances = [float('inf')] * self.num_sectors
        
        angle = msg.angle_min
        for r in msg.ranges:
            if self.min_valid_range <= r <= self.max_valid_range and math.isfinite(r):
                # Normalize angle to [0, 2*pi)
                normalized_angle = angle % (2 * math.pi)
                if normalized_angle < 0:
                    normalized_angle += 2 * math.pi
                
                # Determine sector
                sector = int(normalized_angle / self.sector_size) % self.num_sectors
                
                # Update minimum distance
                sector_distances[sector] = min(sector_distances[sector], r)
            
            angle += msg.angle_increment
        
        # Replace inf with max_valid_range for sectors with no readings
        sector_distances = [
            d if d != float('inf') else self.max_valid_range 
            for d in sector_distances
        ]
        
        return sector_distances

    def _determine_obstacle_zones(self, sector_distances):
        """
        Determine obstacle status for major navigation zones.
        
        Args:
            sector_distances: List of minimum distances per sector
            
        Returns:
            dict: Obstacle status for each zone
        """
        # Define zones based on 8 sectors
        # Sector 0: Front, 1: Front-Right, 2: Right, 3: Back-Right
        # Sector 4: Back, 5: Back-Left, 6: Left, 7: Front-Left
        
        zones = {
            'front': {'clear': True, 'warning': False, 'distance': float('inf')},
            'front_right': {'clear': True, 'warning': False, 'distance': float('inf')},
            'right': {'clear': True, 'warning': False, 'distance': float('inf')},
            'back_right': {'clear': True, 'warning': False, 'distance': float('inf')},
            'back': {'clear': True, 'warning': False, 'distance': float('inf')},
            'back_left': {'clear': True, 'warning': False, 'distance': float('inf')},
            'left': {'clear': True, 'warning': False, 'distance': float('inf')},
            'front_left': {'clear': True, 'warning': False, 'distance': float('inf')},
        }
        
        zone_mapping = [
            'front', 'front_right', 'right', 'back_right',
            'back', 'back_left', 'left', 'front_left'
        ]
        
        for i, zone_name in enumerate(zone_mapping):
            if i < len(sector_distances):
                dist = sector_distances[i]
                zones[zone_name]['distance'] = dist
                
                if dist < self.obstacle_threshold:
                    zones[zone_name]['clear'] = False
                    zones[zone_name]['warning'] = True
                elif dist < self.warning_threshold:
                    zones[zone_name]['warning'] = True
        
        return zones

    def _calculate_safe_direction(self, sector_distances):
        """
        Calculate the safest direction for navigation.
        
        Args:
            sector_distances: List of minimum distances per sector
            
        Returns:
            tuple: (angle in radians, confidence score 0-1)
        """
        if not sector_distances:
            return (0.0, 0.0)
        
        # Find sector with maximum distance
        max_dist = max(sector_distances)
        max_sector = sector_distances.index(max_dist)
        
        # Calculate angle to center of safest sector
        safe_angle = max_sector * self.sector_size + self.sector_size / 2
        
        # Normalize angle to [-pi, pi]
        if safe_angle > math.pi:
            safe_angle -= 2 * math.pi
        
        # Calculate confidence based on how much better the safest sector is
        avg_dist = sum(sector_distances) / len(sector_distances)
        if avg_dist > 0:
            confidence = min((max_dist - avg_dist) / avg_dist + 0.5, 1.0)
        else:
            confidence = 0.5
        
        return (safe_angle, max(0.0, confidence))

    def _generate_recommended_velocity(self, sector_distances, obstacle_zones):
        """
        Generate a conservative recommended velocity based on obstacle detection.
        
        This is just a reference - actual velocity commands should come from
        a separate navigation controller that may use more sophisticated logic.
        
        Args:
            sector_distances: List of minimum distances per sector
            obstacle_zones: Obstacle zone status
            
        Returns:
            Twist: Recommended velocity message
        """
        vel = Twist()
        
        # Get front zone status
        front_dist = obstacle_zones['front']['distance']
        front_left_dist = obstacle_zones['front_left']['distance']
        front_right_dist = obstacle_zones['front_right']['distance']
        
        # Minimum front distance
        min_front = min(front_dist, front_left_dist, front_right_dist)
        
        # Linear velocity based on front clearance
        if min_front < self.obstacle_threshold:
            vel.linear.x = 0.0  # Stop
        elif min_front < self.warning_threshold:
            # Slow down proportionally
            vel.linear.x = 0.2 * (min_front - self.obstacle_threshold) / (
                self.warning_threshold - self.obstacle_threshold
            )
        else:
            vel.linear.x = 0.2  # Normal speed
        
        # Angular velocity to avoid obstacles
        if not obstacle_zones['front']['clear']:
            # Turn towards the clearer side
            if front_left_dist > front_right_dist:
                vel.angular.z = 0.3  # Turn left
            else:
                vel.angular.z = -0.3  # Turn right
        
        return vel

    def _publish_sector_distances(self, sector_distances):
        """
        Publish sector distance data.
        
        Args:
            sector_distances: List of minimum distances per sector
        """
        msg = Float32MultiArray()
        msg.data = [float(d) for d in sector_distances]
        self.sector_distances_pub.publish(msg)

    def _publish_obstacle_zones(self, obstacle_zones):
        """
        Publish obstacle zone status as JSON.
        
        Args:
            obstacle_zones: Obstacle zone status dictionary
        """
        msg = String()
        msg.data = json.dumps(obstacle_zones)
        self.obstacle_zones_pub.publish(msg)

    def _publish_safe_direction(self, safe_direction):
        """
        Publish safe direction recommendation.
        
        Args:
            safe_direction: Tuple of (angle, confidence)
        """
        msg = Float32MultiArray()
        msg.data = [float(safe_direction[0]), float(safe_direction[1])]
        self.safe_direction_pub.publish(msg)

    def _publish_recommended_velocity(self, vel):
        """
        Publish recommended velocity.
        
        Args:
            vel: Twist message with recommended velocity
        """
        self.recommended_vel_pub.publish(vel)

    def _publish_navigation_markers(self, sector_distances, obstacle_zones, header):
        """
        Publish visualization markers for navigation data.
        
        Args:
            sector_distances: List of minimum distances per sector
            obstacle_zones: Obstacle zone status
            header: Message header for timestamps
        """
        marker_array = MarkerArray()
        
        # Create sector distance markers
        for i, dist in enumerate(sector_distances):
            angle = i * self.sector_size + self.sector_size / 2
            
            # Line from robot to sector edge
            marker = Marker()
            marker.header = header
            marker.ns = 'sector_lines'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            marker.scale.x = 0.02  # Line width
            
            # Color based on distance
            if dist < self.obstacle_threshold:
                marker.color.r = 1.0
                marker.color.g = 0.0
            elif dist < self.warning_threshold:
                marker.color.r = 1.0
                marker.color.g = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            # Line points
            from geometry_msgs.msg import Point
            p1 = Point()
            p1.x = 0.0
            p1.y = 0.0
            p1.z = 0.1
            
            p2 = Point()
            p2.x = min(dist, self.max_valid_range) * math.cos(angle)
            p2.y = min(dist, self.max_valid_range) * math.sin(angle)
            p2.z = 0.1
            
            marker.points.append(p1)
            marker.points.append(p2)
            
            marker.lifetime = Duration(sec=0, nanosec=200000000)
            marker_array.markers.append(marker)
        
        self.nav_markers_pub.publish(marker_array)


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    
    node = LidarNavigationSupport()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
