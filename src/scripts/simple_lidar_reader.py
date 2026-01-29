#!/usr/bin/env python3

"""
Simple Lidar Data Reader Example

This example demonstrates how to subscribe to and process lidar scan data.
It can serve as a template for creating your own lidar-based applications.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class SimpleLidarReader(Node):
    """
    A simple example node that reads lidar data and prints basic statistics.
    """

    def __init__(self):
        super().__init__('simple_lidar_reader')
        
        # Create subscriber to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info('Simple Lidar Reader started')
        self.get_logger().info('Listening to /lidar/scan topic...')

    def scan_callback(self, msg):
        """
        Callback function for processing lidar scan data.
        
        Args:
            msg (LaserScan): The incoming laser scan message
        """
        # Basic scan information
        num_readings = len(msg.ranges)
        
        # Filter valid readings
        valid_ranges = [r for r in msg.ranges 
                       if msg.range_min <= r <= msg.range_max]
        
        if not valid_ranges:
            self.get_logger().info('No valid lidar readings')
            return
        
        # Calculate statistics
        min_distance = min(valid_ranges)
        max_distance = max(valid_ranges)
        avg_distance = sum(valid_ranges) / len(valid_ranges)
        
        # Find closest obstacle direction
        min_idx = msg.ranges.index(min_distance)
        closest_angle = msg.angle_min + (min_idx * msg.angle_increment)
        closest_angle_deg = math.degrees(closest_angle)
        
        # Print statistics
        self.get_logger().info(
            f'Scan stats: '
            f'Total points: {num_readings}, '
            f'Valid: {len(valid_ranges)}, '
            f'Min dist: {min_distance:.2f}m, '
            f'Max dist: {max_distance:.2f}m, '
            f'Avg dist: {avg_distance:.2f}m, '
            f'Closest at: {closest_angle_deg:.1f}°'
        )
        
        # Example: Detect obstacles in front (within 30 degrees)
        front_cone_angle = math.radians(30)
        obstacles_in_front = []
        
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                angle = msg.angle_min + (i * msg.angle_increment)
                if abs(angle) < front_cone_angle:
                    obstacles_in_front.append(r)
        
        if obstacles_in_front:
            closest_front = min(obstacles_in_front)
            if closest_front < 2.0:  # Within 2 meters
                self.get_logger().warn(
                    f'Obstacle detected in front at {closest_front:.2f}m!'
                )


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    
    node = SimpleLidarReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
