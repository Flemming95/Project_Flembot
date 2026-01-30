#!/usr/bin/env python3

"""
Scan to Image Map Generator

This ROS2 node subscribes to lidar scan data and generates image maps of the scans.
The images can be saved to files and published for real-time visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import numpy as np
import math
import os
from datetime import datetime


class ScanToImage(Node):
    """
    A ROS2 node that converts lidar scans to image maps.
    """

    def __init__(self):
        super().__init__('scan_to_image')
        
        # Declare parameters
        self.declare_parameter('image_size', 500)  # Image size in pixels
        self.declare_parameter('max_range', 10.0)  # Maximum range to display (meters)
        self.declare_parameter('save_images', False)  # Whether to save images to files
        self.declare_parameter('save_path', '/tmp/scan_images')  # Path to save images
        self.declare_parameter('save_interval', 1.0)  # Interval between saved images (seconds)
        
        # Get parameters
        self.image_size = self.get_parameter('image_size').value
        self.max_range = self.get_parameter('max_range').value
        self.save_images = self.get_parameter('save_images').value
        self.save_path = self.get_parameter('save_path').value
        self.save_interval = self.get_parameter('save_interval').value
        
        # Create save directory if needed
        if self.save_images:
            os.makedirs(self.save_path, exist_ok=True)
            self.get_logger().info(f'Saving images to: {self.save_path}')
        
        # Initialize CV bridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()
        
        # Timing for save interval
        self.last_save_time = self.get_clock().now()
        
        # Create subscriber to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10
        )
        
        # Create publisher for scan image
        self.image_pub = self.create_publisher(
            Image,
            '/scan_image',
            10
        )
        
        self.get_logger().info('Scan to Image node initialized')
        self.get_logger().info(f'Image size: {self.image_size}x{self.image_size} pixels')
        self.get_logger().info(f'Max range: {self.max_range}m')

    def scan_callback(self, msg):
        """
        Callback function for processing lidar scan data and generating images.
        
        Args:
            msg (LaserScan): The incoming laser scan message
        """
        # Create blank image (black background)
        image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        
        # Calculate center of image and scale factor
        center_x = self.image_size // 2
        center_y = self.image_size // 2
        scale = (self.image_size / 2) / self.max_range
        
        # Draw the robot position (blue circle)
        self._draw_circle(image, center_x, center_y, 5, (255, 128, 0))  # Blue (BGR)
        
        # Draw range circles for reference
        self._draw_range_circles(image, center_x, center_y, scale)
        
        # Convert polar coordinates to Cartesian and draw points
        angle = msg.angle_min
        scan_points = []
        
        for r in msg.ranges:
            if msg.range_min <= r <= min(msg.range_max, self.max_range):
                if math.isfinite(r):
                    # Convert polar to Cartesian
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    
                    # Convert to pixel coordinates (flip y for image coordinates)
                    px = int(center_x + x * scale)
                    py = int(center_y - y * scale)  # Flip y-axis
                    
                    # Check bounds
                    if 0 <= px < self.image_size and 0 <= py < self.image_size:
                        scan_points.append((px, py))
                        
                        # Color based on distance (green=close, yellow=medium, red=far)
                        color = self._get_distance_color(r)
                        self._draw_circle(image, px, py, 2, color)
            
            angle += msg.angle_increment
        
        # Add timestamp text
        self._add_timestamp(image)
        
        # Add scale text
        self._add_scale_info(image)
        
        # Publish the image
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            ros_image.header = msg.header
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
        
        # Save image if enabled and interval has passed
        if self.save_images:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.last_save_time).nanoseconds / 1e9
            if elapsed >= self.save_interval:
                self._save_image(image)
                self.last_save_time = current_time

    def _draw_circle(self, image, x, y, radius, color):
        """
        Draw a filled circle on the image.
        
        Args:
            image: The image array
            x, y: Center coordinates
            radius: Circle radius
            color: BGR color tuple
        """
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy <= radius * radius:
                    px, py = x + dx, y + dy
                    if 0 <= px < self.image_size and 0 <= py < self.image_size:
                        image[py, px] = color

    def _draw_range_circles(self, image, cx, cy, scale):
        """
        Draw concentric circles to show range reference.
        
        Args:
            image: The image array
            cx, cy: Center coordinates
            scale: Scale factor (pixels per meter)
        """
        # Draw circles at 1m, 2m, 5m, and max_range
        ranges = [1.0, 2.0, 5.0, self.max_range]
        color = (64, 64, 64)  # Dark gray
        
        for r in ranges:
            if r <= self.max_range:
                radius = int(r * scale)
                self._draw_ring(image, cx, cy, radius, color)

    def _draw_ring(self, image, cx, cy, radius, color):
        """
        Draw a ring (circle outline) on the image.
        
        Args:
            image: The image array
            cx, cy: Center coordinates
            radius: Ring radius
            color: BGR color tuple
        """
        for angle in range(360):
            rad = math.radians(angle)
            x = int(cx + radius * math.cos(rad))
            y = int(cy + radius * math.sin(rad))
            if 0 <= x < self.image_size and 0 <= y < self.image_size:
                image[y, x] = color

    def _get_distance_color(self, distance):
        """
        Get color based on distance (close=green, far=red).
        
        Args:
            distance: Distance in meters
            
        Returns:
            BGR color tuple
        """
        # Normalize distance to 0-1 range
        normalized = min(distance / self.max_range, 1.0)
        
        # Interpolate from green (close) to yellow to red (far)
        if normalized < 0.5:
            # Green to yellow
            g = 255
            r = int(normalized * 2 * 255)
            b = 0
        else:
            # Yellow to red
            g = int((1 - (normalized - 0.5) * 2) * 255)
            r = 255
            b = 0
        
        return (b, g, r)  # BGR format

    def _add_timestamp(self, image):
        """
        Add timestamp text to the image.
        
        Args:
            image: The image array
        """
        # Simple text rendering using basic shapes
        # This is a simplified version - could use OpenCV putText for better quality
        timestamp = datetime.now().strftime("%H:%M:%S")
        # For simplicity, we just add a small indicator in the corner
        # A production version would use cv2.putText
        pass

    def _add_scale_info(self, image):
        """
        Add scale information to the image.
        
        Args:
            image: The image array
        """
        # Add a simple scale bar at the bottom
        bar_length = int(self.image_size / 4)  # 1/4 of image width
        bar_y = self.image_size - 20
        bar_x_start = 20
        
        # Draw scale bar (white)
        for x in range(bar_x_start, bar_x_start + bar_length):
            if 0 <= x < self.image_size:
                image[bar_y, x] = (255, 255, 255)
                image[bar_y - 1, x] = (255, 255, 255)

    def _save_image(self, image):
        """
        Save the image to a file.
        
        Args:
            image: The image array to save
        """
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.save_path, f'scan_{timestamp}.png')
            
            # Use numpy to save as a simple format
            # For full PNG support, would need cv2.imwrite
            # Here we'll save as raw data that can be converted later
            np.save(filename.replace('.png', '.npy'), image)
            
            self.get_logger().debug(f'Saved scan image: {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    
    node = ScanToImage()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
