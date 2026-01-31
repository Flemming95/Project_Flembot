#!/usr/bin/env python3

"""
Lidar Object Detection Node

This ROS2 node subscribes to lidar scan data and performs simple object detection
by clustering nearby points. Detected objects are published as visualization markers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import math


class LidarObjectDetector(Node):
    """
    A ROS2 node that detects objects from lidar scan data using clustering.
    """

    def __init__(self):
        super().__init__('lidar_object_detector')
        
        # Declare parameters
        self.declare_parameter('distance_threshold', 0.2)  # meters
        self.declare_parameter('min_cluster_size', 5)  # minimum points per cluster
        self.declare_parameter('max_cluster_size', 100)  # maximum points per cluster
        
        # Get parameters
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        
        # Create subscriber to lidar scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.scan_callback,
            10
        )
        
        # Create publisher for detected objects
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/detected_objects',
            10
        )
        
        self.get_logger().info('Lidar Object Detector node initialized')
        self.get_logger().info(f'Distance threshold: {self.distance_threshold}m')
        self.get_logger().info(f'Min cluster size: {self.min_cluster_size} points')
        self.get_logger().info(f'Max cluster size: {self.max_cluster_size} points')

    def scan_callback(self, msg):
        """
        Callback function for processing lidar scan data.
        
        Args:
            msg (LaserScan): The incoming laser scan message
        """
        # Convert polar coordinates to Cartesian and filter in one pass
        points = []
        angle = msg.angle_min
        
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if math.isfinite(x) and math.isfinite(y):
                    points.append((x, y))
            angle += msg.angle_increment
        
        if len(points) < self.min_cluster_size:
            # Not enough points to detect objects
            self.publish_empty_markers()
            return
        
        # Cluster points to detect objects
        clusters = self.cluster_points(points)
        
        # Publish detected objects as markers
        self.publish_markers(clusters, msg.header)
        
        self.get_logger().debug(f'Detected {len(clusters)} objects')

    def cluster_points(self, points):
        """
        Cluster points based on distance threshold using a simple nearest-neighbor approach.
        
        Args:
            points (list): List of (x, y) tuples
            
        Returns:
            list: List of clusters, where each cluster is a list of (x, y) tuples
        """
        if not points:
            return []
        
        clusters = []
        visited = [False] * len(points)
        
        for i in range(len(points)):
            if visited[i]:
                continue
                
            # Start a new cluster
            cluster = [points[i]]
            visited[i] = True
            
            # Find all points within distance threshold
            j = 0
            while j < len(cluster):
                current_point = cluster[j]
                
                for k in range(len(points)):
                    if visited[k]:
                        continue
                    
                    dist = self.euclidean_distance(current_point, points[k])
                    if dist < self.distance_threshold:
                        cluster.append(points[k])
                        visited[k] = True
                        
                        # Limit cluster size
                        if len(cluster) >= self.max_cluster_size:
                            break
                
                j += 1
                
                if len(cluster) >= self.max_cluster_size:
                    break
            
            # Only keep clusters with minimum size
            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)
        
        return clusters

    def euclidean_distance(self, p1, p2):
        """
        Calculate Euclidean distance between two points.
        
        Args:
            p1 (tuple): First point (x, y)
            p2 (tuple): Second point (x, y)
            
        Returns:
            float: Distance between points
        """
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def calculate_cluster_center(self, cluster):
        """
        Calculate the centroid of a cluster.
        
        Args:
            cluster (list): List of (x, y) tuples
            
        Returns:
            tuple: Centroid (x, y) of the cluster
        """
        x_sum = sum(p[0] for p in cluster)
        y_sum = sum(p[1] for p in cluster)
        return (x_sum / len(cluster), y_sum / len(cluster))

    def publish_markers(self, clusters, header):
        """
        Publish detected objects as visualization markers.
        
        Args:
            clusters (list): List of detected clusters
            header: Header from the original scan message
        """
        marker_array = MarkerArray()
        
        for i, cluster in enumerate(clusters):
            # Calculate cluster properties
            center = self.calculate_cluster_center(cluster)
            
            # Create a sphere marker for the object center
            marker = Marker()
            marker.header = header
            marker.ns = 'detected_objects'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # Size based on cluster size
            size = 0.1 + (len(cluster) / 100.0)
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            
            # Color (green for detected objects)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime = Duration(sec=1, nanosec=0)
            
            marker_array.markers.append(marker)
            
            # Create a text marker showing the number of points
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = 'object_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = center[0]
            text_marker.pose.position.y = center[1]
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0
            
            text_marker.scale.z = 0.1
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f'Obj {i}\n{len(cluster)} pts'
            text_marker.lifetime = Duration(sec=1, nanosec=0)
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)

    def publish_empty_markers(self):
        """
        Publish an empty marker array to clear previous markers.
        """
        marker_array = MarkerArray()
        self.marker_pub.publish(marker_array)


def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    
    node = LidarObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
