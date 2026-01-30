#!/usr/bin/env python3

"""
Real Gridmap Generator

This script generates "real" gridmaps from Gazebo SDF world files.
It parses the world files to extract wall and obstacle positions/geometries
and creates a binary occupancy gridmap representation.

The gridmaps are saved to the gridmaps/real/ folder.

Usage:
    python3 real_gridmap_generator.py [world_name]
    
    If no world_name is provided, it will generate gridmaps for all worlds.
"""

import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import math
from typing import List, Tuple, Optional, Dict
import json
from datetime import datetime


class RealGridmapGenerator:
    """
    Generates occupancy gridmaps from SDF world files.
    
    The gridmap represents the real world layout by extracting
    static obstacles (walls, boxes, cylinders) from the world definition.
    """
    
    def __init__(self, resolution: float = 0.05, 
                 world_size: Tuple[float, float] = (12.0, 10.0),
                 origin: Tuple[float, float] = (-6.0, -5.0)):
        """
        Initialize the gridmap generator.
        
        Args:
            resolution: Grid cell size in meters (default: 0.05m = 5cm)
            world_size: Size of the world (width, height) in meters
            origin: Origin point (x, y) of the gridmap (bottom-left corner)
        """
        self.resolution = resolution
        self.world_size = world_size
        self.origin = origin
        
        # Calculate grid dimensions
        self.width = int(world_size[0] / resolution)
        self.height = int(world_size[1] / resolution)
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.
        
        Args:
            x: X coordinate in world frame
            y: Y coordinate in world frame
            
        Returns:
            Tuple of (grid_x, grid_y) indices
        """
        grid_x = int((x - self.origin[0]) / self.resolution)
        grid_y = int((y - self.origin[1]) / self.resolution)
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
        x = grid_x * self.resolution + self.origin[0]
        y = grid_y * self.resolution + self.origin[1]
        return (x, y)
    
    def parse_pose(self, pose_str: str) -> Tuple[float, float, float, float, float, float]:
        """
        Parse a pose string from SDF format.
        
        Args:
            pose_str: Pose string in format "x y z roll pitch yaw"
            
        Returns:
            Tuple of (x, y, z, roll, pitch, yaw)
        """
        parts = pose_str.split()
        if len(parts) >= 6:
            return (float(parts[0]), float(parts[1]), float(parts[2]),
                    float(parts[3]), float(parts[4]), float(parts[5]))
        elif len(parts) >= 3:
            return (float(parts[0]), float(parts[1]), float(parts[2]), 0.0, 0.0, 0.0)
        else:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    def parse_size(self, size_str: str) -> Tuple[float, float, float]:
        """
        Parse a size string from SDF format.
        
        Args:
            size_str: Size string in format "x y z"
            
        Returns:
            Tuple of (x, y, z) dimensions
        """
        parts = size_str.split()
        if len(parts) >= 3:
            return (float(parts[0]), float(parts[1]), float(parts[2]))
        else:
            return (1.0, 1.0, 1.0)
    
    def fill_box(self, gridmap: np.ndarray, center_x: float, center_y: float,
                 size_x: float, size_y: float, yaw: float = 0.0) -> None:
        """
        Fill a rectangular area in the gridmap (rotated box).
        
        Args:
            gridmap: The gridmap array to modify
            center_x: X center of the box in world coordinates
            center_y: Y center of the box in world coordinates
            size_x: Width of the box
            size_y: Height of the box
            yaw: Rotation angle in radians
        """
        # Get bounding box for the rotated rectangle
        corners = []
        half_x = size_x / 2
        half_y = size_y / 2
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Calculate rotated corner points
        for dx, dy in [(-half_x, -half_y), (half_x, -half_y),
                       (half_x, half_y), (-half_x, half_y)]:
            rx = dx * cos_yaw - dy * sin_yaw + center_x
            ry = dx * sin_yaw + dy * cos_yaw + center_y
            corners.append((rx, ry))
        
        # Get bounding box
        min_x = min(c[0] for c in corners)
        max_x = max(c[0] for c in corners)
        min_y = min(c[1] for c in corners)
        max_y = max(c[1] for c in corners)
        
        # Iterate over bounding box and fill cells inside the rotated rectangle
        step = self.resolution / 2  # Sub-cell sampling for accuracy
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                # Check if point is inside the rotated rectangle
                # Transform point to box-local coordinates
                dx = x - center_x
                dy = y - center_y
                local_x = dx * cos_yaw + dy * sin_yaw
                local_y = -dx * sin_yaw + dy * cos_yaw
                
                if abs(local_x) <= half_x and abs(local_y) <= half_y:
                    gx, gy = self.world_to_grid(x, y)
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        gridmap[gy, gx] = 100  # Occupied
                
                y += step
            x += step
    
    def fill_cylinder(self, gridmap: np.ndarray, center_x: float, center_y: float,
                      radius: float) -> None:
        """
        Fill a circular area in the gridmap.
        
        Args:
            gridmap: The gridmap array to modify
            center_x: X center of the cylinder in world coordinates
            center_y: Y center of the cylinder in world coordinates
            radius: Radius of the cylinder
        """
        # Get bounding box
        min_x = center_x - radius
        max_x = center_x + radius
        min_y = center_y - radius
        max_y = center_y + radius
        
        # Iterate over bounding box and fill cells inside the circle
        step = self.resolution / 2
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                dist = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                if dist <= radius:
                    gx, gy = self.world_to_grid(x, y)
                    if 0 <= gx < self.width and 0 <= gy < self.height:
                        gridmap[gy, gx] = 100  # Occupied
                y += step
            x += step
    
    def parse_world_file(self, world_file: str) -> np.ndarray:
        """
        Parse an SDF world file and generate a gridmap.
        
        Args:
            world_file: Path to the SDF world file
            
        Returns:
            numpy array representing the occupancy gridmap
        """
        # Initialize gridmap (0 = free, 100 = occupied)
        gridmap = np.zeros((self.height, self.width), dtype=np.int8)
        
        try:
            tree = ET.parse(world_file)
            root = tree.getroot()
        except ET.ParseError as e:
            print(f"Error parsing {world_file}: {e}")
            return gridmap
        
        # Find all models in the world
        for world in root.findall('.//world'):
            for model in world.findall('model'):
                model_name = model.get('name', '')
                
                # Skip ground plane
                if 'ground' in model_name.lower():
                    continue
                
                # Get model pose
                pose_elem = model.find('pose')
                if pose_elem is not None and pose_elem.text:
                    model_pose = self.parse_pose(pose_elem.text)
                else:
                    model_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                
                # Check if static (walls and obstacles are usually static)
                static_elem = model.find('static')
                if static_elem is not None and static_elem.text.lower() == 'true':
                    # Process collision geometries
                    for link in model.findall('.//link'):
                        for collision in link.findall('collision'):
                            geometry = collision.find('geometry')
                            if geometry is not None:
                                # Box geometry
                                box = geometry.find('box')
                                if box is not None:
                                    size_elem = box.find('size')
                                    if size_elem is not None and size_elem.text:
                                        size = self.parse_size(size_elem.text)
                                        self.fill_box(gridmap,
                                                     model_pose[0], model_pose[1],
                                                     size[0], size[1],
                                                     model_pose[5])  # yaw
                                
                                # Cylinder geometry
                                cylinder = geometry.find('cylinder')
                                if cylinder is not None:
                                    radius_elem = cylinder.find('radius')
                                    if radius_elem is not None and radius_elem.text:
                                        radius = float(radius_elem.text)
                                        self.fill_cylinder(gridmap,
                                                          model_pose[0], model_pose[1],
                                                          radius)
        
        return gridmap
    
    def save_gridmap(self, gridmap: np.ndarray, world_name: str, 
                     output_dir: str) -> str:
        """
        Save the gridmap to a file.
        
        Args:
            gridmap: The gridmap array
            world_name: Name of the world
            output_dir: Directory to save the gridmap
            
        Returns:
            Path to the saved gridmap file
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # Save as numpy file
        filename = f"{world_name}_real.npy"
        filepath = os.path.join(output_dir, filename)
        np.save(filepath, gridmap)
        
        # Also save metadata as JSON
        metadata = {
            'world_name': world_name,
            'resolution': self.resolution,
            'width': self.width,
            'height': self.height,
            'origin': list(self.origin),
            'world_size': list(self.world_size),
            'timestamp': datetime.now().isoformat(),
            'type': 'real'
        }
        metadata_path = os.path.join(output_dir, f"{world_name}_real_metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        return filepath
    
    def visualize_gridmap(self, gridmap: np.ndarray, world_name: str,
                          output_dir: str) -> Optional[str]:
        """
        Save a visualization of the gridmap as a text-based representation.
        
        Args:
            gridmap: The gridmap array
            world_name: Name of the world
            output_dir: Directory to save the visualization
            
        Returns:
            Path to the visualization file or None if failed
        """
        # Create a simple text visualization
        vis_path = os.path.join(output_dir, f"{world_name}_real_preview.txt")
        
        # Downsample for text display
        scale = 4  # Show every 4th cell
        with open(vis_path, 'w') as f:
            f.write(f"Gridmap: {world_name}\n")
            f.write(f"Resolution: {self.resolution}m, Size: {self.width}x{self.height}\n")
            f.write(f"Origin: ({self.origin[0]}, {self.origin[1]})\n")
            f.write("-" * (self.width // scale + 2) + "\n")
            
            for y in range(self.height - 1, -1, -scale):  # Top to bottom
                row = "|"
                for x in range(0, self.width, scale):
                    if gridmap[y, x] > 50:
                        row += "#"
                    else:
                        row += " "
                row += "|"
                f.write(row + "\n")
            
            f.write("-" * (self.width // scale + 2) + "\n")
        
        return vis_path


def get_package_paths() -> Tuple[str, str]:
    """
    Get the paths to the worlds and gridmaps directories.
    
    Returns:
        Tuple of (worlds_dir, gridmaps_dir)
    """
    # When running from source, find relative paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)  # src/
    
    worlds_dir = os.path.join(package_dir, 'worlds')
    gridmaps_dir = os.path.join(package_dir, 'gridmaps', 'real')
    
    return worlds_dir, gridmaps_dir


def main():
    """
    Main function to generate real gridmaps from world files.
    """
    worlds_dir, output_dir = get_package_paths()
    
    # Check if worlds directory exists
    if not os.path.exists(worlds_dir):
        print(f"Error: Worlds directory not found: {worlds_dir}")
        sys.exit(1)
    
    # Create generator
    generator = RealGridmapGenerator()
    
    # Get list of world files to process
    if len(sys.argv) > 1:
        # Process specific world
        world_name = sys.argv[1]
        if not world_name.endswith('.sdf'):
            world_name = f"{world_name}.sdf"
        world_files = [os.path.join(worlds_dir, world_name)]
    else:
        # Process all world files
        world_files = [os.path.join(worlds_dir, f) 
                       for f in os.listdir(worlds_dir) 
                       if f.endswith('.sdf')]
    
    print(f"Real Gridmap Generator")
    print(f"======================")
    print(f"Resolution: {generator.resolution}m")
    print(f"Grid size: {generator.width}x{generator.height}")
    print(f"Output directory: {output_dir}")
    print()
    
    for world_file in sorted(world_files):
        if not os.path.exists(world_file):
            print(f"Warning: World file not found: {world_file}")
            continue
        
        world_name = os.path.splitext(os.path.basename(world_file))[0]
        print(f"Processing: {world_name}")
        
        # Generate gridmap
        gridmap = generator.parse_world_file(world_file)
        
        # Count occupied cells
        occupied = np.sum(gridmap > 50)
        total = gridmap.size
        print(f"  - Occupied cells: {occupied} / {total} ({100*occupied/total:.2f}%)")
        
        # Save gridmap
        filepath = generator.save_gridmap(gridmap, world_name, output_dir)
        print(f"  - Saved: {filepath}")
        
        # Save visualization
        vis_path = generator.visualize_gridmap(gridmap, world_name, output_dir)
        if vis_path:
            print(f"  - Preview: {vis_path}")
        
        print()
    
    print("Done!")


if __name__ == '__main__':
    main()
