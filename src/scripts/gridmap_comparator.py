#!/usr/bin/env python3

"""
Gridmap Comparison Tool

This script compares "real" gridmaps (generated from world files) with
"created" gridmaps (generated from lidar scans) to measure how well
the lidar mapping matches the actual world.

Usage:
    python3 gridmap_comparator.py <real_gridmap.npy> <created_gridmap.npy>
    
    Or to compare all gridmaps for a specific world:
    python3 gridmap_comparator.py --world <world_name>
"""

import os
import sys
import numpy as np
import json
from typing import Dict, Optional, Tuple, List
from datetime import datetime
import argparse


class GridmapComparator:
    """
    Compares real and created gridmaps to measure mapping accuracy.
    """
    
    def load_gridmap(self, filepath: str) -> Tuple[np.ndarray, Optional[Dict]]:
        """
        Load a gridmap and its metadata.
        
        Args:
            filepath: Path to the .npy gridmap file
            
        Returns:
            Tuple of (gridmap array, metadata dict or None)
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Gridmap file not found: {filepath}")
        
        gridmap = np.load(filepath)
        
        # Try to load metadata
        metadata_path = filepath.replace('.npy', '_metadata.json')
        metadata = None
        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
        
        return gridmap, metadata
    
    def compare(self, real_gridmap: np.ndarray, created_gridmap: np.ndarray,
                ignore_unknown: bool = True) -> Dict:
        """
        Compare two gridmaps and compute matching metrics.
        
        Args:
            real_gridmap: The ground truth gridmap (0=free, 100=occupied)
            created_gridmap: The lidar-created gridmap (0=free, 100=occupied, -1=unknown)
            ignore_unknown: If True, ignore cells marked as unknown in created map
            
        Returns:
            Dictionary containing comparison metrics
        """
        # Ensure same dimensions
        if real_gridmap.shape != created_gridmap.shape:
            raise ValueError(
                f"Gridmap dimensions don't match: {real_gridmap.shape} vs {created_gridmap.shape}"
            )
        
        height, width = real_gridmap.shape
        total_cells = width * height
        
        # Binary thresholds
        real_occupied = real_gridmap > 50
        real_free = real_gridmap <= 50
        
        created_occupied = created_gridmap == 100
        created_free = created_gridmap == 0
        created_unknown = created_gridmap == -1
        
        # Count cells in each category
        real_occupied_count = np.sum(real_occupied)
        real_free_count = np.sum(real_free)
        
        created_occupied_count = np.sum(created_occupied)
        created_free_count = np.sum(created_free)
        created_unknown_count = np.sum(created_unknown)
        
        # Calculate matching metrics
        # True Positives: Both mark as occupied
        true_positives = np.sum(real_occupied & created_occupied)
        
        # True Negatives: Both mark as free
        true_negatives = np.sum(real_free & created_free)
        
        # False Positives: Created says occupied, real says free
        false_positives = np.sum(real_free & created_occupied)
        
        # False Negatives: Created says free, real says occupied
        false_negatives = np.sum(real_occupied & created_free)
        
        # Cells that are known in created map
        known_mask = ~created_unknown
        known_cells = np.sum(known_mask)
        
        # Calculate metrics
        if ignore_unknown:
            # Only consider cells that are known in the created map
            correct = true_positives + true_negatives
            incorrect = false_positives + false_negatives
            
            # Calculate metrics only for known cells
            real_occ_known = np.sum(real_occupied & known_mask)
            real_free_known = np.sum(real_free & known_mask)
            
            accuracy = correct / known_cells if known_cells > 0 else 0.0
            
            # Precision: Of cells marked occupied, how many are correct?
            precision = true_positives / (true_positives + false_positives) \
                if (true_positives + false_positives) > 0 else 0.0
            
            # Recall: Of actual occupied cells, how many were detected?
            recall = true_positives / real_occ_known if real_occ_known > 0 else 0.0
            
            # F1 Score
            f1 = 2 * precision * recall / (precision + recall) \
                if (precision + recall) > 0 else 0.0
            
            # IoU for occupied cells
            intersection_occ = true_positives
            union_occ = real_occ_known + created_occupied_count - true_positives
            iou_occupied = intersection_occ / union_occ if union_occ > 0 else 0.0
            
            # IoU for free cells
            intersection_free = true_negatives
            union_free = real_free_known + created_free_count - true_negatives
            iou_free = intersection_free / union_free if union_free > 0 else 0.0
            
        else:
            # Consider all cells
            correct = true_positives + true_negatives
            incorrect = total_cells - correct
            accuracy = correct / total_cells if total_cells > 0 else 0.0
            
            precision = true_positives / (true_positives + false_positives) \
                if (true_positives + false_positives) > 0 else 0.0
            recall = true_positives / real_occupied_count if real_occupied_count > 0 else 0.0
            f1 = 2 * precision * recall / (precision + recall) \
                if (precision + recall) > 0 else 0.0
            
            intersection_occ = true_positives
            union_occ = real_occupied_count + created_occupied_count - true_positives
            iou_occupied = intersection_occ / union_occ if union_occ > 0 else 0.0
            
            intersection_free = true_negatives
            union_free = real_free_count + created_free_count - true_negatives
            iou_free = intersection_free / union_free if union_free > 0 else 0.0
        
        # Coverage: What percentage of the map was explored?
        coverage = known_cells / total_cells if total_cells > 0 else 0.0
        
        return {
            'dimensions': {
                'width': width,
                'height': height,
                'total_cells': total_cells
            },
            'real_map': {
                'occupied_cells': int(real_occupied_count),
                'free_cells': int(real_free_count)
            },
            'created_map': {
                'occupied_cells': int(created_occupied_count),
                'free_cells': int(created_free_count),
                'unknown_cells': int(created_unknown_count)
            },
            'confusion_matrix': {
                'true_positives': int(true_positives),
                'true_negatives': int(true_negatives),
                'false_positives': int(false_positives),
                'false_negatives': int(false_negatives)
            },
            'metrics': {
                'accuracy': float(accuracy),
                'precision': float(precision),
                'recall': float(recall),
                'f1_score': float(f1),
                'iou_occupied': float(iou_occupied),
                'iou_free': float(iou_free),
                'coverage': float(coverage)
            },
            'settings': {
                'ignore_unknown': ignore_unknown
            }
        }
    
    def create_difference_map(self, real_gridmap: np.ndarray, 
                               created_gridmap: np.ndarray) -> np.ndarray:
        """
        Create a difference map showing where the gridmaps disagree.
        
        Returns:
            numpy array with values:
            - 0: Both agree (both free or both occupied)
            - 1: False positive (created occupied, real free)
            - 2: False negative (created free, real occupied)
            - 3: Unknown in created map
        """
        diff_map = np.zeros_like(real_gridmap, dtype=np.int8)
        
        real_occupied = real_gridmap > 50
        real_free = real_gridmap <= 50
        
        created_occupied = created_gridmap == 100
        created_free = created_gridmap == 0
        created_unknown = created_gridmap == -1
        
        # Mark false positives
        diff_map[real_free & created_occupied] = 1
        
        # Mark false negatives
        diff_map[real_occupied & created_free] = 2
        
        # Mark unknown
        diff_map[created_unknown] = 3
        
        return diff_map
    
    def save_comparison_report(self, results: Dict, output_path: str,
                                real_name: str, created_name: str):
        """
        Save a comparison report to a file.
        
        Args:
            results: Comparison results dictionary
            output_path: Path to save the report
            real_name: Name of the real gridmap file
            created_name: Name of the created gridmap file
        """
        with open(output_path, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write("GRIDMAP COMPARISON REPORT\n")
            f.write("=" * 70 + "\n\n")
            
            f.write(f"Real gridmap:    {real_name}\n")
            f.write(f"Created gridmap: {created_name}\n")
            f.write(f"Generated:       {datetime.now().isoformat()}\n\n")
            
            f.write("-" * 70 + "\n")
            f.write("DIMENSIONS\n")
            f.write("-" * 70 + "\n")
            dims = results['dimensions']
            f.write(f"  Grid size:    {dims['width']} x {dims['height']}\n")
            f.write(f"  Total cells:  {dims['total_cells']}\n\n")
            
            f.write("-" * 70 + "\n")
            f.write("CELL COUNTS\n")
            f.write("-" * 70 + "\n")
            real = results['real_map']
            created = results['created_map']
            f.write(f"  Real map:     {real['occupied_cells']} occupied, "
                   f"{real['free_cells']} free\n")
            f.write(f"  Created map:  {created['occupied_cells']} occupied, "
                   f"{created['free_cells']} free, {created['unknown_cells']} unknown\n\n")
            
            f.write("-" * 70 + "\n")
            f.write("CONFUSION MATRIX\n")
            f.write("-" * 70 + "\n")
            cm = results['confusion_matrix']
            f.write(f"  True Positives (TP):  {cm['true_positives']:8d}  "
                   f"(Both agree: occupied)\n")
            f.write(f"  True Negatives (TN):  {cm['true_negatives']:8d}  "
                   f"(Both agree: free)\n")
            f.write(f"  False Positives (FP): {cm['false_positives']:8d}  "
                   f"(Created: occupied, Real: free)\n")
            f.write(f"  False Negatives (FN): {cm['false_negatives']:8d}  "
                   f"(Created: free, Real: occupied)\n\n")
            
            f.write("-" * 70 + "\n")
            f.write("METRICS\n")
            f.write("-" * 70 + "\n")
            m = results['metrics']
            f.write(f"  Accuracy:     {m['accuracy']*100:6.2f}%  "
                   f"(Correctly classified cells)\n")
            f.write(f"  Precision:    {m['precision']*100:6.2f}%  "
                   f"(Of occupied predictions, how many correct)\n")
            f.write(f"  Recall:       {m['recall']*100:6.2f}%  "
                   f"(Of actual obstacles, how many detected)\n")
            f.write(f"  F1 Score:     {m['f1_score']*100:6.2f}%  "
                   f"(Harmonic mean of precision/recall)\n")
            f.write(f"  IoU Occupied: {m['iou_occupied']*100:6.2f}%  "
                   f"(Intersection over Union for obstacles)\n")
            f.write(f"  IoU Free:     {m['iou_free']*100:6.2f}%  "
                   f"(Intersection over Union for free space)\n")
            f.write(f"  Coverage:     {m['coverage']*100:6.2f}%  "
                   f"(Percentage of map explored)\n\n")
            
            f.write("=" * 70 + "\n")
            f.write("SUMMARY\n")
            f.write("=" * 70 + "\n")
            f.write(f"  Overall match: {m['accuracy']*100:.1f}%\n")
            f.write(f"  Obstacle detection: {m['recall']*100:.1f}%\n")
            f.write(f"  Map coverage: {m['coverage']*100:.1f}%\n")
            f.write("=" * 70 + "\n")
        
        # Also save as JSON for programmatic access
        json_path = output_path.replace('.txt', '.json')
        with open(json_path, 'w') as f:
            json.dump(results, f, indent=2)


def get_package_paths() -> Tuple[List[str], List[str]]:
    """
    Get the paths to the real and created gridmaps directories.
    
    Searches in multiple locations to handle both source and installed packages:
    - Source directory: src/gridmaps/
    - Install directory: install/.../lib/gridmaps/
    
    Returns:
        Tuple of (list of real_dirs, list of created_dirs)
    """
    real_dirs = []
    created_dirs = []
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    
    # Check relative to script location (works for both source and install)
    real_dir = os.path.join(package_dir, 'gridmaps', 'real')
    created_dir = os.path.join(package_dir, 'gridmaps', 'created')
    
    if os.path.exists(real_dir):
        real_dirs.append(real_dir)
    if os.path.exists(created_dir):
        created_dirs.append(created_dir)
    
    # Also check common workspace locations
    # Try to find workspace root by looking for 'src' directory
    current = os.path.abspath(script_dir)
    for _ in range(10):  # Limit search depth
        parent = os.path.dirname(current)
        if parent == current:
            break
        
        # Check for typical ROS2 workspace structure
        src_gridmaps_real = os.path.join(parent, 'src', 'gridmaps', 'real')
        src_gridmaps_created = os.path.join(parent, 'src', 'gridmaps', 'created')
        
        if os.path.exists(src_gridmaps_real) and src_gridmaps_real not in real_dirs:
            real_dirs.append(src_gridmaps_real)
        if os.path.exists(src_gridmaps_created) and src_gridmaps_created not in created_dirs:
            created_dirs.append(src_gridmaps_created)
        
        # Check install directory
        install_gridmaps_real = os.path.join(parent, 'install', 'gazebo_differential_drive_robot', 
                                             'lib', 'gridmaps', 'real')
        install_gridmaps_created = os.path.join(parent, 'install', 'gazebo_differential_drive_robot',
                                                'lib', 'gridmaps', 'created')
        
        if os.path.exists(install_gridmaps_real) and install_gridmaps_real not in real_dirs:
            real_dirs.append(install_gridmaps_real)
        if os.path.exists(install_gridmaps_created) and install_gridmaps_created not in created_dirs:
            created_dirs.append(install_gridmaps_created)
        
        current = parent
    
    return real_dirs, created_dirs


def find_gridmaps_for_world(world_name: str, real_dirs: List[str], 
                            created_dirs: List[str]) -> Tuple[Optional[str], List[str]]:
    """
    Find gridmap files for a specific world.
    
    Args:
        world_name: Name of the world
        real_dirs: List of directories containing real gridmaps
        created_dirs: List of directories containing created gridmaps
        
    Returns:
        Tuple of (real_gridmap_path, list of created_gridmap_paths)
    """
    # Find real gridmap
    real_path = None
    for real_dir in real_dirs:
        if os.path.exists(real_dir):
            for f in os.listdir(real_dir):
                if f.startswith(world_name) and f.endswith('_real.npy'):
                    real_path = os.path.join(real_dir, f)
                    break
        if real_path:
            break
    
    # Find created gridmaps from all directories
    created_paths = []
    for created_dir in created_dirs:
        if os.path.exists(created_dir):
            for f in os.listdir(created_dir):
                if f.startswith(world_name) and f.endswith('.npy') and 'created' in f:
                    filepath = os.path.join(created_dir, f)
                    if filepath not in created_paths:
                        created_paths.append(filepath)
    
    return real_path, sorted(created_paths)


def main():
    """
    Main function to run gridmap comparison.
    """
    parser = argparse.ArgumentParser(
        description='Compare real and created gridmaps'
    )
    parser.add_argument('real_gridmap', nargs='?', 
                        help='Path to real gridmap .npy file')
    parser.add_argument('created_gridmap', nargs='?',
                        help='Path to created gridmap .npy file')
    parser.add_argument('--world', '-w', type=str,
                        help='World name to compare all gridmaps for')
    parser.add_argument('--output', '-o', type=str,
                        help='Output directory for comparison reports')
    parser.add_argument('--all', '-a', action='store_true',
                        help='Compare all available worlds')
    parser.add_argument('--real-dir', type=str,
                        help='Directory containing real gridmaps (can specify multiple with comma)')
    parser.add_argument('--created-dir', type=str,
                        help='Directory containing created gridmaps (can specify multiple with comma)')
    
    args = parser.parse_args()
    
    comparator = GridmapComparator()
    
    # Get default paths, then override with explicit arguments if provided
    real_dirs, created_dirs = get_package_paths()
    
    if args.real_dir:
        # Parse comma-separated directories
        explicit_real_dirs = [d.strip() for d in args.real_dir.split(',')]
        real_dirs = [d for d in explicit_real_dirs if os.path.exists(d)]
        if not real_dirs:
            print(f"Error: Specified real-dir(s) do not exist: {args.real_dir}")
            sys.exit(1)
    
    if args.created_dir:
        # Parse comma-separated directories
        explicit_created_dirs = [d.strip() for d in args.created_dir.split(',')]
        created_dirs = [d for d in explicit_created_dirs if os.path.exists(d)]
        if not created_dirs:
            print(f"Error: Specified created-dir(s) do not exist: {args.created_dir}")
            sys.exit(1)
    
    # Print search paths for debugging
    print(f"Searching for real gridmaps in: {real_dirs}")
    print(f"Searching for created gridmaps in: {created_dirs}")
    print()
    
    # Determine output directory
    if args.output:
        output_dir = args.output
    elif real_dirs:
        output_dir = os.path.join(os.path.dirname(real_dirs[0]), 'comparisons')
    else:
        output_dir = os.path.join(os.getcwd(), 'gridmap_comparisons')
    os.makedirs(output_dir, exist_ok=True)
    
    if args.real_gridmap and args.created_gridmap:
        # Compare two specific gridmaps
        real_path = args.real_gridmap
        created_path = args.created_gridmap
        
        print(f"Comparing gridmaps:")
        print(f"  Real:    {real_path}")
        print(f"  Created: {created_path}")
        print()
        
        real_map, real_meta = comparator.load_gridmap(real_path)
        created_map, created_meta = comparator.load_gridmap(created_path)
        
        results = comparator.compare(real_map, created_map)
        
        # Print summary
        m = results['metrics']
        print(f"Results:")
        print(f"  Accuracy:  {m['accuracy']*100:.2f}%")
        print(f"  Precision: {m['precision']*100:.2f}%")
        print(f"  Recall:    {m['recall']*100:.2f}%")
        print(f"  F1 Score:  {m['f1_score']*100:.2f}%")
        print(f"  Coverage:  {m['coverage']*100:.2f}%")
        
        # Save report
        report_name = f"comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        report_path = os.path.join(output_dir, report_name)
        comparator.save_comparison_report(results, report_path,
                                          os.path.basename(real_path),
                                          os.path.basename(created_path))
        print(f"\nReport saved to: {report_path}")
        
    elif args.world:
        # Compare all gridmaps for a specific world
        real_path, created_paths = find_gridmaps_for_world(
            args.world, real_dirs, created_dirs
        )
        
        if not real_path:
            print(f"Error: No real gridmap found for world '{args.world}'")
            print(f"Run: python3 real_gridmap_generator.py {args.world}")
            sys.exit(1)
        
        if not created_paths:
            print(f"Error: No created gridmaps found for world '{args.world}'")
            print("Run the lidar_gridmap_generator.py to create one.")
            sys.exit(1)
        
        print(f"Comparing gridmaps for world: {args.world}")
        print(f"Real gridmap: {real_path}")
        print(f"Created gridmaps: {len(created_paths)} found")
        print()
        
        real_map, real_meta = comparator.load_gridmap(real_path)
        
        for created_path in created_paths:
            print(f"Comparing with: {os.path.basename(created_path)}")
            created_map, created_meta = comparator.load_gridmap(created_path)
            
            results = comparator.compare(real_map, created_map)
            m = results['metrics']
            
            print(f"  Accuracy: {m['accuracy']*100:.2f}%, "
                  f"Precision: {m['precision']*100:.2f}%, "
                  f"Recall: {m['recall']*100:.2f}%, "
                  f"Coverage: {m['coverage']*100:.2f}%")
            
            # Save report
            created_name = os.path.splitext(os.path.basename(created_path))[0]
            report_name = f"comparison_{args.world}_{created_name}.txt"
            report_path = os.path.join(output_dir, report_name)
            comparator.save_comparison_report(results, report_path,
                                              os.path.basename(real_path),
                                              os.path.basename(created_path))
        
        print(f"\nReports saved to: {output_dir}")
        
    elif args.all:
        # Compare all available worlds
        if not real_dirs:
            print("Error: No real gridmaps directories found")
            print("Run: python3 real_gridmap_generator.py")
            sys.exit(1)
        
        # Get all world names from real gridmaps
        world_names = set()
        for real_dir in real_dirs:
            if os.path.exists(real_dir):
                for f in os.listdir(real_dir):
                    if f.endswith('_real.npy'):
                        world_name = f.replace('_real.npy', '')
                        world_names.add(world_name)
        
        if not world_names:
            print("No real gridmaps found.")
            sys.exit(1)
        
        print(f"Comparing all worlds: {sorted(world_names)}")
        print()
        
        for world_name in sorted(world_names):
            real_path, created_paths = find_gridmaps_for_world(
                world_name, real_dirs, created_dirs
            )
            
            if not created_paths:
                print(f"{world_name}: No created gridmaps found")
                continue
            
            real_map, _ = comparator.load_gridmap(real_path)
            
            for created_path in created_paths:
                created_map, _ = comparator.load_gridmap(created_path)
                results = comparator.compare(real_map, created_map)
                m = results['metrics']
                
                print(f"{world_name}: Accuracy={m['accuracy']*100:.1f}%, "
                      f"Coverage={m['coverage']*100:.1f}%")
                
                # Save report
                created_name = os.path.splitext(os.path.basename(created_path))[0]
                report_name = f"comparison_{world_name}_{created_name}.txt"
                report_path = os.path.join(output_dir, report_name)
                comparator.save_comparison_report(results, report_path,
                                                  os.path.basename(real_path),
                                                  os.path.basename(created_path))
        
        print(f"\nReports saved to: {output_dir}")
        
    else:
        # Print usage
        parser.print_help()
        print()
        print("Examples:")
        print("  python3 gridmap_comparator.py real.npy created.npy")
        print("  python3 gridmap_comparator.py --world world_01_empty")
        print("  python3 gridmap_comparator.py --all")


if __name__ == '__main__':
    main()
