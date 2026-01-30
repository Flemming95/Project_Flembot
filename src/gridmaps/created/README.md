# Created Gridmaps

This folder stores gridmaps created from lidar scan data while the robot
drives along the walls of an arena.

These gridmaps can be compared with the "real" gridmaps to measure how
well the lidar mapping matches the actual world layout.

## Generating Created Gridmaps

1. Launch the robot simulation with lidar:
   ```bash
   ros2 launch gazebo_differential_drive_robot robot_with_lidar.launch.py \
       world:=world_01_empty.sdf
   ```

2. Start the lidar gridmap generator:
   ```bash
   ros2 run gazebo_differential_drive_robot lidar_gridmap_generator.py \
       --ros-args -p world_name:=world_01_empty
   ```

3. Start the wall-following scanner (or manually drive the robot):
   ```bash
   ros2 run gazebo_differential_drive_robot wall_following_scanner.py
   ```

4. The gridmap will be automatically saved when wall following is complete,
   or you can manually save it:
   ```bash
   ros2 service call /gridmap/save std_srvs/srv/Trigger
   ```

## File Format

Each scan session generates:
- `<world_name>_created_<timestamp>.npy` - NumPy array with occupancy data
- `<world_name>_created_<timestamp>_metadata.json` - Metadata
- `<world_name>_created_<timestamp>_preview.txt` - Text visualization

## Comparing Gridmaps

Use the gridmap_comparator.py script to compare real vs created gridmaps:

```bash
python3 scripts/gridmap_comparator.py --world world_01_empty
```
