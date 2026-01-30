# Real Gridmaps

This folder stores "real" gridmaps generated from Gazebo SDF world files.

These gridmaps represent the ground truth layout of each world, extracted
from the wall and obstacle definitions in the world files.

## Generating Real Gridmaps

Run the real_gridmap_generator.py script to generate gridmaps for all worlds:

```bash
python3 scripts/real_gridmap_generator.py
```

Or generate for a specific world:

```bash
python3 scripts/real_gridmap_generator.py world_01_empty
```

## File Format

Each world generates:
- `<world_name>_real.npy` - NumPy array with occupancy data (0=free, 100=occupied)
- `<world_name>_real_metadata.json` - Metadata including resolution, dimensions, origin
- `<world_name>_real_preview.txt` - Text-based visualization
