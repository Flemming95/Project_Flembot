# flembot_bringup config

This directory contains configuration files used when launching the simulator.

Files:
- `ros_gz_config.yaml` — minimal configuration for ros_gz_sim and the ros-gz bridge. Edit this file to set defaults for `bridge_name`, transport, physics options, or any other parameters expected by the `ros_gz_sim` launch files.

Notes:
- The launch file `src/flembot_bringup/launch/sim.launch.py` will look for this file in the installed package-share `share/flembot_bringup/config/ros_gz_config.yaml`. If present, it will pass it to ros_gz_sim as the `config_file` launch-argument.
- If you add new names/parameters here, ensure the included ros_gz_sim launch accepts the same argument names (or map them in the include).
