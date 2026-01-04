from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import xacro
import os
import tempfile

# --- Mapping from simple name to file ---
WORLD_MAPPING = {
    'empty': 'empty.world',
    'custom': 'custom_world.world',  # Add more worlds here
}

SIM_PKG_CANDIDATES = ['ros_gz_sim', 'ros_gz', 'gz', 'gazebo_ros']
LAUNCH_FILE_CANDIDATES = [
    'launch/ros_gz_sim.launch.py',
    'launch/gz_sim.launch.py',
    'launch/gz_server.launch.py',
    'launch/gz.launch.py',
    'launch/gazebo.launch.py',
    'launch/gazebo.launch'
]
SPAWN_LAUNCH_CANDIDATES = [
    'launch/gz_spawn_model.launch.py',
    'launch/ros_gz_spawn_model.launch.py',
    'launch/gz_spawn_model.launch',
    'launch/ros_gz_spawn_model.launch'
]
SPAWN_SCRIPT_BASENAMES = [
    'spawn_entity.py', 'spawn_entity', 'gz_spawn_model', 'ros_gz_spawn_model'
]

# ... keep helper functions find_package_with_file, find_spawn_launch, find_spawn_script, _locate_repo_config unchanged ...

def generate_launch_description():
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Name of the world to load (e.g., empty, custom)'
    )

    world_arg = LaunchConfiguration('world')

    def resolve_world(context, *args, **kwargs):
        world_name = world_arg.perform(context)

        # prefer installed package share, fallback to workspace src
        bringup_pkg = None
        try:
            bringup_pkg = get_package_share_directory('flembot_bringup')
        except PackageNotFoundError:
            pass

        if world_name not in WORLD_MAPPING:
            raise RuntimeError(f"Unknown world '{world_name}'. Available: {list(WORLD_MAPPING.keys())}")

        if bringup_pkg:
            world_file = os.path.join(bringup_pkg, 'worlds', WORLD_MAPPING[world_name])
            worlds_dir = os.path.join(bringup_pkg, 'worlds')
        else:
            world_file = os.path.join(os.getcwd(), 'src', 'flembot_bringup', 'worlds', WORLD_MAPPING[world_name])
            worlds_dir = os.path.join(os.getcwd(), 'src', 'flembot_bringup', 'worlds')

        if not os.path.exists(world_file):
            raise RuntimeError(f"World file not found: {world_file}")

        # Robot description (unchanged)
        description_pkg = get_package_share_directory('flembot_description')
        xacro_file = os.path.join(description_pkg, 'urdf', 'flembot.urdf.xacro')
        robot_description_config = xacro.process_file(xacro_file)

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_config.toxml()
            }]
        )

        # Simulator include
        sim_pkg, sim_launch = find_package_with_file(SIM_PKG_CANDIDATES, LAUNCH_FILE_CANDIDATES)
        if sim_pkg is None or sim_launch is None:
            raise RuntimeError(
                "No supported simulator launch file found. Ensure one of the packages "
                f"{SIM_PKG_CANDIDATES} is installed and provides a launch file."
            )

        # Setup launch args. For ros_gz_sim, pass world_sdf_file (full path) and set GZ_RESOURCE_PATH.
        launch_args = {}
        env_action = None
        if sim_pkg and (sim_pkg == 'ros_gz_sim' or sim_pkg.startswith('ros_gz')):
            # bridge/config as before
            launch_args['bridge_name'] = 'bridge'
            launch_args['config_file'] = _locate_repo_config()
            # Give ros_gz_sim the SDF path explicitly:
            launch_args['world_sdf_file'] = world_file
            # Also set GZ_RESOURCE_PATH so gz finds models/worlds by name if needed
            existing = os.environ.get('GZ_RESOURCE_PATH', '')
            new_val = worlds_dir + (':' + existing if existing else '')
            env_action = SetEnvironmentVariable('GZ_RESOURCE_PATH', new_val)
        else:
            # legacy gazebo_ros: pass full path as 'world' argument (keeps compatibility)
            launch_args['world'] = world_file

        sim_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments=launch_args.items(),
        )

        # Spawn robot (unchanged)
        spawn_pkg, spawn_launch = find_spawn_launch(SIM_PKG_CANDIDATES, SPAWN_LAUNCH_CANDIDATES)
        spawn_action = None

        if spawn_launch:
            spawn_action = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
                launch_arguments={'topic': 'robot_description', 'entity': 'flembot'}.items()
            )
        else:
            spawn_pkg_script, spawn_script = find_spawn_script(SIM_PKG_CANDIDATES, SPAWN_SCRIPT_BASENAMES)
            if spawn_script:
                if spawn_script.endswith('.py'):
                    spawn_action = ExecuteProcess(
                        cmd=['python3', spawn_script, '-topic', 'robot_description', '-entity', 'flembot'],
                        output='screen'
                    )
                else:
                    spawn_action = ExecuteProcess(
                        cmd=[spawn_script, '-topic', 'robot_description', '-entity', 'flembot'],
                        output='screen'
                    )
            else:
                try:
                    _ = get_package_share_directory('gazebo_ros')
                    spawn_action = Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'flembot'],
                        output='screen'
                    )
                except PackageNotFoundError:
                    raise RuntimeError(
                        "Could not find a spawn launch/script/executable for the simulator. "
                        "Install ros_gz_sim or gazebo_ros or provide a compatible spawn utility."
                    )

        actions = []
        if env_action:
            actions.append(env_action)
        actions.extend([sim_include, robot_state_publisher, spawn_action])
        return actions

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        declare_world_cmd,
        OpaqueFunction(function=resolve_world)
        ])
