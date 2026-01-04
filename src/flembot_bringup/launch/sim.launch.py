from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import xacro
import os

# --- Mapping from simple name to file ---
WORLD_MAPPING = {
    'empty': 'empty.world',
    'custom': 'custom_world.world',  # Add more worlds here
}

# Search candidates: include ros_gz_sim (new), ros_gz, gz, and legacy gazebo_ros
SIM_PKG_CANDIDATES = ['ros_gz_sim', 'ros_gz', 'gz', 'gazebo_ros']
# Common simulator launch files to try (prioritise ros_gz_sim names)
LAUNCH_FILE_CANDIDATES = [
    'launch/ros_gz_sim.launch.py',
    'launch/gz_sim.launch.py',
    'launch/gz_server.launch.py',
    'launch/gz.launch.py',
    'launch/gazebo.launch.py',       # legacy gazebo_ros
    'launch/gazebo.launch'           # fallback
]
# Spawn-launch candidates (ros_gz_sim provides gz_spawn_model / ros_gz_spawn_model)
SPAWN_LAUNCH_CANDIDATES = [
    'launch/gz_spawn_model.launch.py',
    'launch/ros_gz_spawn_model.launch.py',
    'launch/gz_spawn_model.launch',
    'launch/ros_gz_spawn_model.launch'
]
# Spawn script/file basenames as last-ditch attempt
SPAWN_SCRIPT_BASENAMES = [
    'spawn_entity.py', 'spawn_entity', 'gz_spawn_model', 'ros_gz_spawn_model'
]


def find_package_with_file(pkg_names, filenames):
    for pkg in pkg_names:
        try:
            pkg_share = get_package_share_directory(pkg)
        except PackageNotFoundError:
            continue
        for fname in filenames:
            candidate = os.path.join(pkg_share, fname)
            if os.path.exists(candidate):
                return pkg, candidate
    return None, None


def find_spawn_launch(pkg_names, filenames):
    for pkg in pkg_names:
        try:
            pkg_share = get_package_share_directory(pkg)
        except PackageNotFoundError:
            continue
        for fname in filenames:
            candidate = os.path.join(pkg_share, fname)
            if os.path.exists(candidate):
                return pkg, candidate
    return None, None


def find_spawn_script(pkg_names, basenames):
    for pkg in pkg_names:
        try:
            pkg_share = get_package_share_directory(pkg)
        except PackageNotFoundError:
            continue
        for root, _, files in os.walk(pkg_share):
            for f in files:
                if f in basenames:
                    return pkg, os.path.join(root, f)
    return None, None


def generate_launch_description():
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Name of the world to load (e.g., empty, custom)'
    )

    world_arg = LaunchConfiguration = 'world'  # used only for clarity below

    def resolve_world(context, *args, **kwargs):
        # get world argument value
        world_name = context.launch_configurations.get('world', 'empty')
        bringup_pkg = get_package_share_directory('flembot_bringup')

        if world_name not in WORLD_MAPPING:
            raise RuntimeError(f"Unknown world '{world_name}'. Available: {list(WORLD_MAPPING.keys())}")

        world_file = os.path.join(bringup_pkg, 'worlds', WORLD_MAPPING[world_name])

        # Robot description
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

        # Simulator include: prefer ros_gz_sim/ros_gz/gz, fallback to gazebo_ros
        sim_pkg, sim_launch = find_package_with_file(SIM_PKG_CANDIDATES, LAUNCH_FILE_CANDIDATES)
        if sim_pkg is None or sim_launch is None:
            raise RuntimeError(
                "No supported simulator launch file found. Ensure one of the packages "
                f"{SIM_PKG_CANDIDATES} is installed and provides a launch file."
            )

        sim_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={'world': world_file}.items(),
        )

        # Try to find a spawn launch (ros_gz_sim provides gz_spawn_model.launch)
        spawn_pkg, spawn_launch = find_spawn_launch(SIM_PKG_CANDIDATES, SPAWN_LAUNCH_CANDIDATES)
        spawn_action = None

        if spawn_launch:
            # Many ros_gz_sim spawn launch files accept 'topic' and 'entity' or 'name' arguments;
            # try common names. We'll pass 'topic' and 'entity' which matches the legacy behaviour.
            spawn_action = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch),
                launch_arguments={'topic': 'robot_description', 'entity': 'flembot'}.items()
            )
        else:
            # As a fallback, try to find a spawn script file anywhere (e.g., spawn_entity.py)
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
                # Last resort: try legacy gazebo_ros spawn executable as a Node (if gazebo_ros installed)
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

        return [sim_include, robot_state_publisher, spawn_action]

    # Return LaunchDescription with OpaqueFunction so we can resolve runtime package locations
    from launch.actions import OpaqueFunction
    return LaunchDescription([
        declare_world_cmd,
        OpaqueFunction(function=resolve_world)
    ])
