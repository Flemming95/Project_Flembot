from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
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


def _locate_repo_config():
    """
    Return a path to a ros_gz config file with the following preference:
      1) installed package share: <share/flembot_bringup>/config/ros_gz_config.yaml
      2) workspace source path: <cwd>/src/flembot_bringup/config/ros_gz_config.yaml
      3) create a small temp file and return its path
    """
    # 1) installed package share
    try:
        share = get_package_share_directory('flembot_bringup')
        installed_path = os.path.join(share, 'config', 'ros_gz_config.yaml')
        if os.path.exists(installed_path):
            return installed_path
    except PackageNotFoundError:
        installed_path = None

    # 2) workspace source path (helpful during development before colcon install)
    src_path = os.path.join(os.getcwd(), 'src', 'flembot_bringup', 'config', 'ros_gz_config.yaml')
    if os.path.exists(src_path):
        return src_path

    # 3) create tiny temp file to satisfy the included launch's requirement
    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml', prefix='ros_gz_cfg_')
    tmp.write('# auto-generated minimal ros_gz config\n')
    tmp.flush()
    tmp.close()
    return tmp.name


def generate_launch_description():
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Name of the world to load (e.g., empty, custom)'
    )

    world_arg = LaunchConfiguration('world')

    def resolve_world(context, *args, **kwargs):
        world_name = world_arg.perform(context)
        bringup_pkg = None
        try:
            bringup_pkg = get_package_share_directory('flembot_bringup')
        except PackageNotFoundError:
            # it's okay; we'll still attempt to use workspace src path for config
            pass

        if world_name not in WORLD_MAPPING:
            raise RuntimeError(f"Unknown world '{world_name}'. Available: {list(WORLD_MAPPING.keys())}")

        world_file = None
        if bringup_pkg:
            world_file = os.path.join(bringup_pkg, 'worlds', WORLD_MAPPING[world_name])
        else:
            # fallback to workspace src
            world_file = os.path.join(os.getcwd(), 'src', 'flembot_bringup', 'worlds', WORLD_MAPPING[world_name])
        if not os.path.exists(world_file):
            raise RuntimeError(f"World file not found: {world_file}")

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

        # Simulator include
        sim_pkg, sim_launch = find_package_with_file(SIM_PKG_CANDIDATES, LAUNCH_FILE_CANDIDATES)
        if sim_pkg is None or sim_launch is None:
            raise RuntimeError(
                "No supported simulator launch file found. Ensure one of the packages "
                f"{SIM_PKG_CANDIDATES} is installed and provides a launch file."
            )

        # Prepare launch args and supply config_file if requested by ros_gz-style launches
        launch_args = {'world': world_file}
        if sim_pkg and (sim_pkg.startswith('ros_gz') or sim_pkg == 'ros_gz_sim'):
            launch_args['bridge_name'] = 'bridge'
            cfg_path = _locate_repo_config()
            launch_args['config_file'] = cfg_path

        sim_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments=launch_args.items(),
        )

        # Spawn robot
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

        return [sim_include, robot_state_publisher, spawn_action]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        declare_world_cmd,
        OpaqueFunction(function=resolve_world)
    ])
