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


def find_package_with_file(pkg_names, filenames):
    """
    Return (pkg_name, full_path) for the first matching file found in package shares,
    or (None, None) if none found.
    """
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
    """
    Return (pkg_name, full_path) for the first matching spawn launch file found.
    """
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
    """
    Search inside each candidate package share for a file with one of the given basenames.
    Return (pkg_name, full_path) or (None, None).
    """
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
        pass

    # 2) workspace source path (helpful during development before colcon install)
    src_path = os.path.join(os.getcwd(), 'src', 'flembot_bringup', 'config', 'ros_gz_config.yaml')
    if os.path.exists(src_path):
        return src_path

    # 3) create tiny temp file to satisfy the included launch's requirement
    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml', prefix='ros_gz_cfg_')
    tmp.write('# auto-generated minimal ros_gz config\n[]\n')
    tmp.flush()
    tmp.close()
    return tmp.name


def _build_gz_resource_path_with_defaults(worlds_dir):
    """
    Build a robust GZ_RESOURCE_PATH that:
      - preserves existing GZ_RESOURCE_PATH if set,
      - prepends the local worlds_dir so local resources are found first,
      - appends common system model/resource locations (only those that actually exist).
    Returns the final string to set in the environment.
    """
    existing = os.environ.get('GZ_RESOURCE_PATH', '')

    # Candidate system locations to append if they exist on this machine
    sys_candidates = [
        '/usr/share/gz',                 # gz system dir
        '/usr/share/gz/models',
        '/usr/share/gazebo',             # older gazebo paths
        '/usr/share/gazebo/models',
        '/usr/share/gazebo-11/models',   # common packaged path (Gazebo 11)
        '/usr/share/gazebo-11',
        '/usr/local/share/gazebo/models',
        '/usr/local/share/gz/models',
    ]

    sys_existing = [p for p in sys_candidates if os.path.exists(p)]

    # Build components; keep existing (if any) and add system existing paths
    components = []
    # local worlds dir should be first so local files are found preferentially
    components.append(worlds_dir)
    # keep existing if non-empty
    if existing:
        components.append(existing)
    # append any system locations found
    components.extend(sys_existing)

    # Deduplicate while preserving order
    seen = set()
    final_components = []
    for c in components:
        if c not in seen:
            final_components.append(c)
            seen.add(c)

    return ':'.join(final_components)


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

        # Setup launch args. For ros_gz_sim, pass world_sdf_file (full path) and set GZ_RESOURCE_PATH.
        launch_args = {}
        env_action = None
        if sim_pkg and (sim_pkg == 'ros_gz_sim' or sim_pkg.startswith('ros_gz')):
            # bridge/config as before
            launch_args['bridge_name'] = 'bridge'
            launch_args['config_file'] = _locate_repo_config()
            # Give ros_gz_sim the SDF path explicitly:
            launch_args['world_sdf_file'] = world_file
            # Build a safe GZ_RESOURCE_PATH that preserves system paths
            gz_resource_value = _build_gz_resource_path_with_defaults(worlds_dir)
            env_action = SetEnvironmentVariable('GZ_RESOURCE_PATH', gz_resource_value)
        else:
            # legacy gazebo_ros: pass full path as 'world' argument (keeps compatibility)
            launch_args['world'] = world_file

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

        actions = []
        if env_action:
            actions.append(env_action)
        actions.extend([sim_include, robot_state_publisher, spawn_action])
        return actions

    return LaunchDescription([
        declare_world_cmd,
        OpaqueFunction(function=resolve_world)
    ])
