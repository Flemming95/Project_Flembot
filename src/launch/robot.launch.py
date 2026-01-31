import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# Dictionary mapping short world names to their full filenames
WORLD_ALIASES = {
    # By number only
    '1': 'world_01_empty.sdf',
    '2': 'world_02_boxes.sdf',
    '3': 'world_03_cylinders.sdf',
    '4': 'world_04_mixed.sdf',
    '5': 'world_05_corridor.sdf',
    '6': 'world_06_scattered.sdf',
    '7': 'world_07_zigzag.sdf',
    '8': 'world_08_corners.sdf',
    '9': 'world_09_slalom.sdf',
    '10': 'world_10_challenge.sdf',
    # By descriptive name
    'empty': 'world_01_empty.sdf',
    'boxes': 'world_02_boxes.sdf',
    'cylinders': 'world_03_cylinders.sdf',
    'mixed': 'world_04_mixed.sdf',
    'corridor': 'world_05_corridor.sdf',
    'scattered': 'world_06_scattered.sdf',
    'zigzag': 'world_07_zigzag.sdf',
    'corners': 'world_08_corners.sdf',
    'slalom': 'world_09_slalom.sdf',
    'challenge': 'world_10_challenge.sdf',
}


def resolve_world_path(world_arg: str, package_name: str) -> str:
    """
    Resolve a world argument to a full path.
    
    Accepts:
    - Short numeric aliases: '1', '2', ..., '10'
    - Descriptive aliases: 'empty', 'boxes', 'corridor', etc.
    - Full filename: 'world_01_empty.sdf'
    - Filename without extension: 'world_01_empty'
    - Full path: '/path/to/world.sdf'
    
    Returns the full path to the world file.
    """
    # If it's already an absolute path, use it directly
    if os.path.isabs(world_arg):
        return world_arg
    
    # Get the worlds directory from the package share
    worlds_dir = os.path.join(
        get_package_share_directory(package_name),
        'worlds'
    )
    
    # Check if it's a known alias
    if world_arg in WORLD_ALIASES:
        return os.path.join(worlds_dir, WORLD_ALIASES[world_arg])
    
    # Check if the filename exists directly (with or without .sdf extension)
    if not world_arg.endswith('.sdf'):
        world_with_ext = world_arg + '.sdf'
    else:
        world_with_ext = world_arg
    
    # Check if file exists in worlds directory
    full_path = os.path.join(worlds_dir, world_with_ext)
    if os.path.exists(full_path):
        return full_path
    
    # Fall back to returning the argument as-is (for built-in Gazebo worlds like 'empty.sdf')
    # Note: If the world file doesn't exist, Gazebo will report the error
    return world_arg


def generate_launch_description():
    # Define the robot's name and package name
    robot_name = "differential_drive_robot"
    package_name = "gazebo_differential_drive_robot"

    # Define a launch argument for the world file, defaulting to "empty.sdf"
    # Supports short aliases: '1'-'10' for world numbers, or descriptive names
    # like 'empty', 'boxes', 'corridor', 'challenge', etc.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description="World to load. Options: 1-10, 'empty', 'boxes', 'cylinders', 'mixed', 'corridor', 'scattered', 'zigzag', 'corners', 'slalom', 'challenge', or a file path"
    )

    # Define launch arguments for initial pose
    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0', description='Initial X position')

    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0', description='Initial Y position')

    z_arg = DeclareLaunchArgument(
        'z', default_value='0.5', description='Initial Z position')

    roll_arg = DeclareLaunchArgument(
        'R', default_value='0.0', description='Initial Roll')

    pitch_arg = DeclareLaunchArgument(
        'P', default_value='0.0', description='Initial Pitch')

    yaw_arg = DeclareLaunchArgument(
        'Y', default_value='0.0', description='Initial Yaw')

    # Retrieve launch configurations for initial pose
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # Set paths to Xacro model and configuration files
    robot_model_path = os.path.join(
        get_package_share_directory(package_name),
        'model',
        'robot.xacro'
    )

    gz_bridge_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_bridge.yaml'
    )

    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(robot_model_path).toxml()

    # Function to create Gazebo launch with resolved world path
    def create_gazebo_launch_with_resolved_world(context):
        # Get the world argument value at launch time
        world_input = LaunchConfiguration('world').perform(context)
        resolved_world = resolve_world_path(world_input, package_name)

        # Prepare to include the Gazebo simulation launch file
        gazebo_pkg_launch = PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        )

        # Include the Gazebo launch description with resolved world path
        gazebo_launch = IncludeLaunchDescription(
            gazebo_pkg_launch,
            launch_arguments={
                'gz_args': f'-r -v 4 {resolved_world}',
                'on_exit_shutdown': 'true'
            }.items()
        )

        return [gazebo_launch]

    # Create a node to spawn the robot model in the Gazebo environment
    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-string', robot_description,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen',
    )

    # Create a node to publish the robot's state based on its URDF description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}
        ],
        output='screen'
    )

    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        OpaqueFunction(function=create_gazebo_launch_with_resolved_world),
        spawn_model_gazebo_node,
        robot_state_publisher_node,
        gz_bridge_node,
    ])
