from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

# --- Mapping from simple name to file ---
WORLD_MAPPING = {
    'empty': 'empty.world',
    'custom': 'custom_world.world',  # Add more worlds here
}

def generate_launch_description():

    # --- Declare launch argument ---
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Name of the world to load (e.g., empty, custom)'
    )

    world_arg = LaunchConfiguration('world')

    # --- Resolve world file path dynamically ---
    def resolve_world(context, *args, **kwargs):
        world_name = world_arg.perform(context)
        bringup_pkg = get_package_share_directory('flembot_bringup')

        if world_name not in WORLD_MAPPING:
            raise RuntimeError(f"Unknown world '{world_name}'. Available: {list(WORLD_MAPPING.keys())}")

        world_file = os.path.join(bringup_pkg, 'worlds', WORLD_MAPPING[world_name])

        # --- Robot description ---
        description_pkg = get_package_share_directory('flembot_description')
        xacro_file = os.path.join(description_pkg, 'urdf', 'my_robot.urdf.xacro')
        robot_description_config = xacro.process_file(xacro_file)

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_config.toxml()
            }]
        )

        # --- Gazebo ---
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            launch_arguments={'world': world_file}.items(),
        )

        # --- Spawn robot ---
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'flembot'
            ],
            output='screen'
        )

        return [gazebo, robot_state_publisher, spawn_entity]

    return LaunchDescription([
        declare_world_cmd,
        OpaqueFunction(function=resolve_world)
    ])

