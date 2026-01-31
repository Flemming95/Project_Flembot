import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file that starts both the robot simulation and the lidar object detector.
    
    This extends the base robot launch file with lidar object detection functionality.
    """
    
    package_name = "gazebo_differential_drive_robot"
    
    # Define launch arguments for the world and initial pose
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Specify the world file for Gazebo (e.g., empty.sdf)'
    )
    
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
    
    # Launch arguments for the lidar object detector
    distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='0.2',
        description='Distance threshold for clustering points (meters)'
    )
    
    min_cluster_size_arg = DeclareLaunchArgument(
        'min_cluster_size',
        default_value='5',
        description='Minimum number of points per cluster'
    )
    
    max_cluster_size_arg = DeclareLaunchArgument(
        'max_cluster_size',
        default_value='100',
        description='Maximum number of points per cluster'
    )
    
    # Include the base robot launch file
    base_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'R': LaunchConfiguration('R'),
            'P': LaunchConfiguration('P'),
            'Y': LaunchConfiguration('Y'),
        }.items()
    )
    
    # Create the lidar object detector node
    lidar_object_detector_node = Node(
        package=package_name,
        executable='lidar_object_detector.py',
        name='lidar_object_detector',
        output='screen',
        parameters=[{
            'distance_threshold': LaunchConfiguration('distance_threshold'),
            'min_cluster_size': LaunchConfiguration('min_cluster_size'),
            'max_cluster_size': LaunchConfiguration('max_cluster_size'),
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        distance_threshold_arg,
        min_cluster_size_arg,
        max_cluster_size_arg,
        # Nodes
        base_robot_launch,
        lidar_object_detector_node,
    ])
