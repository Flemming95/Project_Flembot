import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file that starts the robot with full navigation support.
    
    This includes:
    - Robot simulation in Gazebo
    - Lidar object detection
    - Scan-to-image conversion
    - Navigation support node
    - RViz2 for visualization
    
    Velocity commands can be sent via a separate publisher to /cmd_vel topic.
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
    
    # Launch arguments for navigation support
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='1.0',
        description='Distance below which an obstacle is considered critical (meters)'
    )
    
    warning_threshold_arg = DeclareLaunchArgument(
        'warning_threshold',
        default_value='2.0',
        description='Distance below which a warning is issued (meters)'
    )
    
    # Launch argument for RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )
    
    # Launch arguments for scan-to-image
    save_images_arg = DeclareLaunchArgument(
        'save_images',
        default_value='false',
        description='Whether to save scan images to files'
    )
    
    image_size_arg = DeclareLaunchArgument(
        'image_size',
        default_value='500',
        description='Size of the scan image in pixels'
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
    
    # Create the scan-to-image node
    scan_to_image_node = Node(
        package=package_name,
        executable='scan_to_image.py',
        name='scan_to_image',
        output='screen',
        parameters=[{
            'image_size': LaunchConfiguration('image_size'),
            'max_range': 10.0,
            'save_images': LaunchConfiguration('save_images'),
            'save_path': '/tmp/scan_images',
            'save_interval': 1.0,
        }]
    )
    
    # Create the navigation support node
    lidar_navigation_support_node = Node(
        package=package_name,
        executable='lidar_navigation_support.py',
        name='lidar_navigation_support',
        output='screen',
        parameters=[{
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'warning_threshold': LaunchConfiguration('warning_threshold'),
            'num_sectors': 8,
            'min_valid_range': 0.1,
            'max_valid_range': 10.0,
        }]
    )
    
    # RViz2 node with pre-configured settings
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'rviz',
        'robot_navigation.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
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
        obstacle_threshold_arg,
        warning_threshold_arg,
        use_rviz_arg,
        save_images_arg,
        image_size_arg,
        # Nodes
        base_robot_launch,
        lidar_object_detector_node,
        scan_to_image_node,
        lidar_navigation_support_node,
        rviz_node,
    ])
