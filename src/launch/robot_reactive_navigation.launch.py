import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for reactive navigation with object detection.
    
    This launch file starts:
    - Robot simulation in Gazebo
    - Lidar object detection
    - Reactive navigation controller
    - Optionally RViz2 for visualization
    
    The reactive navigation controller allows for object detection-based
    navigation commands such as "move forward until close to wall, then turn".
    
    Usage:
        ros2 launch gazebo_differential_drive_robot robot_reactive_navigation.launch.py
    
    To send navigation commands:
        ros2 topic pub /navigation/command std_msgs/msg/String "data: 'forward_until_close:front:0.5:turn_left'"
    
    Or use the example script:
        ros2 run gazebo_differential_drive_robot navigation_examples.py
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
    
    # Launch arguments for the reactive navigation controller
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.3',
        description='Linear velocity for navigation (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Angular velocity for navigation (rad/s)'
    )
    
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='0.5',
        description='Default distance threshold for obstacle detection (meters)'
    )
    
    safety_threshold_arg = DeclareLaunchArgument(
        'safety_threshold',
        default_value='0.3',
        description='Safety threshold for emergency stop (meters)'
    )
    
    # Launch arguments for RViz
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz2'
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
            'distance_threshold': 0.2,
            'min_cluster_size': 5,
            'max_cluster_size': 100,
        }]
    )
    
    # Create the reactive navigation controller node
    reactive_navigation_controller_node = Node(
        package=package_name,
        executable='reactive_navigation_controller.py',
        name='reactive_navigation_controller',
        output='screen',
        parameters=[{
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'default_obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'safety_threshold': LaunchConfiguration('safety_threshold'),
            'min_valid_range': 0.1,
            'max_valid_range': 10.0,
            'front_cone_angle': 45.0,
            'side_cone_angle': 60.0,
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
        linear_speed_arg,
        angular_speed_arg,
        obstacle_threshold_arg,
        safety_threshold_arg,
        use_rviz_arg,
        # Nodes
        base_robot_launch,
        lidar_object_detector_node,
        reactive_navigation_controller_node,
        rviz_node,
    ])
