#!/usr/bin/env python3

"""
Navigation Command Examples

This script provides example functions demonstrating how to use the reactive
navigation controller for object detection-based navigation.

Run this script to see examples of how to send commands to the navigation
controller, or use it as a reference for integrating navigation commands
into your own applications.

Usage:
    ros2 run gazebo_differential_drive_robot navigation_examples.py

Make sure the reactive_navigation_controller is running:
    ros2 run gazebo_differential_drive_robot reactive_navigation_controller.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import time


class NavigationExamples(Node):
    """
    A ROS2 node demonstrating navigation command examples.
    
    This class provides methods for sending various navigation commands
    to the reactive navigation controller.
    """

    def __init__(self):
        super().__init__('navigation_examples')
        
        # Publisher for navigation commands
        self.command_pub = self.create_publisher(
            String,
            '/navigation/command',
            10
        )
        
        # Subscriber for navigation status
        self.status_sub = self.create_subscription(
            String,
            '/navigation/status',
            self.status_callback,
            10
        )
        
        # Subscriber for condition triggers
        self.trigger_sub = self.create_subscription(
            String,
            '/navigation/condition_triggered',
            self.trigger_callback,
            10
        )
        
        # Service clients
        self.stop_client = self.create_client(Trigger, '/navigation/stop')
        self.pause_client = self.create_client(Trigger, '/navigation/pause')
        self.resume_client = self.create_client(Trigger, '/navigation/resume')
        
        self.latest_status = None
        self.get_logger().info('Navigation Examples node initialized')

    def status_callback(self, msg: String):
        """Callback for navigation status updates."""
        self.latest_status = json.loads(msg.data)

    def trigger_callback(self, msg: String):
        """Callback for condition trigger events."""
        data = json.loads(msg.data)
        self.get_logger().info(
            f'Condition triggered: {data.get("condition")} = {data.get("value"):.2f}m'
        )

    def send_command(self, command: str):
        """
        Send a navigation command string.
        
        Args:
            command: The command string to send
        """
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')

    def send_json_command(self, command_dict: dict):
        """
        Send a navigation command as JSON.
        
        Args:
            command_dict: Dictionary with command parameters
        """
        msg = String()
        msg.data = json.dumps(command_dict)
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent JSON command: {command_dict}')

    def stop_navigation(self):
        """Stop all navigation."""
        if self.stop_client.wait_for_service(timeout_sec=1.0):
            future = self.stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result():
                self.get_logger().info(f'Stop result: {future.result().message}')
        else:
            self.get_logger().warn('Stop service not available')

    def pause_navigation(self):
        """Pause current navigation."""
        if self.pause_client.wait_for_service(timeout_sec=1.0):
            future = self.pause_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result():
                self.get_logger().info(f'Pause result: {future.result().message}')

    def resume_navigation(self):
        """Resume paused navigation."""
        if self.resume_client.wait_for_service(timeout_sec=1.0):
            future = self.resume_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result():
                self.get_logger().info(f'Resume result: {future.result().message}')

    # =========================================================================
    # Example Navigation Commands
    # =========================================================================

    def example_move_forward_until_wall(self):
        """
        Example: Move forward until close to a wall, then turn left.
        
        This demonstrates the core use case of reactive navigation based on
        object detection.
        """
        self.get_logger().info('--- Example: Move forward until wall, then turn left ---')
        
        # Move forward until the front obstacle is within 0.5 meters,
        # then automatically turn left
        self.send_command('forward_until_close:front:0.5:turn_left')

    def example_navigate_around_obstacle(self):
        """
        Example: Navigate around an obstacle.
        
        This demonstrates a sequence of commands to navigate around a detected
        obstacle.
        """
        self.get_logger().info('--- Example: Navigate around obstacle ---')
        
        # Sequence:
        # 1. Move forward until obstacle
        # 2. Turn left 90 degrees
        # 3. Move forward a bit
        # 4. Turn right 90 degrees
        # 5. Move forward until clear
        commands = 'forward_until_close:front:0.5:turn_left;' \
                   'turn_left:90;' \
                   'forward:1.0;' \
                   'turn_right:90;' \
                   'forward:2.0'
        
        self.send_command(commands)

    def example_wall_following(self):
        """
        Example: Follow a wall on the left side.
        
        Maintains a distance of 0.5 meters from the left wall for 10 meters.
        """
        self.get_logger().info('--- Example: Wall following ---')
        
        # Follow wall on left side, maintain 0.5m distance, for 10m
        self.send_command('follow_wall:left:0.5:10.0')

    def example_find_clear_path(self):
        """
        Example: Turn until finding a clear path.
        
        Useful when the robot encounters a dead end.
        """
        self.get_logger().info('--- Example: Find clear path ---')
        
        # Turn left until front is clear (at least 1.0 meter)
        self.send_command('turn_until_clear:left:1.0')

    def example_json_commands(self):
        """
        Example: Send commands using JSON format.
        
        JSON format allows for more structured command specification.
        """
        self.get_logger().info('--- Example: JSON commands ---')
        
        # Single command
        self.send_json_command({
            'action': 'forward_until_close',
            'condition_direction': 'front',
            'condition_distance': 0.5,
            'next_action': 'turn_left:90'
        })

    def example_multiple_json_commands(self):
        """
        Example: Send multiple commands as a JSON array.
        """
        self.get_logger().info('--- Example: Multiple JSON commands ---')
        
        commands = [
            {'action': 'forward', 'distance': 1.0},
            {'action': 'turn_right', 'angle': 90},
            {'action': 'forward', 'distance': 0.5}
        ]
        
        msg = String()
        msg.data = json.dumps(commands)
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent {len(commands)} JSON commands')

    def example_simple_movements(self):
        """
        Example: Simple movement commands.
        """
        self.get_logger().info('--- Example: Simple movements ---')
        
        # Move forward for 1 meter
        self.send_command('forward:1.0')

    def example_backward_until_close(self):
        """
        Example: Move backward until close to an obstacle.
        """
        self.get_logger().info('--- Example: Backward until close ---')
        
        # Move backward until back sensor detects obstacle within 0.5m
        self.send_command('backward_until_close:back:0.5')


def main(args=None):
    """
    Main function demonstrating navigation examples.
    """
    rclpy.init(args=args)
    
    node = NavigationExamples()
    
    # Wait for controller to be ready
    node.get_logger().info('Waiting for navigation controller...')
    time.sleep(1.0)
    
    # Print available examples
    node.get_logger().info('')
    node.get_logger().info('=' * 60)
    node.get_logger().info('NAVIGATION COMMAND EXAMPLES')
    node.get_logger().info('=' * 60)
    node.get_logger().info('')
    node.get_logger().info('Available command formats:')
    node.get_logger().info('  stop                              - Stop all movement')
    node.get_logger().info('  forward                           - Move forward indefinitely')
    node.get_logger().info('  forward:2.0                       - Move forward for 2 meters')
    node.get_logger().info('  backward:1.0                      - Move backward for 1 meter')
    node.get_logger().info('  turn_left                         - Turn left 90 degrees')
    node.get_logger().info('  turn_left:45                      - Turn left 45 degrees')
    node.get_logger().info('  turn_right:90                     - Turn right 90 degrees')
    node.get_logger().info('  forward_until_close:front:0.5     - Move forward until front is 0.5m')
    node.get_logger().info('  forward_until_close:front:0.5:turn_left - Then turn left')
    node.get_logger().info('  turn_until_clear:left:1.0         - Turn left until 1m clear')
    node.get_logger().info('  follow_wall:left:0.5              - Follow wall at 0.5m')
    node.get_logger().info('  follow_wall:left:0.5:10.0         - Follow wall for 10m')
    node.get_logger().info('')
    node.get_logger().info('Multiple commands can be chained with ";"')
    node.get_logger().info('Example: forward:1.0;turn_left:90;forward:2.0')
    node.get_logger().info('')
    node.get_logger().info('Directions: front, front_left, front_right, left, right,')
    node.get_logger().info('            back, back_left, back_right, any')
    node.get_logger().info('')
    node.get_logger().info('=' * 60)
    node.get_logger().info('')
    
    # Run a demonstration
    node.get_logger().info('Running demonstration: Move forward until wall, then turn left')
    node.get_logger().info('(Press Ctrl+C to stop)')
    node.get_logger().info('')
    
    # Send the example command
    node.example_move_forward_until_wall()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping navigation...')
        node.stop_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
