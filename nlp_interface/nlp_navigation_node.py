#!/usr/bin/env python3

"""
Natural Language Navigation Node

This ROS2 node provides a natural language interface for the reactive navigation
controller. It subscribes to natural language commands and translates them to
the robot's native command format.

Usage:
    ros2 run gazebo_differential_drive_robot nlp_navigation_node.py

    Send natural language commands:
    ros2 topic pub /nlp/command std_msgs/msg/String "data: 'move forward two meters'" --once

Make sure the reactive_navigation_controller is running:
    ros2 run gazebo_differential_drive_robot reactive_navigation_controller.py
"""

import sys
from pathlib import Path

# Add nlp_interface to path
NLP_INTERFACE_PATH = Path(__file__).parent.parent.parent / "nlp_interface"
if NLP_INTERFACE_PATH.exists():
    sys.path.insert(0, str(NLP_INTERFACE_PATH.parent))

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

from nlp_interface import NLPCommandTranslator


class NLPNavigationNode:
    """
    ROS2 Node for natural language navigation control.
    
    This node bridges natural language input to the reactive navigation
    controller by translating human-readable commands to robot commands.
    
    Subscriptions:
        /nlp/command (String): Natural language command input
        
    Publications:
        /navigation/command (String): Translated robot commands
        /nlp/feedback (String): Translation feedback and status
    """
    
    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS2 (rclpy) is not available")
        
        # Initialize the ROS2 node
        self._node = Node("nlp_navigation_node")
        
        # Declare parameters
        self._node.declare_parameter("use_llm", True)
        self._node.declare_parameter("llm_provider", "huggingface")
        self._node.declare_parameter("llm_model", "")
        
        # Get parameters
        use_llm = self._node.get_parameter("use_llm").value
        llm_provider = self._node.get_parameter("llm_provider").value
        llm_model = self._node.get_parameter("llm_model").value or None
        
        # Initialize the translator
        self.translator = NLPCommandTranslator(
            use_llm=use_llm,
            llm_provider=llm_provider,
            llm_model=llm_model,
        )
        
        # Create subscriber for natural language commands
        self.nlp_sub = self._node.create_subscription(
            String,
            "/nlp/command",
            self.nlp_command_callback,
            10,
        )
        
        # Create publisher for translated commands
        self.command_pub = self._node.create_publisher(
            String,
            "/navigation/command",
            10,
        )
        
        # Create publisher for feedback
        self.feedback_pub = self._node.create_publisher(
            String,
            "/nlp/feedback",
            10,
        )
        
        self._node.get_logger().info("NLP Navigation Node initialized")
        self._node.get_logger().info(f"Using LLM: {use_llm}, Provider: {llm_provider}")
        self._node.get_logger().info("Listening for natural language commands on /nlp/command")
    
    def nlp_command_callback(self, msg: "String"):
        """Handle incoming natural language commands."""
        natural_language = msg.data.strip()
        
        if not natural_language:
            return
        
        self._node.get_logger().info(f"Received NL command: '{natural_language}'")
        
        # Translate the command
        try:
            command, method = self.translator.translate(natural_language)
            
            self._node.get_logger().info(
                f"Translated to: '{command}' (via {method})"
            )
            
            # Publish the translated command
            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)
            
            # Publish feedback
            feedback_msg = String()
            feedback_msg.data = (
                f"Translated '{natural_language}' -> '{command}' (method: {method})"
            )
            self.feedback_pub.publish(feedback_msg)
            
        except Exception as e:
            self._node.get_logger().error(f"Translation error: {e}")
            
            # Publish error feedback
            feedback_msg = String()
            feedback_msg.data = f"Error translating '{natural_language}': {e}"
            self.feedback_pub.publish(feedback_msg)
    
    def spin(self):
        """Spin the ROS2 node."""
        rclpy.spin(self._node)
    
    def destroy(self):
        """Destroy the ROS2 node."""
        self._node.destroy_node()


def main(args=None):
    """Main entry point."""
    if not ROS_AVAILABLE:
        print("Error: ROS2 (rclpy) is not available.")
        print("This node requires ROS2 to be installed and sourced.")
        print("\nFor testing the translator without ROS2, run:")
        print("  python -m nlp_interface.translator")
        return 1
    
    rclpy.init(args=args)
    
    try:
        node = NLPNavigationNode()
        node.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        if "node" in locals():
            node.destroy()
        rclpy.shutdown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
