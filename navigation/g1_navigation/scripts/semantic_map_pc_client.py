#!/usr/bin/env python3
"""
Sends instructions to PC, receives goal poses, sends to Nav2
"""

import socket
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math
from dotenv import load_dotenv
import os

class DovSGLaptopClient(Node):
    def __init__(self):
        super().__init__('dovsg_laptop_client')
        load_dotenv()
        
        # ROS2 Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # PC connection details
        self.pc_ip = os.getenv("IP_ADDRESS")
        self.pc_port = int(os.getenv("PORT"))
        self.timeout = 30  # 30 seconds timeout for DovSG processing
        
        self.get_logger().info(f"Laptop Client initialized")
        self.get_logger().info(f"PC Server: {self.pc_ip}:{self.pc_port}")
    
    def send_instruction_to_pc(self, instruction: str) -> dict:
        """
        Send instruction to PC and get goal pose
        
        Args:
            instruction: Task description (e.g., "table", "red pepper")
            
        Returns:
            dict with status, x, y, yaw, message
        """
        try:
            self.get_logger().info(f"Connecting to PC at {self.pc_ip}:{self.pc_port}...")
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect((self.pc_ip, self.pc_port))
            
            self.get_logger().info("Connected! Sending instruction...")
            
            # Send instruction
            request = {'instruction': instruction}
            sock.sendall(json.dumps(request).encode())
            
            # Receive response
            self.get_logger().info("Waiting for goal pose from DovSG...")
            response_data = sock.recv(4096).decode()
            response = json.loads(response_data)
            
            sock.close()
            
            if response.get('status') == 'success':
                self.get_logger().info(
                    f"Goal pose received: x={response['x']:.3f}, "
                    f"y={response['y']:.3f}, yaw={response['yaw']:.3f}"
                )
            else:
                self.get_logger().error(f"PC returned error: {response.get('message')}")
            
            return response
            
        except socket.timeout:
            self.get_logger().error("Timeout waiting for PC response")
            return {'status': 'error', 'message': 'Connection timeout'}
        except ConnectionRefusedError:
            self.get_logger().error("Connection refused - is PC server running?")
            return {'status': 'error', 'message': 'Connection refused'}
        except Exception as e:
            self.get_logger().error(f"Error communicating with PC: {e}")
            return {'status': 'error', 'message': str(e)}
    
    def send_goal_to_nav2(self, x: float, y: float, yaw: float):
        """Send navigation goal to Nav2"""
        self.get_logger().info(f"Sending goal to Nav2: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal)
        
        self.get_logger().info("Goal sent to Nav2!")
        return future
    
    def process_task_instruction(self, instruction: str) -> bool:
        """
        Complete pipeline: instruction → PC (DovSG) → Nav2
        
        Args:
            instruction: Task description
            
        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Processing task: '{instruction}'")
        self.get_logger().info(f"{'='*60}")
        
        # Step 1: Send instruction to PC, get goal pose
        self.get_logger().info("Step 1: Sending instruction to PC for DovSG processing...")
        response = self.send_instruction_to_pc(instruction)
        
        if response.get('status') != 'success':
            self.get_logger().error(f"Failed to get goal pose: {response.get('message')}")
            return False
        
        # Step 2: Send goal pose to Nav2
        self.get_logger().info("Step 2: Sending goal pose to Nav2...")
        self.send_goal_to_nav2(
            x=response['x'],
            y=response['y'],
            yaw=response['yaw']
        )
        
        self.get_logger().info("Task processing complete!")
        return True


def main():
    import sys
    
    rclpy.init()
    client = DovSGLaptopClient()
    
    if len(sys.argv) > 1:
        # Single instruction from command line
        instruction = ' '.join(sys.argv[1:])
        client.process_task_instruction(instruction)
    else:
        # Interactive mode
        client.get_logger().info("\n" + "="*60)
        client.get_logger().info("DovSG Laptop Client - Interactive Mode")
        client.get_logger().info("="*60)
        client.get_logger().info("Enter task instructions (e.g., 'table', 'red pepper')")
        client.get_logger().info("Type 'exit' to quit\n")
        
        try:
            while True:
                instruction = input("Enter instruction: ").strip()
                
                if instruction.lower() == 'exit':
                    break
                
                if not instruction:
                    continue
                
                client.process_task_instruction(instruction)
                print()  # Blank line between tasks
                
        except KeyboardInterrupt:
            client.get_logger().info("\nShutting down...")
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
