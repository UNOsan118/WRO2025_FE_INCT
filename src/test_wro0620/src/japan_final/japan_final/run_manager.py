#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from ros_robot_controller_msgs.msg import BuzzerState, ButtonState

class RunManagerNode(Node):
    """
    A manager node that waits for a button press to launch the driving logic.
    """
    def __init__(self):
        super().__init__('run_manager')
        self.get_logger().info('Run Manager Node has been started.')
        
        self.state = "WAITING_FOR_START"
        self.launch_process = None

        # This node no longer needs a buzzer publisher, as it doesn't control hardware.
        # It now subscribes to the button topic to detect the start signal.
        self.button_sub = self.create_subscription(ButtonState, '/ros_robot_controller/button', self.button_callback, 10)
        
        self.get_logger().info('System ready. Waiting for start signal (Button 1)...')
        
        rclpy.get_default_context().on_shutdown(self.shutdown_callback)

    def button_callback(self, msg: ButtonState):
        if self.state == "WAITING_FOR_START" and msg.id == 1 and msg.state == 1:
            self.get_logger().info('--- START button pressed! Launching the driving logic... ---')
            self.state = "DRIVING"
            
            # This launch file ONLY contains the driving node (fuzzy_controller)
            launch_command = [
                'ros2', 'launch',
                'japan_final', 'final_run.launch.py' 
            ]
            
            self.get_logger().info(f"Executing command: {' '.join(launch_command)}")
            self.launch_process = subprocess.Popen(launch_command)
            
            self.destroy_subscription(self.button_sub)
    
    def shutdown_callback(self):
        self.get_logger().info('Shutdown requested.')
        if self.launch_process:
            self.get_logger().info('Terminating the launched driving process.')
            self.launch_process.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = RunManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()