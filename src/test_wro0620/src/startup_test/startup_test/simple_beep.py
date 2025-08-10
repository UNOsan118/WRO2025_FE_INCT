#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState

class SimpleBeepNode(Node):
    """
    A robust node that beeps the buzzer once for a specific duration and then exits.
    It explicitly sends a 'stop' command to ensure the sound terminates.
    """
    def __init__(self):
        super().__init__('simple_beep_node')
        
        self.declare_parameter('frequency', 1500)
        self.declare_parameter('duration_sec', 0.2)
        
        self.freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.duration = self.get_parameter('duration_sec').get_parameter_value().double_value

        self.get_logger().info(f'Preparing to beep at {self.freq} Hz for {self.duration} seconds.')
        
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        
        # Start a timer to periodically check for a subscriber connection.
        self.connection_timer = self.create_timer(0.3, self._wait_for_connection_and_beep)

    def _wait_for_connection_and_beep(self):
        """
        Checks for a subscriber. Once connected, sends a command to start beeping,
        then schedules a separate command to stop it.
        """
        # Wait until at least one subscriber is connected.
        if self.buzzer_pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for buzzer subscriber...', throttle_duration_sec=1.0)
            return

        # Connection is established. Cancel this timer.
        self.connection_timer.cancel()
        
        # --- Start Beeping ---
        # Send a command to start beeping indefinitely.
        self.get_logger().info('Subscriber found. Starting beep...')
        start_msg = BuzzerState()
        start_msg.freq = self.freq
        start_msg.on_time = 60.0  # A long duration to simulate continuous sound
        start_msg.off_time = 0.0
        start_msg.repeat = 0      # 0 usually means repeat forever
        self.buzzer_pub.publish(start_msg)
        
        # --- Schedule Stop ---
        # Create a new one-shot timer to stop the beep and shut down the node.
        self.shutdown_timer = self.create_timer(self.duration, self._stop_beep_and_shutdown)

    def _stop_beep_and_shutdown(self):
        """Sends a command to stop the buzzer and then shuts down the node."""
        # --- Stop Beeping ---
        # A message with frequency 0 is the standard way to stop the sound.
        self.get_logger().info('Stopping beep...')
        stop_msg = BuzzerState()
        stop_msg.freq = 0
        self.buzzer_pub.publish(stop_msg)
        
        # --- Schedule Final Exit ---
        # Give the stop message a moment to be sent before exiting the process.
        self.create_timer(0.1, self.shutdown_callback)

    def shutdown_callback(self):
        # Gracefully exit the node
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBeepNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()