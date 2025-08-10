#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

from ros_robot_controller_msgs.msg import BuzzerState, ButtonState

class BuzzerButtonTestNode(Node):
    """
    A node to test the buzzer and buttons.
    - Beeps the buzzer for a specific duration on startup.
    - Beeps the buzzer whenever a button is pressed.
    """
    def __init__(self):
        super().__init__('buzzer_button_test_node')
        self.get_logger().info('Buzzer and Button Test Node has been started.')

        self.buzzer_pub = self.create_publisher(
            BuzzerState,
            '/ros_robot_controller/set_buzzer',
            10)

        self.button_sub = self.create_subscription(
            ButtonState,
            '/ros_robot_controller/button',
            self.button_callback,
            10)
            
        # --- Change 1: Store the timer in a member variable ---
        # This timer's only job is to call startup_beep once.
        self.startup_timer = self.create_timer(0.5, self.startup_beep)
        
    def startup_beep(self):
        """ Plays a beep for 0.5 seconds on startup. """
        self.get_logger().info('Playing startup beep for 0.5 seconds...')
        self.beep(frequency=1000, duration_sec=0.5)
        
        # --- Change 2: Cancel the startup timer so it doesn't run again ---
        self.startup_timer.cancel()

    def button_callback(self, msg: ButtonState):
        """
        This function is called every time a message is received on the button topic.
        """
        if msg.state == 1: # 'PRESSED' event
            self.get_logger().info(f'Button {msg.id} PRESSED. Beeping.')
            self.beep(frequency=1500, duration_sec=0.05)

    def beep(self, frequency: int, duration_sec: float):
        """
        Helper function to start a beep and schedule it to stop.
        """
        # Start Beeping
        start_msg = BuzzerState()
        start_msg.freq = frequency
        start_msg.on_time = 60.0
        start_msg.off_time = 0.0
        start_msg.repeat = 0
        self.buzzer_pub.publish(start_msg)

        # Schedule Stop
        stop_callback = partial(self.stop_beep)
        self.create_timer(duration_sec, stop_callback)

    def stop_beep(self):
        """
        Sends a command to stop the buzzer by setting the frequency to 0.
        """
        stop_msg = BuzzerState()
        stop_msg.freq = 0
        self.buzzer_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerButtonTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_beep()
        rclpy.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()