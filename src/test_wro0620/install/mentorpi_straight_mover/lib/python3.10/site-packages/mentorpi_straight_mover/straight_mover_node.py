import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotMover(Node):
    """
    A ROS2 node to test MentorPi robot movement (Ackermann steering).
    Publishes Twist messages to the /controller/cmd_vel topic.
    """
    def __init__(self):
        super().__init__('robot_movement_tester_node')
        
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        timer_period = 0.1  # 10 Hz for publishing commands
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # --- Movement Parameters ---
        # Adjust these values to test different movements.
        # Max advisable linear velocity (x) is -0.6 to 0.6 m/s according to documentation.
        # Max advisable angular velocity (z) is usually around -3.0 to 3.0 rad/s.
        
        self.linear_x_speed = 0.3  # Set to 0.2 for forward, -0.2 for backward, 0.0 for no linear movement
        self.linear_y_speed = 0.0  # Always 0 for Ackermann steering
        self.angular_z_speed = 1.0 # Set to 0.5 for left turn, -0.5 for right turn, 0.0 for no rotation
        
        # --- Test Mode (currently set to curved movement) ---
        self.get_logger().info('Testing curved movement. Linear Speed: %f m/s, Angular Speed: %f rad/s' % (self.linear_x_speed, self.angular_z_speed))

        # --- Register shutdown hook ---
        # This is the CORRECT way to register a shutdown callback in rclpy.
        # It's called on the rclpy default context, NOT on a logger object.
        rclpy.get_default_context().on_shutdown(self.shutdown_callback)
        self.get_logger().info('Shutdown hook registered.')

    def timer_callback(self):
        """
        Callback function for the timer. Publishes movement commands.
        """
        twist_msg = Twist()
        
        twist_msg.linear.x = self.linear_x_speed
        twist_msg.linear.y = self.linear_y_speed # Should be 0.0 for Ackermann
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.angular_z_speed
        
        self.publisher_.publish(twist_msg)

    def shutdown_callback(self):
        """
        This function is called by rclpy automatically when ROS is shutting down (e.g., on Ctrl+C).
        It sends stop commands while the publisher is still valid.
        """
        self.get_logger().info('Shutdown hook triggered. Sending final stop commands...')
        
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.angular.z = 0.0
        
        # Publish stop command multiple times to ensure it is received by the base controller.
        # A small delay is added to allow time for the message to propagate.
        for _ in range(5):
            try:
                self.publisher_.publish(stop_twist)
                time.sleep(0.02) # Wait 20 milliseconds
            except Exception as e:
                self.get_logger().warn(f"Failed to publish stop command during shutdown: {e}")
                break # If publishing fails, no need to try more

        self.get_logger().info('Stop commands sent. Robot should be halted.')

def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover() # Create the node instance
    
    try:
        rclpy.spin(robot_mover) # Spin the node; this handles KeyboardInterrupt internally
    except Exception as e: # Catch any other unexpected exceptions during spin
        robot_mover.get_logger().error(f"An unexpected error occurred during rclpy.spin(): {e}")
    finally:
        # rclpy.spin() automatically calls rclpy.shutdown() on KeyboardInterrupt.
        # Calling it again here would cause the "rcl_shutdown already called" error.
        # So, we only destroy the node here.
        robot_mover.get_logger().info('Node destruction initiated.')
        robot_mover.destroy_node()
        # rclpy.shutdown() is NOT called here.
        
if __name__ == '__main__':
    main()
