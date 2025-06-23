import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time # For brief sleep if needed

class StopRobot(Node):
    """
    A ROS2 node to stop the MentorPi robot by continuously publishing zero Twist messages.
    """
    def __init__(self):
        super().__init__('stop_robot_node')
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        timer_period = 0.1 # Publish at 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Robot stop node started. Publishing zero velocity to /controller/cmd_vel.')

    def timer_callback(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        self.publisher_.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    stop_robot_node = StopRobot()
    try:
        rclpy.spin(stop_robot_node)
    except KeyboardInterrupt:
        stop_robot_node.get_logger().info('Stop robot node interrupted. Ensuring robot is stopped.')
        # Ensure one last stop command is sent before shutdown
        final_stop_cmd = Twist()
        final_stop_cmd.linear.x = 0.0
        final_stop_cmd.angular.z = 0.0
        stop_robot_node.publisher_.publish(final_stop_cmd)
        time.sleep(0.1) # Give a brief moment for the message to be processed
    finally:
        stop_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
