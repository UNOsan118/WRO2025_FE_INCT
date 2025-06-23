import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import time
from tf_transformations import euler_from_quaternion

class YawController(Node):
    """
    A ROS2 node to control MentorPi's yaw angle based on IMU data.
    It subscribes to IMU data, calculates the current yaw angle,
    and publishes Twist messages to /controller/cmd_vel to maintain a target yaw.
    """
    def __init__(self): 
        super().__init__('yaw_controller_node')
        
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.imu_subscription_ = self.create_subscription(
            Imu,                     
            '/imu',                  
            self.imu_callback,       
            10                       
        )
        self.imu_subscription_ 
        
        
        self.linear_x_speed = 0.2  
        self.target_yaw_degrees = 0.0 
        
        
        
        self.kp = 0.02   
        self.ki = 0.0000 
        self.kd = 0.000  
        
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()
        
        self.current_yaw_degrees = 0.0 
        self.yaw_received = False      
        
        
        
        timer_period = 0.2 # 0.05 
        self.control_timer = self.create_timer(timer_period, self.control_loop_callback)
        
        
        rclpy.get_default_context().on_shutdown(self.shutdown_callback)
        self.get_logger().info('Yaw Controller Node Started. Subscribing to /imu and publishing to /controller/cmd_vel.')
        self.get_logger().info('Target Yaw: %f degrees' % self.target_yaw_degrees)

    def imu_callback(self, msg):
        """
        Callback function for received Imu messages.
        Updates the current yaw angle.
        """
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw_degrees = math.degrees(yaw)
        self.yaw_received = True


    def normalize_angle_degrees(self, angle):
        """
        Normalizes an angle to be within the range [-180, 180).
        """
        while angle >= 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def control_loop_callback(self):
        """
        Main control loop callback.
        Calculates error, applies PID, and publishes Twist commands.
        """
        if not self.yaw_received:
            self.get_logger().warn('Waiting for IMU data...')
            return
        
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 
        
        if dt == 0: 
            return

        
        error = self.normalize_angle_degrees(self.target_yaw_degrees - self.current_yaw_degrees)
        
        
        self.integral_error += error * dt
        derivative_error = (error - self.last_error) / dt
        
        
        p_term = self.kp * error
        
        
        integral_limit = 100.0 
        self.integral_error = max(min(self.integral_error, integral_limit), -integral_limit)
        i_term = self.ki * self.integral_error
        
        d_term = self.kd * derivative_error
        
        
        
        
        angular_z = p_term + i_term + d_term
        
        
        max_angular_z = 3.0 
        angular_z = max(min(angular_z, max_angular_z), -max_angular_z)

        
        self.get_logger().info(
            'Yaw: %.2f deg | Target: %.2f deg | Error: %.2f deg | Ang_Z: %.2f rad/s' % (
                self.current_yaw_degrees, self.target_yaw_degrees, error, angular_z
            )
        )

        
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_x_speed
        twist_msg.angular.z = angular_z
        
        self.cmd_vel_publisher_.publish(twist_msg)
        
        
        self.last_error = error
        self.last_time = current_time


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
        
        
        
        for _ in range(5):
            try:
                self.publisher_.publish(stop_twist)
                time.sleep(0.02) 
            except Exception as e:
                self.get_logger().warn(f"Failed to publish stop command during shutdown: {e}")
                break 

        self.get_logger().info('Stop commands sent. Robot should be halted.')


def main(args=None):
    rclpy.init(args=args)
    yaw_controller_node = YawController()
    
    try:
        rclpy.spin(yaw_controller_node)
    except Exception as e:
        yaw_controller_node.get_logger().error(f"An unexpected error occurred during rclpy.spin(): {e}")
    finally:
        yaw_controller_node.get_logger().info('Node destruction initiated.')
        yaw_controller_node.destroy_node()
        
if __name__ == '__main__':
    main()
