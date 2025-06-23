import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from tf_transformations import euler_from_quaternion

class ImuSubscriber(Node):
    """
    A ROS2 node to subscribe to the /imu topic, convert quaternion to Euler angles (Yaw),
    and print both raw IMU data and the calculated Yaw angle.
    """
    def __init__(self):
        super().__init__('imu_subscriber_node')
        
        self.subscription = self.create_subscription(
            Imu,                     # Message type
            '/imu',                  # Topic name
            self.imu_callback,       # Callback function
            10                       # Queue size
        )
        self.subscription # Prevent unused variable warning
        
        self.get_logger().info('IMU Subscriber Node Started. Subscribing to /imu topic and printing raw data + Yaw.')

    def imu_callback(self, msg):
        """
        Callback function for received Imu messages.
        Prints raw IMU data and the calculated Yaw angle.
        """
        # Get the quaternion data from the message
        orientation_q = msg.orientation
        
        # Create a tuple from the quaternion components for conversion
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        # Convert quaternion to Euler angles (roll, pitch, yaw) in radians
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # Convert yaw from radians to degrees for easier understanding
        yaw_degrees = math.degrees(yaw)
        
        # Get angular velocity and linear acceleration
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        
        # Log all the data
        self.get_logger().info(
            '--- IMU Data Received ---\n'
            '  Orientation (Quaternion):\n'
            '    x: %f, y: %f, z: %f, w: %f\n'
            '  Yaw Angle: %f degrees\n\n' # Added Yaw Angle here
            '  Angular Velocity (rad/s):\n'
            '    x: %f, y: %f, z: %f\n'
            '  Linear Acceleration (m/s^2):\n'
            '    x: %f, y: %f, z: %f' % (
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w,
                yaw_degrees, # Pass yaw_degrees for printing
                angular_velocity.x, angular_velocity.y, angular_velocity.z,
                linear_acceleration.x, linear_acceleration.y, linear_acceleration.z
            )
        )

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        imu_subscriber.get_logger().info('IMU Subscriber Node interrupted and shutting down.')
    finally:
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
