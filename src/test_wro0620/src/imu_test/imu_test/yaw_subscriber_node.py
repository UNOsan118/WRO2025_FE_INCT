# ~/test_wro0620/src/imu_test/imu_test/yaw_subscriber_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class YawSubscriber(Node):
    def __init__(self):
        super().__init__('yaw_subscriber_node')
        
        self.subscription = self.create_subscription(
            Float64,
            'imu/yaw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Yaw subscriber node has been started and is waiting for messages...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Yaw: {msg.data:.2f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = YawSubscriber()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()