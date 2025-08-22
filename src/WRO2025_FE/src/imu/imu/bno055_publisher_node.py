# ~/test_wro0620/src/imu_test/imu_test/bno055_publisher_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import adafruit_bno055
import json
import os

class Bno055Publisher(Node):
    def __init__(self):
        super().__init__('bno055_publisher_node')
        
        self.declare_parameter('calibration_file', 'ABSOLUTE_PATH_TO_YOUR_CALIBRATION_FILE.json')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)

        calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)

        try:
            i2c = board.I2C()
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            
            IMU_MODE = 0x08
            self.sensor.mode = IMU_MODE
            self.get_logger().info('BNO055 sensor found and set to IMU mode.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO055 sensor: {e}')
            rclpy.shutdown()
            return

        self.load_calibration(calibration_file)

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def load_calibration(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().warn(f'Calibration file not found at: {file_path}. Using factory defaults.')
            return

        try:
            with open(file_path, "r") as f:
                cal_data = json.load(f)
            
            self.sensor.offsets_accelerometer = tuple(cal_data["accel_offset"])
            self.sensor.offsets_gyroscope = tuple(cal_data["gyro_offset"])
            self.get_logger().info(f'Successfully loaded calibration data from {file_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load or set calibration data: {e}')

    def timer_callback(self):
        try:
            accel = self.sensor.acceleration
            gyro = self.sensor.gyro
            quat = self.sensor.quaternion

            if any(v is None for v in [accel, gyro, quat]):
                self.get_logger().warn('Incomplete sensor data received, skipping publish.')
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation.w = quat[0]
            msg.orientation.x = quat[1]
            msg.orientation.y = quat[2]
            msg.orientation.z = quat[3]
            msg.orientation_covariance[0] = -1.0

            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            msg.angular_velocity_covariance[0] = -1.0

            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            msg.linear_acceleration_covariance[0] = -1.0
            
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Could not read sensor data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Bno055Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()