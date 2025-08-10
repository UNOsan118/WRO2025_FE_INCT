# ~/test_wro0620/src/imu_test/imu_test/yaw_publisher_node.py (Degree output version)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import board
import adafruit_bno055
import json
import os
import math
import time

class YawPublisher(Node):
    def __init__(self):
        super().__init__('yaw_publisher_node')
        
        self.declare_parameter('calibration_file', 'ABSOLUTE_PATH_TO_YOUR_CALIBRATION_FILE.json')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('zero_on_start', True)

        calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.zero_on_start = self.get_parameter('zero_on_start').get_parameter_value().bool_value
        
        self.publisher_ = self.create_publisher(Float64, 'imu/yaw', 10)

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

        self.initial_yaw_rad = 0.0
        if self.zero_on_start:
            self.set_initial_yaw()

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def quaternion_to_yaw_rad(self, quaternion):
        if quaternion is None or any(v is None for v in quaternion): return None
        w, x, y, z = quaternion
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def set_initial_yaw(self):
        self.get_logger().info("Setting initial angle as zero reference...")
        initial_yaw = None
        for _ in range(10): 
            quat = self.sensor.quaternion
            initial_yaw = self.quaternion_to_yaw_rad(quat)
            if initial_yaw is not None:
                break
            time.sleep(0.1)
        
        if initial_yaw is None:
            self.get_logger().warn("Could not read initial yaw. Defaulting to 0.")
            self.initial_yaw_rad = 0.0
        else:
            self.initial_yaw_rad = initial_yaw
            self.get_logger().info(f"Initial angle set. Reference Yaw: {math.degrees(self.initial_yaw_rad):.2f} degrees")

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
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f'Failed to load or set calibration data: {e}')

    def timer_callback(self):
        try:
            quat = self.sensor.quaternion
            current_yaw_rad = self.quaternion_to_yaw_rad(quat)

            if current_yaw_rad is not None:
                relative_yaw_rad = current_yaw_rad - self.initial_yaw_rad
                
                if relative_yaw_rad > math.pi:
                    relative_yaw_rad -= 2 * math.pi
                elif relative_yaw_rad < -math.pi:
                    relative_yaw_rad += 2 * math.pi


                relative_yaw_deg = math.degrees(relative_yaw_rad)

                msg = Float64()
                msg.data = relative_yaw_deg
                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Could not read sensor data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()