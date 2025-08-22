# ~/test_wro0620/src/imu_test/imu_test/yaw_publisher_node.py (IMU_MODE Optimized Version)
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
        
        # --- Parameters ---
        self.declare_parameter('calibration_file', 'ABSOLUTE_PATH_TO_YOUR_CALIBRATION_FILE.json')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0) # Higher rate can provide smoother data
        self.declare_parameter('zero_on_start', True)
        self.declare_parameter('initial_yaw_avg_count', 20) # Number of samples to average for initial yaw

        calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.zero_on_start = self.get_parameter('zero_on_start').get_parameter_value().bool_value
        self.initial_yaw_avg_count = self.get_parameter('initial_yaw_avg_count').get_parameter_value().integer_value
        
        # --- Publisher ---
        self.publisher_ = self.create_publisher(Float64, 'imu/yaw', 10)

        # --- Sensor Initialization ---
        try:
            i2c = board.I2C()
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            # Use IMU_MODE which combines accelerometer and gyroscope without magnetometer
            IMU_MODE = 0x08
            self.sensor.mode = IMU_MODE
            self.get_logger().info('BNO055 sensor found and set to IMU mode (magnetometer disabled).')
            # It takes a moment for the sensor to switch modes
            time.sleep(0.1) 
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BNO055 sensor: {e}')
            rclpy.shutdown()
            return

        self.load_calibration(calibration_file)

        # --- Initial Yaw Offset ---
        self.initial_yaw_offset_rad = 0.0
        if self.zero_on_start:
            self.set_initial_yaw()

        # --- Main Timer ---
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(f"Yaw publisher started with a rate of {publish_rate} Hz.")

    def quaternion_to_yaw_rad(self, quaternion):
        """Converts a quaternion (w, x, y, z) to yaw in radians."""
        if quaternion is None or any(v is None for v in quaternion):
            self.get_logger().warn('Received invalid quaternion data.', throttle_duration_sec=5.0)
            return None
        w, x, y, z = quaternion
        
        # Standard formula for yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def set_initial_yaw(self):
        self.get_logger().info("Averaging initial angle for zero reference (please keep steady)...")
        yaw_samples = []
        
        # Wait for the sensor to provide stable readings
        time.sleep(0.5)

        for _ in range(self.initial_yaw_avg_count): 
            try:
                quat = self.sensor.quaternion
                yaw_rad = self.quaternion_to_yaw_rad(quat)
                if yaw_rad is not None:
                    yaw_samples.append(yaw_rad)
                time.sleep(0.02) # Sleep between samples
            except Exception:
                self.get_logger().warn('Failed to read quaternion during initialization, retrying...')
                time.sleep(0.1)
        
        if yaw_samples:
            # More robust averaging for angles (handling wraparound)
            sum_x = sum(math.cos(y) for y in yaw_samples)
            sum_y = sum(math.sin(y) for y in yaw_samples)
            self.initial_yaw_offset_rad = math.atan2(sum_y, sum_x)
            self.get_logger().info(f"Initial angle set. Reference Offset: {math.degrees(self.initial_yaw_offset_rad):.2f} degrees")
        else:
            self.get_logger().error("Could not read initial yaw after multiple attempts. Defaulting to 0.")
            self.initial_yaw_offset_rad = 0.0

    def load_calibration(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().warn(f'Calibration file not found at: {file_path}. Using factory defaults.')
            return
        try:
            with open(file_path, "r") as f:
                cal_data = json.load(f)

            # IMU_MODE only requires accelerometer and gyroscope calibration
            self.sensor.offsets_accelerometer = tuple(cal_data["accel_offset"])
            self.sensor.offsets_gyroscope = tuple(cal_data["gyro_offset"])
            self.sensor.radius_accelerometer = cal_data.get("accel_radius", 1000) # Use default if not present
            
            self.get_logger().info(f'Successfully loaded Accel/Gyro calibration data from {file_path}')
            # Wait for calibration data to be applied
            time.sleep(0.1) 
        except Exception as e:
            self.get_logger().error(f'Failed to load or set calibration data: {e}')

    def timer_callback(self):
        try:
            quat = self.sensor.quaternion
            current_yaw_rad = self.quaternion_to_yaw_rad(quat)

            if current_yaw_rad is not None:
                # Calculate the difference from the initial angle
                relative_yaw_rad = current_yaw_rad - self.initial_yaw_offset_rad
                
                # Normalize the angle to the range [-pi, pi] for clean output
                if relative_yaw_rad > math.pi:
                    relative_yaw_rad -= 2 * math.pi
                elif relative_yaw_rad < -math.pi:
                    relative_yaw_rad += 2 * math.pi

                # Convert to degrees for publishing
                relative_yaw_deg = math.degrees(relative_yaw_rad)

                msg = Float64()
                msg.data = relative_yaw_deg
                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Could not read sensor data: {e}', throttle_duration_sec=5.0)

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
