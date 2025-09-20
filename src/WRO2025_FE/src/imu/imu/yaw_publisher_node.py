# ~/test_wro0620/src/imu_test/imu_test/yaw_publisher_node.py (IMU_MODE Optimized Version)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
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
        self.declare_parameter('calibration_file', 'bno055_full_calibration.json')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0) # Higher rate can provide smoother data
        self.declare_parameter('zero_on_start', True)
        self.declare_parameter('initial_yaw_avg_count', 20) # Number of samples to average for initial yaw

        calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.zero_on_start = self.get_parameter('zero_on_start').get_parameter_value().bool_value
        self.initial_yaw_avg_count = self.get_parameter('initial_yaw_avg_count').get_parameter_value().integer_value
        
        # --- Publishers ---
        self.yaw_publisher_ = self.create_publisher(Float64, 'imu/yaw', 10)
        self.imu_data_publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        # --- Instance variables for mode switching ---
        self.first_run = True
        self.cal_data = None
        self.initial_yaw_offset_rad = 0.0
        self.ndof_correction_offset_rad = 0.0
        
        # --- Sensor Initialization (Starts in IMU_MODE) ---
        max_retries = 5
        retry_delay_sec = 1.0
        for attempt in range(max_retries):
            try:
                i2c = board.I2C()
                self.sensor = adafruit_bno055.BNO055_I2C(i2c)
                IMU_MODE = 0x08
                self.sensor.mode = IMU_MODE
                self.get_logger().info('BNO055 sensor found and set to IMU mode (0x08).')
                time.sleep(0.1) 
                break
            except Exception as e:
                self.get_logger().warn(f'Attempt {attempt + 1}/{max_retries} failed to initialize BNO055 sensor: {e}')
                if attempt + 1 == max_retries:
                    self.get_logger().error('Could not initialize BNO055 sensor after multiple retries. Shutting down.')
                    rclpy.shutdown()
                    return
                time.sleep(retry_delay_sec)

        # --- Load FULL calibration data but only apply Gyro/Accel initially ---
        self.load_calibration(calibration_file)
        if self.cal_data:
            try:
                self.sensor.offsets_accelerometer = tuple(self.cal_data["accel_offset"])
                self.sensor.offsets_gyroscope = tuple(self.cal_data["gyro_offset"])
                self.sensor.radius_accelerometer = self.cal_data.get("accel_radius", 1000)
                self.get_logger().info('Applied Gyro/Accel calibration.')
            except Exception as e:
                self.get_logger().error(f'Failed to apply initial calibration: {e}')

        # --- Set Initial Yaw Offset in IMU_MODE ---
        if self.zero_on_start:
            self.get_logger().info("Setting initial yaw offset in IMU_MODE...")
            self.initial_yaw_offset_rad = self.set_initial_yaw() 
            self.get_logger().info(f"Initial IMU_MODE offset set to: {math.degrees(self.initial_yaw_offset_rad):.2f} degrees")

        # --- Main Timer ---
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info(f"Yaw publisher started in IMU_MODE. Waiting for first movement to switch to NDOF.")


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
            avg_x = sum(math.cos(y) for y in yaw_samples)
            avg_y = sum(math.sin(y) for y in yaw_samples)
            avg_yaw_rad = math.atan2(avg_y, avg_x)
            self.get_logger().info(f"Averaged Raw Angle: {math.degrees(avg_yaw_rad):.2f} degrees")
            return avg_yaw_rad # Return the calculated average raw angle
        else:
            self.get_logger().error("Could not read initial yaw. Defaulting to 0.")
            return 0.0

    def load_calibration(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().warn(f'Calibration file not found: {file_path}. Using factory defaults.')
            return
        try:
            with open(file_path, "r") as f:
                cal_data = json.load(f)
            
            # Apply all calibration data
            self.sensor.offsets_accelerometer = tuple(cal_data["accel_offset"])
            self.sensor.offsets_gyroscope = tuple(cal_data["gyro_offset"])
            self.sensor.offsets_magnetometer = tuple(cal_data["mag_offset"])
            self.sensor.radius_accelerometer = cal_data.get("accel_radius", 1000)
            self.sensor.radius_magnetometer = cal_data.get("mag_radius", 480)
            
            self.get_logger().info(f'Successfully loaded and applied FULL calibration data from {file_path}')
            time.sleep(0.2) # Give a bit more time for offsets to be applied
        except Exception as e:
            self.get_logger().error(f'Failed to load or apply calibration data: {e}')

    def wait_for_calibration_stability(self):
        self.get_logger().info("Waiting for sensor to stabilize after loading calibration...")
        stabilization_timeout_sec = 10.0
        start_time = time.time()
        
        while time.time() - start_time < stabilization_timeout_sec:
            if self.sensor.calibrated:
                cal_status = self.sensor.calibration_status
                self.get_logger().info(f"Sensor stabilized! Cal Status: Sys={cal_status[0]}, Gyro={cal_status[1]}, Accel={cal_status[2]}, Mag={cal_status[3]}")
                return
            time.sleep(0.5)
        
        self.get_logger().warn("Stabilization timeout reached. Yaw may be inaccurate.")

    def timer_callback(self):
        try:
            # --- Switch to NDOF mode on the first run ---
            if self.first_run:
                self.first_run = False
                self.get_logger().info("First callback triggered, switching to NDOF mode...")
                try:
                    # Capture the raw yaw angle in IMU_MODE right before switching
                    imu_mode_raw_angle = self.quaternion_to_yaw_rad(self.sensor.quaternion)
                    if imu_mode_raw_angle is None: imu_mode_raw_angle = 0.0

                    # Switch mode
                    NDOF_MODE = 0x0C
                    self.sensor.mode = NDOF_MODE
                    time.sleep(0.1)
                    self.get_logger().info('Switched to NDOF mode (0x0C).')
                    time.sleep(0.1)

                    # Apply magnetometer calibration data now
                    if self.cal_data and "mag_offset" in self.cal_data:
                        self.sensor.offsets_magnetometer = tuple(self.cal_data["mag_offset"])
                        self.sensor.radius_magnetometer = self.cal_data.get("mag_radius", 480)
                        self.get_logger().info('Applied Magnetometer calibration.')
                    
                    # Capture the raw yaw angle in NDOF_MODE right after switching
                    ndof_mode_raw_angle = self.quaternion_to_yaw_rad(self.sensor.quaternion)
                    if ndof_mode_raw_angle is None: ndof_mode_raw_angle = 0.0

                    # The correction is the difference between the two modes' raw readings
                    self.ndof_correction_offset_rad = ndof_mode_raw_angle - imu_mode_raw_angle
                    self.get_logger().info(f"NDOF Correction Offset set to: {math.degrees(self.ndof_correction_offset_rad):.2f} degrees")

                except Exception as e:
                    self.get_logger().error(f'Failed to switch to NDOF mode or apply mag cal: {e}')
            
            # Read all required data from the sensor at once
            quat = self.sensor.quaternion
            gyro = self.sensor.gyro # Returns (x, y, z) in rad/s
            accel = self.sensor.acceleration # Returns (x, y, z) in m/s^2

            # --- Publish /imu/yaw (existing logic) ---
            current_yaw_rad = self.quaternion_to_yaw_rad(quat)
            if current_yaw_rad is not None:
                # --- SIMPLIFIED: Use simple, normalized subtraction ---
                diff_rad = current_yaw_rad - self.initial_yaw_offset_rad
                relative_yaw_rad = math.atan2(math.sin(diff_rad), math.cos(diff_rad))


                # Convert to degrees for publishing
                relative_yaw_deg = math.degrees(relative_yaw_rad)
                
                # For debugging the initial offset issue
                self.get_logger().debug(f"Current Raw: {math.degrees(current_yaw_rad):.2f}, Offset: {math.degrees(self.initial_yaw_offset_rad):.2f}, Relative Deg: {relative_yaw_deg:.2f}", throttle_duration_sec=1.0)

                yaw_msg = Float64()
                yaw_msg.data = relative_yaw_deg
                self.yaw_publisher_.publish(yaw_msg)

            # --- Publish /imu/data ---
            if all(v is not None for v in quat) and \
               all(v is not None for v in gyro) and \
               all(v is not None for v in accel):
                
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_id

                # Orientation (already a w,x,y,z tuple)
                imu_msg.orientation.w = quat[0]
                imu_msg.orientation.x = quat[1]
                imu_msg.orientation.y = quat[2]
                imu_msg.orientation.z = quat[3]
                imu_msg.orientation_covariance[0] = 999.0 # roll
                imu_msg.orientation_covariance[4] = 999.0 # pitch
                imu_msg.orientation_covariance[8] = 0.01  # yaw

                # Angular Velocity (already in rad/s)
                imu_msg.angular_velocity.x = gyro[0]
                imu_msg.angular_velocity.y = gyro[1]
                imu_msg.angular_velocity.z = gyro[2]
                imu_msg.angular_velocity_covariance[0] = 999.0
                imu_msg.angular_velocity_covariance[4] = 999.0
                imu_msg.angular_velocity_covariance[8] = 0.0025

                # Linear Acceleration (already in m/s^2)
                imu_msg.linear_acceleration.x = accel[0]
                imu_msg.linear_acceleration.y = accel[1]
                imu_msg.linear_acceleration.z = accel[2]
                
                self.imu_data_publisher_.publish(imu_msg)

        except Exception as e:
            # Import traceback for detailed error logging
            import traceback
            self.get_logger().error(
                f'An error occurred in timer_callback: {e}\n{traceback.format_exc()}',
                throttle_duration_sec=5.0
            )

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
