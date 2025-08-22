# ~/test_wro0620/src/imu_test/scripts/calibrate_imu.py
import time
import board
import adafruit_bno055
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def main():
    print("Attempting to initialize BNO055 sensor...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        IMU_MODE = 0x08
        print(f"Setting sensor to IMU mode (Value: {IMU_MODE})...")
        sensor.mode = IMU_MODE
        time.sleep(0.1)

    except Exception as e:
        print(f"Error: Failed to initialize or set mode on the sensor: {e}")
        print("Please check I2C connection and device permissions.")
        exit()

    print("\nStarting calibration. Please follow the instructions.")
    print("Gyro: Keep the sensor completely still on a flat surface.")
    print("Accel: Slowly move the sensor to 6 different stable positions (each face pointing up/down).")

    while sensor.calibration_status[1] < 3 or sensor.calibration_status[2] < 3:
        status = sensor.calibration_status
        print(f"Calibration status: Gyro={status[1]}/3, Accel={status[2]}/3", end='\r')
        time.sleep(0.5)

    print("\n\nGyro and Accelerometer calibration COMPLETE!")
    print("-" * 40)

    try:
        calibration_data = {
            "accel_offset": sensor.offsets_accelerometer,
            "gyro_offset": sensor.offsets_gyroscope,
        }

        print("Successfully read calibration data from the sensor.")

        package_share_directory = get_package_share_directory('imu_test')
        
        # Save the calibration data to the config folder inside the package
        # This makes the path relative to this script's location
        config_dir = Path(package_share_directory) / "config"
        config_dir.mkdir(exist_ok=True)
        file_path = config_dir / "bno055_calibration.json"

        with open(file_path, "w") as f:
            json.dump(calibration_data, f, indent=4)
            
        print(f"\nSuccessfully saved calibration data to: {file_path}")

    except Exception as e:
        print(f"\nAn error occurred while reading or saving data: {e}")

if __name__ == "__main__":
    main()