# ~/test_wro0620/src/imu_test/scripts/calibrate_imu.py
import time
import board
import adafruit_bno055
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def main():
    print("Attempting to initialize BNO055 sensor...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        
        # --- Use NDOF_MODE for full calibration ---
        NDOF_MODE = 0x0C
        print(f"Setting sensor to NDOF mode (Value: {NDOF_MODE}) for full calibration...")
        sensor.mode = NDOF_MODE
        time.sleep(0.1)
    except Exception as e:
        print(f"Error: Failed to initialize or set mode on the sensor: {e}")
        print("Please check I2C connection and device permissions.")
        exit()

    print("\nStarting full calibration. Please follow the instructions.")
    print("1. Gyro: Keep the sensor completely still on a flat surface.")
    print("2. Accel: Slowly move the sensor to 6 different stable positions.")
    print("3. Mag: Move the sensor in a large, slow figure-8 pattern in the air.")

    while not all(s == 3 for s in sensor.calibration_status):
        status = sensor.calibration_status
        print(f"Cal Status: Sys={status[0]}/3, Gyro={status[1]}/3, Accel={status[2]}/3, Mag={status[3]}/3", end='\r')
        time.sleep(0.2)

    print("\n\nALL SENSORS FULLY CALIBRATED!")
    print("-" * 40)

    try:
        # --- Read ALL calibration offsets ---
        calibration_data = {
            "accel_offset": sensor.offsets_accelerometer,
            "gyro_offset": sensor.offsets_gyroscope,
            "mag_offset": sensor.offsets_magnetometer,
            "accel_radius": sensor.radius_accelerometer,
            "mag_radius": sensor.radius_magnetometer
        }

        print("Successfully read calibration data from the sensor.")

        config_dir = Path(get_package_share_directory('imu')) / "config"
        config_dir.mkdir(exist_ok=True)
        file_path = config_dir / "bno055_full_calibration.json"

        with open(file_path, "w") as f:
            json.dump(calibration_data, f, indent=4)
        print(f"\nSuccessfully saved FULL calibration data to: {file_path}")

    except Exception as e:
        print(f"\nAn error occurred: {e}")

if __name__ == "__main__":
    main()