# ~/test_wro0620/src/imu_test/scripts/calibrate_imu.py (High-Precision Version)
import time
import board
import adafruit_bno055
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import sys

def wait_for_enter(prompt):
    """Waits for the user to press Enter to continue."""
    input(prompt)

def print_status(sensor):
    """Prints the calibration status in a more readable format."""
    sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status
    print(
        f"CALIBRATION STATUS: "
        f"System={sys_cal}/3 | "
        f"Gyro={gyro_cal}/3 | "
        f"Accel={accel_cal}/3 | "
        f"Mag={mag_cal}/3"
    )

def main():
    print("=" * 50)
    print(" BNO055 High-Precision Calibration Utility ".center(50, "="))
    print("=" * 50)

    print("\nAttempting to initialize BNO055 sensor...")
    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Start in CONFIG_MODE to ensure we can write calibration data
        CONFIG_MODE = 0x00
        sensor.mode = CONFIG_MODE
        time.sleep(0.1)
        
        # Switch to NDOF mode for the calibration process
        NDOF_MODE = 0x0C
        sensor.mode = NDOF_MODE
        time.sleep(0.1)
        print("Sensor initialized and set to NDOF mode for calibration.")

    except Exception as e:
        print(f"\n[ERROR] Failed to initialize the sensor: {e}")
        print("Please check I2C connection and device permissions.")
        exit()

    # --- Step 1: Gyroscope Calibration ---
    print("\n" + "-" * 50)
    print("STEP 1: Gyroscope Calibration")
    print("-" * 50)
    print("Place the sensor on a flat, completely stationary surface.")
    wait_for_enter("Press Enter when ready...")

    print("Calibrating gyroscope... please do not move the sensor.")
    while sensor.calibration_status[1] < 3: # Index 1 is Gyro
        print_status(sensor)
        time.sleep(0.5)
    print("\nGyroscope calibration COMPLETE!")
    print_status(sensor)

    # --- Step 2: Accelerometer Calibration ---
    print("\n" + "-" * 50)
    print("STEP 2: Accelerometer Calibration")
    print("-" * 50)
    print("You will need to place the sensor in 6 different stable positions.")
    print("Example positions: flat, upside down, on each of its 4 sides.")
    print("Hold each position for a few seconds until the 'Accel' status improves.")
    wait_for_enter("Press Enter to begin...")

    print("Calibrating accelerometer... please start moving the sensor slowly.")
    while sensor.calibration_status[2] < 3: # Index 2 is Accel
        print_status(sensor)
        time.sleep(0.5)
    print("\nAccelerometer calibration COMPLETE!")
    print_status(sensor)
    
    # --- Step 3 (Optional but Recommended): Magnetometer Calibration ---
    print("\n" + "-" * 50)
    print("STEP 3: Magnetometer Calibration (Recommended)")
    print("-" * 50)
    print("To calibrate the magnetometer, you need to make slow figure-8 motions in the air.")
    print("Try to cover all orientations (roll, pitch, yaw).")
    print("Keep the sensor away from large metal objects or magnetic fields.")
    wait_for_enter("Press Enter to begin...")

    print("Calibrating magnetometer... please start making figure-8 motions.")
    while sensor.calibration_status[3] < 3: # Index 3 is Mag
        print_status(sensor)
        time.sleep(0.5)
    print("\nMagnetometer calibration COMPLETE!")
    print_status(sensor)

    print("\n" + "=" * 50)
    print(" ALL CALIBRATION STEPS COMPLETE! ".center(50, "="))
    print("=" * 50)
    
    # --- Step 4: Saving Calibration Data ---
    try:
        # Switch to CONFIG_MODE to read offset and radius data
        sensor.mode = CONFIG_MODE
        time.sleep(0.1)

        # Read all relevant calibration parameters
        calibration_data = {
            "accel_offset": list(sensor.offsets_accelerometer),
            "gyro_offset": list(sensor.offsets_gyroscope),
            "mag_offset": list(sensor.offsets_magnetometer),
            "accel_radius": sensor.radius_accelerometer,
            "mag_radius": sensor.radius_magnetometer,
        }

        print("\nSuccessfully read all calibration data from the sensor:")
        print(json.dumps(calibration_data, indent=4))

        package_share_directory = get_package_share_directory('imu_test')
        config_dir = Path(package_share_directory) / "config"
        config_dir.mkdir(exist_ok=True)
        file_path = config_dir / "bno055_calibration.json"

        with open(file_path, "w") as f:
            json.dump(calibration_data, f, indent=4)
            
        print(f"\n[SUCCESS] Saved calibration data to: {file_path}")

    except Exception as e:
        print(f"\n[ERROR] An error occurred while reading or saving data: {e}")
    
    finally:
        # Return to IMU mode after finishing
        try:
            IMU_MODE = 0x08
            sensor.mode = IMU_MODE
            print("\nSensor has been returned to IMU mode.")
        except Exception as e:
            print(f"\n[WARN] Could not return sensor to IMU mode: {e}")

if __name__ == "__main__":
    main()