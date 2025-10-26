#!/usr/bin/python3
# coding=utf8
# Kinematics for a single-motor rear-drive Ackermann chassis.
import math
import numpy as np
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class RearDriveAckermannChassis: # Renamed class for clarity
    def __init__(self, wheelbase=0.127, wheel_diameter=0.067, drive_motor_id=4, servo_offset_pulse=0): # Renamed parameter for clarity servo_offset_pulse=43
        self.wheelbase = wheelbase
        self.wheel_diameter = wheel_diameter
        self.DRIVE_MOTOR_ID = drive_motor_id # Store the single drive motor ID
        # self.SERVO_OFFSET_PULSE = servo_offset_pulse

        # --- Steering Correction Lookup Table ---
        # Data is based on your measurements.
        # Format: [actual_tire_angle_rad, required_servo_pulse]
        # Pulse is calculated from your "theta (deg)" using the old formula:
        # pulse = 1500 + 2000 * (-theta_deg) / 180
        self.steering_lookup_table = [
            [-0.5201, 2333], # Tire: -29.8 deg, from old theta: -75 deg
            [-0.5114, 2278], # Tire: -29.3 deg, from old theta: -70 deg
            [-0.4765, 2167], # Tire: -27.3 deg, from old theta: -60 deg
            [-0.4451, 2056], # Tire: -25.5 deg, from old theta: -50 deg
            [-0.3892, 1944], # Tire: -22.3 deg, from old theta: -40 deg
            [-0.3211, 1833], # Tire: -18.4 deg, from old theta: -30 deg
            [-0.2496, 1722], # Tire: -14.3 deg, from old theta: -20 deg
            [-0.1187, 1611], # Tire: -6.8 deg, from old theta: -10 deg
            [ 0.0000, 1500], # Tire: 0.0 deg, from old theta: 0 deg
            [ 0.1344, 1389], # Tire: 7.7 deg, from old theta: 10 deg
            [ 0.2810, 1278], # Tire: 16.1 deg, from old theta: 20 deg
            [ 0.4328, 1167], # Tire: 24.8 deg, from old theta: 30 deg
            [ 0.5253, 1139], # Tire: 30.1 deg, from old theta: 32.5 deg
        ]

        # For easier interpolation, separate the table into two arrays
        self.lookup_angles_rad = [row[0] for row in self.steering_lookup_table]
        self.lookup_pulses = [row[1] for row in self.steering_lookup_table]

    def speed_covert(self, speed):
        """
        covert speed m/s to rps (revolutions per second)
        """
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_speed, angular_speed, reset_servo=True):
        # --- NEW: Explicitly handle near-zero speed at the beginning ---
        # This logic is adopted from the stable `ackermann.py` file to prevent unstable calculations.
        if abs(linear_speed) < 0.01: # Use a practical threshold, e.g., 1 cm/s
            msg = MotorState()
            msg.id = self.DRIVE_MOTOR_ID
            msg.rps = 0.0
            
            motors_msg = MotorsState()
            motors_msg.data = [msg]
            
            # Return None for servo angle to indicate no command should be sent.
            return None, motors_msg

        # --- The rest of the code now safely handles non-zero linear speed ---
        servo_angle_pulse = 1500 # + self.SERVO_OFFSET_PULSE 

        if abs(angular_speed) >= 1e-8:
            # 1. Calculate the IDEAL steering angle (theta) using the Ackermann model.
            target_theta_rad = math.atan(self.wheelbase * angular_speed / linear_speed)
            
            # 2. Limit the target angle to the measured physical maximums.
            max_left_angle_rad = self.lookup_angles_rad[0]
            max_right_angle_rad = self.lookup_angles_rad[-1]
            
            # Clamp the target angle within the achievable range
            if target_theta_rad < max_left_angle_rad:
                target_theta_rad = max_left_angle_rad
            elif target_theta_rad > max_right_angle_rad:
                target_theta_rad = max_right_angle_rad

            # 3. Use the lookup table and interpolation to find the required servo pulse.
            servo_angle_pulse = np.interp(target_theta_rad, self.lookup_angles_rad, self.lookup_pulses)
        
        # --- Drive Motor Speed Calculation ---
        drive_motor_rps = self.speed_covert(linear_speed) # * -1
        
        # --- Create Motor Command Message for the single drive motor ---
        motor_data = []
        msg = MotorState()
        msg.id = self.DRIVE_MOTOR_ID  # Use the specified drive motor ID
        msg.rps = float(drive_motor_rps)
        motor_data.append(msg)
        
        motors_msg = MotorsState()
        motors_msg.data = motor_data
        
        return servo_angle_pulse, motors_msg