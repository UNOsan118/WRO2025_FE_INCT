#!/usr/bin/python3
# coding=utf8
# Kinematics for a single-motor rear-drive Ackermann chassis.
import math
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class RearDriveAckermannChassis: # Renamed class for clarity
    def __init__(self, wheelbase=0.145, wheel_diameter=0.067, drive_motor_id=4): # Renamed parameter for clarity
        self.wheelbase = wheelbase
        self.wheel_diameter = wheel_diameter
        self.DRIVE_MOTOR_ID = drive_motor_id # Store the single drive motor ID

    def speed_covert(self, speed):
        """
        covert speed m/s to rps (revolutions per second)
        """
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_speed, angular_speed, reset_servo=True):
        servo_angle_pulse = 1500

        # --- Steering Angle Calculation (This logic remains the same) ---
        if abs(linear_speed) >= 1e-8 and abs(angular_speed) >= 1e-8:
            theta = math.atan(self.wheelbase * angular_speed / linear_speed)
            if abs(theta) > math.radians(29):
                theta = math.copysign(math.radians(29), theta) 
            servo_angle_pulse = 1500 + 2000 * math.degrees(-theta) / 180
        
        # --- Drive Motor Speed Calculation ---
        drive_motor_rps = -1 * self.speed_covert(linear_speed)
        
        # --- Create Motor Command Message for the single drive motor ---
        motor_data = []
        msg = MotorState()
        msg.id = self.DRIVE_MOTOR_ID  # Use the specified drive motor ID
        msg.rps = float(drive_motor_rps)
        motor_data.append(msg)
        
        motors_msg = MotorsState()
        motors_msg.data = motor_data
        
        if abs(linear_speed) < 1e-8:
            motors_msg.data[0].rps = 0.0
            return None, motors_msg
        else:
            return servo_angle_pulse, motors_msg