#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import math
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from controller import ackermann, mecanum
from controller import ackermann_original
from ros_robot_controller_msgs.msg import MotorsState, SetPWMServoState, PWMServoState
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseWithCovarianceStamped, TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

ODOM_POSE_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]))

ODOM_POSE_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                             0, 1e-3, 1e-9, 0, 0, 0,
                             0, 0, 1e6, 0, 0, 0,
                             0, 0, 0, 1e6, 0, 0,
                             0, 0, 0, 0, 1e6, 0,
                             0, 0, 0, 0, 0, 1e-9]))

ODOM_TWIST_COVARIANCE = list(map(float, 
                        [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]))

ODOM_TWIST_COVARIANCE_STOP = list(map(float, 
                            [1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e-9]))

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def qua2rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

    return roll, pitch, yaw

class Controller(Node):
    
    def __init__(self, name):
        # rclpy.init()
        super().__init__(name)

        self.lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.pose_yaw = 0
        self.last_time = None
        self.current_time = None
        signal.signal(signal.SIGINT, self.shutdown)

        self.ackermann = ackermann.AckermannChassis(wheelbase=0.145, track_width=0.133, wheel_diameter=0.067)
        self.mecanum = mecanum.MecanumChassis(wheelbase=0.1368, track_width=0.1446, wheel_diameter=0.065)

        # Initialize the new model for the modified robot.
        # Ensure 'drive_motor_id' matches your hardware. Using 4 as a placeholder.
        self.ackermann_original = ackermann_original.RearDriveAckermannChassis(
            wheelbase=0.145, 
            wheel_diameter=0.067, 
            drive_motor_id=4
        )

        # Declare parameters
        self.declare_parameter('pub_odom_topic', True)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('linear_correction_factor', 1.00)
        self.declare_parameter('linear_correction_factor_tank', 0.52)
        self.declare_parameter('angular_correction_factor', 1.00)
        self.declare_parameter('machine_type', os.environ['MACHINE_TYPE'])
        
        self.pub_odom_topic = self.get_parameter('pub_odom_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        
        #self.machine_type = os.environ.get('MACHINE_TYPE', 'MentorPi_Mecanum')
        self.machine_type = self.get_parameter('machine_type').value
        if self.machine_type == 'JetRover_Tank':
            self.linear_factor = self.get_parameter('linear_correction_factor_tank').value
        else:
            self.linear_factor = self.get_parameter('linear_correction_factor').value
        self.angular_factor = self.get_parameter('angular_correction_factor').value

        self.clock = self.get_clock() 
        if self.pub_odom_topic:
            # self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)            # self.odom_trans = TransformStamped()
            # self.odom_trans.header.frame_id = self.odom_frame_id
            # self.odom_trans.child_frame_id = self.base_frame_id
            
            self.odom = Odometry()
            self.odom.header.frame_id = self.odom_frame_id
            self.odom.child_frame_id = self.base_frame_id
            
            self.odom.pose.covariance = ODOM_POSE_COVARIANCE
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
            
            self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 1)
            # Replace threading.Thread with this create_timer call
            odom_timer_period = 0.02 # 50Hz, same as time.sleep(0.02)
            self.odom_timer = self.create_timer(odom_timer_period, self.cal_odom_fun)

        self.get_logger().info('\033[1;32m%f %f\033[0m' % (self.linear_factor, self.angular_factor))
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 1)
        self.create_subscription(Pose2D, 'set_odom', self.set_odom, 1)
        # self.create_subscription(Twist, 'controller/cmd_vel', self.cmd_vel_callback, 1)
        qos_profile_cmd_vel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Add the qos_profile argument to the subscription
        self.create_subscription(
            Twist, 
            'controller/cmd_vel', 
            self.cmd_vel_callback, 
            qos_profile=qos_profile_cmd_vel
        )
        #self.create_subscription(Twist, '/app/cmd_vel', self.acker_cmd_vel_callback, 1)
        self.create_subscription(Twist, 'cmd_vel', self.app_cmd_vel_callback, 1)
        self.create_service(Trigger, 'controller/load_calibrate_param', self.load_calibrate_param)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        rclpy.shutdown()

    def load_calibrate_param(self, request, response):
        if self.machine_type == 'JetRover_Tank':
            self.linear_factor = self.get_parameter('~linear_correction_factor_tank').value or 0.52
        else:
            self.linear_factor = self.get_parameter('~linear_correction_factor').value or 1.00
        self.angular_factor = self.get_parameter('~angular_correction_factor').value or 1.00
        self.get_logger().info('\033[1;32m%s\033[0m' % 'load_calibrate_param')

        response.success = True
        return response

    def set_odom(self, msg):
        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.base_frame_id
        
        self.odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom.pose.pose.position.x = msg.x
        self.odom.pose.pose.position.y = msg.y
        self.pose_yaw = msg.theta
        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)
        
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.odom_frame_id
        pose.header.stamp = self.clock().now().to_msg()
        pose.pose.pose = self.odom.pose.pose
        pose.pose.covariance = ODOM_POSE_COVARIANCE
        self.pose_pub.publish(pose)

    def app_cmd_vel_callback(self, msg):
        if msg.linear.x > 0.2:
            msg.linear.x = 0.2
        if msg.linear.x < -0.2:
            msg.linear.x = -0.2
        if msg.linear.y > 0.2:
            msg.linear.y = 0.2
        if msg.linear.y < -0.2:
            msg.linear.y = -0.2
        if msg.angular.z > 0.5:
            msg.angular.z = 0.5
        if msg.angular.z < -0.5:
            msg.angular.z = -0.5
        self.cmd_vel_callback(msg)
    # def cmd_vel_callback(self, msg):
    #     if self.machine_type == 'MentorPi_Mecanum':
    #         self.linear_x = msg.linear.x
    #         self.linear_y = msg.linear.y
    #         self.angular_z = msg.angular.z
    #         speeds = self.mecanum.set_velocity(self.linear_x, self.linear_y, self.angular_z)
    #         self.motor_pub.publish(speeds)
    #     elif self.machine_type == 'MentorPi_Acker':
    #         self.linear_x = msg.linear.x
    #         if msg.angular.z != 0:
    #             r = self.linear_x / msg.angular.z
    #             self.angular_z = msg.angular.z
    #         else:
    #             self.angular_z = 0.0
    #         speeds = self.ackermann.set_velocity(self.linear_x, self.angular_z)
    #         self.motor_pub.publish(speeds[1])
    #         if speeds[0] is not None:
    #             servo_state = PWMServoState()
    #             servo_state.id = [3]
    #             servo_state.position = [int(speeds[0])]
    #             data = SetPWMServoState()
    #             data.state = [servo_state]
    #             data.duration = 0.02
    #             self.servo_state_pub.publish(data)
    def cmd_vel_callback(self, msg):
        # Step 1: Safely update shared variables for the odometry thread.
        with self.lock:
            self.linear_x = msg.linear.x
            self.linear_y = msg.linear.y if self.machine_type == 'MentorPi_Mecanum' else 0.0
            self.angular_z = msg.angular.z

        # Step 2: Calculate and publish motor/servo commands based on machine type.
        # This section uses the fresh 'msg' object for calculations.
        if self.machine_type == 'MentorPi_Mecanum':
            speeds = self.mecanum.set_velocity(msg.linear.x, msg.linear.y, msg.angular.z)
            self.motor_pub.publish(speeds)

        elif self.machine_type == 'MentorPi_Acker':
            # Calculate speeds using the standard Ackermann model
            speeds = self.ackermann.set_velocity(msg.linear.x, msg.angular.z)
            self.motor_pub.publish(speeds[1])
            
            if speeds[0] is not None:
                servo_state = PWMServoState()
                servo_state.id = [3]
                servo_state.position = [int(speeds[0])]
                data = SetPWMServoState()
                data.state = [servo_state]
                data.duration = 0.02
                self.servo_state_pub.publish(data)
        
        elif self.machine_type == 'MentorPi_Acker_Original':
            # Calculate speeds using the specific "Original" Ackermann model
            speeds = self.ackermann_original.set_velocity(msg.linear.x, msg.angular.z)
            self.motor_pub.publish(speeds[1])
            
            if speeds[0] is not None:
                servo_state = PWMServoState()
                servo_state.id = [3] 
                servo_state.position = [int(speeds[0])]
                data = SetPWMServoState()
                data.state = [servo_state]
                data.duration = 0.02
                self.servo_state_pub.publish(data)

    def cal_odom_fun(self):
        # Lock the section where shared variables are read
        with self.lock:
            # Copy shared values to local variables immediately after acquiring the lock
            linear_x = self.linear_x
            linear_y = self.linear_y
            angular_z = self.angular_z

            # Use ROS clock for time calculation, which is safer
            current_time = self.get_clock().now()
            if self.last_time is None:
                self.last_time = current_time
                dt = 0.0
            else:
                dt = (current_time - self.last_time).nanoseconds / 1e9
            
            self.last_time = current_time

            delta_x = linear_x * dt * math.cos(self.pose_yaw)
            delta_y = linear_y * dt * math.sin(self.pose_yaw) # Note: linear_y is often 0 for Ackermann
            delta_yaw = angular_z * dt

            self.x += delta_x
            self.y += delta_y
            self.pose_yaw += delta_yaw

            self.odom.header.stamp = current_time.to_msg()
            self.odom.pose.pose.position.x = self.linear_factor * self.x
            self.odom.pose.pose.position.y = self.linear_factor * self.y
            self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)
            
            # Use the local variables for twist
            self.odom.twist.twist.linear.x = linear_x
            self.odom.twist.twist.linear.y = linear_y
            self.odom.twist.twist.angular.z = angular_z

            if linear_x == 0 and linear_y == 0 and angular_z == 0:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
            else:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        
        # Publish outside the lock
        self.odom_pub.publish(self.odom)


def main():
    rclpy.init() # ADD THIS LINE AT THE TOP
    node = Controller('odom_publisher')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

