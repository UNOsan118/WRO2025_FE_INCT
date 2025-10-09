#!/usr/bin/env python3
# encoding: utf-8
# @Date: 2023/08/28
# Simplified stm32 ros2 package for WRO2025 project

import rclpy
import threading
import yaml
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import UInt16, Bool
from ros_robot_controller.ros_robot_controller_sdk import Board, PacketReportKeyEvents
from ros_robot_controller_msgs.srv import GetPWMServoState
from ros_robot_controller_msgs.msg import (
    ButtonState, BuzzerState, MotorsState, LedState,
    SetPWMServoState, PWMServoState
)

class RosRobotController(Node):

    def __init__(self, name):
        super().__init__(name)
        
        self.board = Board(logger=self.get_logger())
        self.board.enable_reception()

        self.declare_parameter('init_finish', False)

        # --- Publishers for required features ---
        self.button_pub = self.create_publisher(ButtonState, '~/button', 1)
        self.battery_pub = self.create_publisher(UInt16, '~/battery', 1)

        # --- Subscriptions for required features ---
        self.create_subscription(MotorsState, '~/set_motor', self.set_motor_state, 10)
        self.create_subscription(SetPWMServoState, '~/pwm_servo/set_state', self.set_pwm_servo_state, 10)
        self.create_subscription(Bool, '~/enable_reception', self.enable_reception, 1)
        # Optional: Keep Led and Buzzer if you might use them for debugging
        self.create_subscription(LedState, '~/set_led', self.set_led_state, 5)
        self.create_subscription(BuzzerState, '~/set_buzzer', self.set_buzzer_state, 5)
        
        # --- Services for required features ---
        self.create_service(GetPWMServoState, '~/pwm_servo/get_state', self.get_pwm_servo_state)

        self.load_servo_offsets()

        # Set initial motor speed to zero at startup
        self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self.clock = self.get_clock()

        # Timer for periodically publishing sensor data (battery, buttons)
        timer_period = 0.05  # 20Hz
        self.pub_timer = self.create_timer(timer_period, self.pub_callback)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('Simplified ros_robot_controller started')

    def load_servo_offsets(self):
        # This function remains unchanged, assuming servo offsets are still needed.
        config_path = '/home/ubuntu/software/Servo_upper_computer/servo_config.yaml'
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)

            if not isinstance(config, dict):
                self.get_logger().error(f"Invalid YAML config format: {config_path}")
                return

            for servo_id in range(1, 5):
                offset = config.get(servo_id, 0)
                try:
                    self.board.pwm_servo_set_offset(servo_id, offset)
                    self.get_logger().info(f"Set servo offset for ID {servo_id} to {offset}")
                except Exception as e:
                    self.get_logger().error(f"Failed to set offset for servo {servo_id}: {e}")

        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_path}")
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to load servo offsets: {e}")

    def get_node_state(self, request, response):
        response.success = True
        return response

    def pub_callback(self):
        # Simplified to only read necessary data
        if getattr(self, 'enable_reception', False):
            # No lock needed here as get_* methods are now thread-safe by being non-blocking
            button_data = self.board.get_button()
            battery_data = self.board.get_battery()

            if button_data is not None:
                self.pub_button_data(self.button_pub, button_data)
            
            if battery_data is not None:
                self.pub_battery_data(self.battery_pub, battery_data)

    def enable_reception(self, msg):
        self.get_logger().info(f"Setting reception to: {msg.data}")
        self.enable_reception = msg.data
        self.board.enable_reception(msg.data)

    def set_motor_state(self, msg):
        data = []
        for i in msg.data:
            data.extend([[i.id, i.rps]])
        self.board.set_motor_speed(data)

    def set_pwm_servo_state(self, msg):
        data = []
        for i in msg.state:
            if i.id and i.position:
                data.extend([[i.id[0], i.position[0]]])
            if i.id and i.offset:
                self.board.pwm_servo_set_offset(i.id[0], i.offset[0])

        if data:
            self.board.pwm_servo_set_position(msg.duration, data)
            
    # --- Optional methods (can be removed if not needed) ---
    def set_led_state(self, msg):
        # self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)
        pass # Commented out to avoid using it unless necessary

    def set_buzzer_state(self, msg):
        # self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)
        pass # Commented out to avoid using it unless necessary
        
    def get_pwm_servo_state(self, request, response):
        states = []
        for i in request.cmd:
            data = PWMServoState()
            if i.get_position:
                state = self.board.pwm_servo_read_position(i.id)
                if state is not None:
                    data.position = [state]
            if i.get_offset:
                state = self.board.pwm_servo_read_offset(i.id)
                if state is not None:
                    data.offset = [state]
            states.append(data)
        response.state = states
        response.success = True
        return response

    def pub_battery_data(self, pub, data):
        if data is not None:
            msg = UInt16()
            msg.data = data
            pub.publish(msg)

    def pub_button_data(self, pub, data):
        if data is not None:
            key_id, key_event = data
            state_map = {
                PacketReportKeyEvents.KEY_EVENT_PRESSED: 1,
                PacketReportKeyEvents.KEY_EVENT_LONGPRESS: 2,
                PacketReportKeyEvents.KEY_EVENT_LONGPRESS_REPEAT: 3,
                PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_LP: 4,
                PacketReportKeyEvents.KEY_EVENT_RELEASE_FROM_SP: 0,
                PacketReportKeyEvents.KEY_EVENT_CLICK: 5,
                PacketReportKeyEvents.KEY_EVENT_DOUBLE_CLICK: 6,
                PacketReportKeyEvents.KEY_EVENT_TRIPLE_CLICK: 7,
            }
            state = state_map.get(key_event, -1)

            if state != -1:
                msg = ButtonState()
                msg.id = key_id
                msg.state = state
                pub.publish(msg)
            else:
                self.get_logger().error(f"Unhandled button event: {key_event}")

def main():
    rclpy.init() 
    node = RosRobotController('ros_robot_controller')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        # Ensure motors are stopped on shutdown
        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
