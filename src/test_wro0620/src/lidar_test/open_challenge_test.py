import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math

class FuzzyProControllerNode(Node):
    """
    Controls the robot using Fuzzy Logic to blend wall following and obstacle avoidance.
    """
    def __init__(self):
        super().__init__('fuzzy_pro_controller_node')

        # --- Parameters ---
        self.forward_speed = 0.2
        self.max_steer = 0.8 # Max steering angle in radians

        self.gain = 2

        # --- ADDED: Variable to store the last known good steering angle ---
        self.last_valid_steer = 0.0

        # --- Fuzzy Logic Control System Setup ---
        self.steering_ctrl = self.setup_fuzzy_system()

        # Communications
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            qos_profile)
        
        rclpy.get_default_context().on_shutdown(self.shutdown_callback)
        self.get_logger().info('Fuzzy Controller node started.')

    def setup_fuzzy_system(self):
        """
        Defines the fuzzy logic system: inputs, outputs, membership functions, and rules.
        """
        # --- Define Inputs (Antecedents) ---
        # Input 1: Distance from the front wall
        front_dist = ctrl.Antecedent(np.arange(0, 3.0, 0.05), 'front_dist')
        # Input 2: dist from the left wall (target_dist - current_dist)
        left_dist = ctrl.Antecedent(np.arange(0, 1.41, 0.01), 'left_dist')

        right_dist = ctrl.Antecedent(np.arange(0, 1.41, 0.1), 'right_dist')

        # --- Define Output (Consequent) ---
        # Output: Steering angle
        steering = ctrl.Consequent(np.arange(-1.0, 1.01, 0.05), 'steering')

        # --- Define Membership Functions (What does "close", "good", "far" mean?) ---
        front_dist['close'] = fuzz.trapmf(front_dist.universe, [0, 0, 1.0, 1.2])
        front_dist['far'] = fuzz.trapmf(front_dist.universe, [1.0, 1.2, 3.0, 3.0])

        left_dist['too_close'] = fuzz.trapmf(left_dist.universe, [0, 0, 0.15, 0.3])
        left_dist['good']      = fuzz.trimf(left_dist.universe, [0.15, 0.3, 0.4])
        left_dist['too_far']   = fuzz.trimf(left_dist.universe, [0.3, 1.41, 1.41])

        right_dist['too_close'] = fuzz.trapmf(right_dist.universe, [0, 0, 0.2, 0.3])
        right_dist['good']      = fuzz.trimf(right_dist.universe, [0.2, 0.3, 0.4])
        right_dist['too_far']   = fuzz.trimf(right_dist.universe, [0.3, 1.41, 1.41])

        steering['sharp_right'] = fuzz.trimf(steering.universe, [-1.0, -0.8, -0.6])
        steering['right'] = fuzz.trimf(steering.universe, [-1.0, -0.5, -0.3])
        steering['straight'] = fuzz.trimf(steering.universe, [-0.2, 0, 0.2])
        steering['left'] = fuzz.trimf(steering.universe, [0.3, 0.5, 1.0])
        steering['sharp_left'] = fuzz.trimf(steering.universe, [0.6, 0.8, 1.0])

        # --- Define Fuzzy Rules (The "IF-THEN" logic) ---
        # Rule 1: If front is close, then turn sharp right (highest priority)
        rule1 = ctrl.Rule(front_dist['close'], steering['sharp_right'])
        rule1.weight = 1.0

        # Rule 2: If front is far AND left is too far, then turn left.
        rule2 = ctrl.Rule(front_dist['far'] & left_dist['too_far'], steering['left'])
        rule2.weight = 0.6
        
        # Rule 3: If front is far AND left is too close, then turn right.
        rule3 = ctrl.Rule(front_dist['far'] & left_dist['too_close'], steering['right'])
        #rule3.weight = 0.8

        # Rule 4: If front is far AND left is good, then go straight.
        rule4 = ctrl.Rule(front_dist['far'] & left_dist['good'], steering['straight'])
        #rule4.weight = 0.6

        rule5 = ctrl.Rule(right_dist['too_close'], steering['sharp_left'])

        # --- Create and return the Control System ---
        steering_control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        return ctrl.ControlSystemSimulation(steering_control_system)


    def scan_callback(self, msg):
        """Main logic loop, executed on each LIDAR scan."""
        if not msg.ranges: return

        # --- Data Extraction ---
        front_distance = msg.ranges[0]
        # if math.isinf(front_distance): front_distance = 2.0

        num_points = len(msg.ranges)

        # left_index = num_points // 4
        left_index = (num_points * 3) // 4
        left_distance = msg.ranges[left_index]
        # if math.isinf(left_distance): left_distance = 10.0

        last_right = 10

        # --- Check for invalid sensor readings (nan or inf) ---
        # If front or left distance is invalid, maintain last steering and exit.
        if (
                math.isnan(front_distance) or math.isinf(front_distance) or 
                math.isnan(left_distance) or math.isinf(left_distance) 
            ):
            
            self.get_logger().warn('Invalid LIDAR data received (nan/inf). Maintaining last steer angle.', throttle_duration_sec=1.0)
            
            # Publish the last known good command
            twist_msg = Twist()
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = self.last_valid_steer # Use the stored value
            self.publisher_.publish(twist_msg)
            return # Exit the callback early
        
        # right_index = (num_points * 3) // 4
        right_index = num_points // 4
        right_distance = msg.ranges[right_index]
        if math.isnan(right_distance) or math.isinf(right_distance): right_distance = last_right
        else: last_right = right_distance

        # --- Fuzzy Logic Calculation ---
        # 1. Provide inputs to the fuzzy system
        self.steering_ctrl.input['front_dist'] = front_distance
        self.steering_ctrl.input['left_dist'] = left_distance
        self.steering_ctrl.input['right_dist'] = right_distance
        
        # 2. Compute the result
        self.steering_ctrl.compute()
        
        # 3. Get the crisp output value
        final_steer = self.steering_ctrl.output['steering']
        
        # Clamp the output to the robot's physical limits
        final_steer = max(min(final_steer, self.max_steer), -self.max_steer)

        # ---  Store the newly calculated steer angle as the last valid one ---
        self.last_valid_steer = final_steer

        # --- Publish Command ---
        twist_msg = Twist()
        twist_msg.linear.x = self.forward_speed * self.gain
        twist_msg.angular.z = final_steer * self.gain
        self.publisher_.publish(twist_msg)

        self.get_logger().info(
            f'[FUZZY] Front: {front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m | '
            f'Final Steer: {final_steer:.2f}',
            throttle_duration_sec=0.2
        )

    def shutdown_callback(self):
        self.publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = FuzzyProControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()