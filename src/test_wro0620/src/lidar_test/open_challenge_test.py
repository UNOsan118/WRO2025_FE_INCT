import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math

class BidirectionalFuzzyControllerNode(Node):
    """
    Controls the robot using Fuzzy Logic.
    Supports both clockwise (right wall as inner) and
    counter-clockwise (left wall as inner) wall following.
    """
    def __init__(self):
        super().__init__('bidirectional_fuzzy_controller_node')

        # --- Parameters ---
        self.forward_speed = 0.2
        self.max_steer = 0.8
        self.gain = 2.0
        
        # --- MODIFIED: Direction is now set directly here ---
        # Change this value to 'cw' or 'ccw' to switch direction.
        # 'cw': Clockwise, follows the LEFT wall as the outer wall.
        # 'ccw': Counter-clockwise, follows the RIGHT wall as the outer wall.
        self.direction = 'ccw' 

        # --- MODIFIED: Target distance from the OUTER wall ---

        self.last_valid_steer = 0.0
        self.steering_ctrl = self.setup_fuzzy_system()

        self.last_inner = 10

        # Communications
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            qos_profile)
        
        rclpy.get_default_context().on_shutdown(self.shutdown_callback)
        self.get_logger().info(f'Bidirectional Fuzzy Controller started in "{self.direction.upper()}" mode.')

# (in BidirectionalFuzzyControllerNode class)

    def setup_fuzzy_system(self):
        """
        Defines the fuzzy logic system using front, inner, and outer wall distances.
        """
        # --- Define Inputs (Antecedents) ---
        front_dist = ctrl.Antecedent(np.arange(0, 3.0, 0.05), 'front_dist')
        outer_wall_dist = ctrl.Antecedent(np.arange(0, 1.0, 0.05), 'outer_wall_dist')
        inner_wall_dist = ctrl.Antecedent(np.arange(0, 1.0, 0.05), 'inner_wall_dist')

        # --- Define Output (Consequent) ---
        steering = ctrl.Consequent(np.arange(-1.0, 1.01, 0.05), 'steering')

        # --- Define Membership Functions ---
        front_dist['close'] = fuzz.trapmf(front_dist.universe, [0, 0, 1.0, 1.2])
        front_dist['far']   = fuzz.trapmf(front_dist.universe, [1.0, 1.2, 3.0, 3.0])

        outer_wall_dist['too_close'] = fuzz.trimf(outer_wall_dist.universe, [0, 0, 0.25])
        outer_wall_dist['good']      = fuzz.trimf(outer_wall_dist.universe, [0.2, 0.3, 0.4])
        outer_wall_dist['too_far']   = fuzz.trimf(outer_wall_dist.universe, [0.35, 1.0, 1.0])
        
        inner_wall_dist['close'] = fuzz.trapmf(inner_wall_dist.universe, [0, 0, 0.2, 0.4])
        
        steering['turn_inner_sharp'] = fuzz.trimf(steering.universe, [-1.0, -0.8, -0.6])
        steering['turn_inner']       = fuzz.trimf(steering.universe, [-0.7, -0.4, -0.1])
        steering['straight']         = fuzz.trimf(steering.universe, [-0.2, 0, 0.2])
        steering['turn_outer']       = fuzz.trimf(steering.universe, [0.1, 0.4, 0.7])
        steering['turn_outer_sharp'] = fuzz.trimf(steering.universe, [0.6, 0.8, 1.0])
        
        # --- Define Fuzzy Rules ---
        # Priority rules (these override others)
        rule1 = ctrl.Rule(front_dist['close'], steering['turn_inner_sharp'])
        rule2 = ctrl.Rule(inner_wall_dist['close'], steering['turn_outer_sharp'])
        
        # Default wall following rules
        rule3 = ctrl.Rule(front_dist['far'] & outer_wall_dist['too_far'], steering['turn_outer'])
        rule4 = ctrl.Rule(front_dist['far'] & outer_wall_dist['too_close'], steering['turn_inner'])
        rule5 = ctrl.Rule(front_dist['far'] & outer_wall_dist['good'], steering['straight'])
        
        steering_control_system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        return ctrl.ControlSystemSimulation(steering_control_system)


    def scan_callback(self, msg):
        """Main logic loop, executed on each LIDAR scan."""
        if not msg.ranges: return

        # --- Data Extraction ---
        num_points = len(msg.ranges)
        front_distance = msg.ranges[0]
        left_scan_dist = msg.ranges[(num_points * 3) // 4]
        right_scan_dist = msg.ranges[num_points // 4]

        
        # --- MODIFIED: Map left/right scans to INNER and OUTER walls ---
        if self.direction == 'cw': # Clockwise
            # The outer wall is on the LEFT, the inner wall is on the RIGHT.
            outer_wall_dist = left_scan_dist
            inner_wall_dist = right_scan_dist
        else: # Counter-clockwise
            # The outer wall is on the RIGHT, the inner wall is on the LEFT.
            outer_wall_dist = right_scan_dist
            inner_wall_dist = left_scan_dist

        # --- Invalid Data Check ---
        if any(math.isnan(d) or math.isinf(d) for d in [front_distance, outer_wall_dist]):
            self.get_logger().warn('Invalid LIDAR data received (nan/inf). Maintaining last steer angle.', throttle_duration_sec=1.0)
            
            # Publish the last known good command
            twist_msg = Twist()
            twist_msg.linear.x = self.forward_speed
            twist_msg.angular.z = self.last_valid_steer # Use the stored value
            self.publisher_.publish(twist_msg)
            return
        
        if math.isnan(inner_wall_dist) or math.isinf(inner_wall_dist): inner_wall_dist = self.last_inner
        else: self.last_inner = inner_wall_dist

        # --- Fuzzy Logic Calculation ---
        self.steering_ctrl.input['front_dist'] = front_distance
        self.steering_ctrl.input['outer_wall_dist'] = outer_wall_dist
        self.steering_ctrl.input['inner_wall_dist'] = inner_wall_dist
        
        self.steering_ctrl.compute()
        
        final_steer = self.steering_ctrl.output['steering']
        
        # --- MODIFIED: Remap abstract steering to actual left/right steering ---
        # For 'cw' (left outer): 'turn_outer' (positive) means turning left (positive steer). Correct.
        # For 'ccw' (right outer): 'turn_outer' (positive) should mean turning right (negative steer). So, we invert.
        if self.direction == 'ccw':
            final_steer = -final_steer # Invert the steering output

        # Clamp, store, and publish
        final_steer = max(min(final_steer, self.max_steer), -self.max_steer)
        self.last_valid_steer = final_steer

        twist_msg = Twist()
        twist_msg.linear.x = self.forward_speed * self.gain
        twist_msg.angular.z = final_steer * self.gain
        self.publisher_.publish(twist_msg)

        self.get_logger().info(
            f'[{self.direction.upper()}] Front: {front_distance:.2f}m, Outer: {outer_wall_dist:.2f}m, Inner: {inner_wall_dist:.2f}m | Steer: {final_steer:.2f}',
            throttle_duration_sec=0.2
        )

    def shutdown_callback(self):
        self.publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = BidirectionalFuzzyControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()