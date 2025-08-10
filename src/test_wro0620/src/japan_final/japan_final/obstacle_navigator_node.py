import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image # Add Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
import numpy as np
import math
from enum import Enum, auto
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError 
import os
from datetime import datetime

# (Enums are the same)
class State(Enum):
    DETERMINE_COURSE = auto() 
    FINISHED = auto()
    STRAIGHT = auto()
    TURNING = auto()

class DetermineCourseSubState(Enum):
    INITIALIZING_CAMERA = auto()
    DETECTING_OBSTACLE_COLOR = auto()
    PREPARE_TO_START = auto()
    DECIDE_INITIAL_PATH = auto()
    APPROACH_INITIAL_WALL = auto()
    DETECTING_STRAIGHT = auto()

class StraightSubState(Enum):
    ALIGN_WITH_OUTER_WALL = auto()
    ALIGN_WITH_INNER_WALL = auto()
    PRE_PLANNING_ADJUST = auto()
    PLAN_NEXT_AVOIDANCE = auto()
    POST_PLANNING_REVERSE = auto()
    AVOID_OUTER_TURN_IN = auto()
    AVOID_OUTER_PASS_THROUGH = auto()
    AVOID_INNER_TURN_IN = auto()
    AVOID_INNER_PASS_THROUGH = auto()

class TurningSubState(Enum):
    APPROACH = auto()
    REVERSE = auto()
    CREATING_SPACE = auto()
    CHECK_AND_FINISH = auto()
    FINALIZE_TURN = auto()

class ObstacleNavigatorNode(Node):
    def __init__(self):
        super().__init__('obstacle_navigator_node')

        self.declare_parameter('correct_mirrored_scan', True,)

        # --- State Machine Parameters ---
        self.state = State.DETERMINE_COURSE
        self.determine_course_sub_state = DetermineCourseSubState.INITIALIZING_CAMERA
        self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
        self.turning_sub_state = None 
        
        self.inner_wall_disappear_threshold = 2.0
        self.inner_wall_disappear_count = 3
        self.max_turns = 12

        # --- Camera Control Parameters ---
        self.declare_parameter('pan_servo_id', 1, )
        self.declare_parameter('tilt_servo_id', 2, )
        self.declare_parameter('initial_pan_position', 1850)
        self.declare_parameter('camera_move_duration_sec', 0.3)
        self.declare_parameter('planning_detection_threshold', 300)
        
        # --- Parameters for Dynamic Camera Aiming ---
        self.declare_parameter('tilt_position_cw', 500)
        self.declare_parameter('tilt_position_ccw', 2500)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('roi_planning_universal', [240, 0, 0, 480, 640, 480, 400, 0])
        self.declare_parameter('roi_planning_start_area', [320, 0, 80, 480, 640, 480, 400, 0])

        # --- Course Detection Parameters ---
        self.declare_parameter('course_detection_threshold_m', 1.5)
        self.declare_parameter('course_detection_slow_speed', 0.15)
        self.declare_parameter('course_detection_speed', 0.15)
        self.declare_parameter('initial_approach_target_dist_m', 0.25)
        self.declare_parameter('initial_approach_target_dist_start_area_m', 0.4)

        # --- Vision Processing Parameters (from initial_obstacle_detection_node) ---
        self.declare_parameter('red_lower1', [0, 100, 80])
        self.declare_parameter('red_upper1', [3, 255, 243])
        self.declare_parameter('red_lower2', [174, 100, 80])
        self.declare_parameter('red_upper2', [179, 255, 243])
        self.declare_parameter('green_lower', [59, 100, 67])
        self.declare_parameter('green_upper', [77, 255, 158])
        self.declare_parameter('roi_left', [150, 50, 90, 430])
        self.declare_parameter('roi_right', [290, 50, 90, 430])
        self.declare_parameter('detection_pixel_threshold', 500)
        self.declare_parameter('detection_samples', 10) # Number of frames to sample for majority vote

        # --- Turning Maneuver Parameters ---
        self.declare_parameter('turn_reverse_steer', 0.8)
        self.declare_parameter('kp_approach_angle', 0.02)
        self.declare_parameter('reverse_yaw_tolerance_deg', 5.0)

        # Fixed approach angles based on last path
        self.declare_parameter('turn_approach_angle_from_outer_path_deg', 35.0)
        self.declare_parameter('turn_approach_angle_from_inner_path_deg', -5.0)
        self.declare_parameter('turn_approach_angle_from_outer_path_and_start_area_deg', 27.0)
        self.declare_parameter('turn_approach_dist_from_outer_path', 0.25) # Target distance when approaching from an outer path
        self.declare_parameter('turn_approach_dist_from_inner_path', 0.1) # Target distance when approaching from an inner path
        self.declare_parameter('turn_approach_dist_from_outer_path_and_start_area_m', 0.18)

        self.declare_parameter('dynamic_approach_start_angle_deg', -60.0)
        self.declare_parameter('dynamic_approach_start_dist_m', 1.0)
        self.declare_parameter('dynamic_approach_straighten_offset_m', 0.17)

        # --- PARAMETERS for Yaw Correction Logic ---
        self.declare_parameter('yaw_correction_threshold_deg', 75.0)
        self.declare_parameter('yaw_correction_wide_dist', 1.5)

        # --- Driving Logic Parameters ---
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('max_steer', 0.8)
        self.declare_parameter('gain', 2.0)
        self.declare_parameter('direction', 'ccw')

        # Parameter for the required clearance distance from an obstacle
        self.declare_parameter('obstacle_clearance_dist_m', 0.67)

        # --- Parameters for Avoidance Control ---
        self.declare_parameter('avoid_target_outer_dist_m', 0.2)
        self.declare_parameter('avoid_kp_angle', 0.03) # P-gain for yaw control
        self.declare_parameter('avoid_kp_dist', 1.5)  # P-gain for distance control
        self.declare_parameter('avoid_speed', 0.2)
        self.declare_parameter('avoid_inner_pass_thresh_m', 0.6)

        # --- Parameters for Avoidance Sequence Control ---
        self.declare_parameter('avoid_turn_in_kp_angle', 0.04)
        self.declare_parameter('avoid_turn_in_yaw_tolerance_deg', 5.0)

        # --- Parameters for Simplified Avoidance Sequence ---
        self.declare_parameter('avoid_outer_approach_angle_deg', 65.0)
        self.declare_parameter('avoid_outer_approach_target_dist_m', 0.40)

        self.declare_parameter('avoid_outer_approach_target_dist_start_area_m', 0.6)

        self.declare_parameter('avoid_inner_approach_angle_deg', 65.0)
        self.declare_parameter('avoid_inner_approach_target_dist_m', 0.40)

        # self.declare_parameter('avoidance_path_plan', ['outer', 'inner_to_outer', 'outer_to_inner', 'inner'])
        self.declare_parameter('avoidance_path_plan', ['', '', '', ''])
        self.declare_parameter('entrance_obstacle_plan', [False, False, False, False])

        # --- Parameters for PID-based Wall Following ---
        self.declare_parameter('align_target_inner_dist_m', 0.2)
        self.declare_parameter('align_dist_tolerance_m', 0.02)
        self.declare_parameter('align_kp_angle', 0.03)
        self.declare_parameter('align_kp_dist', 8.0)
        self.declare_parameter('align_target_outer_dist_m', 0.2)
        self.declare_parameter('align_target_outer_dist_start_area_m', 0.45)

        self.declare_parameter('save_debug_images', True, )
        self.declare_parameter('debug_image_path', '/home/ubuntu/test_wro0620/src/japan_final/images', )

        self.declare_parameter('log_level', 'DEBUG') # Options: 'DEBUG', 'INFO', 'WARN', 'ERROR'

        # Load all parameters
        self._load_parameters()

        # --- Internal State Variables ---
        self.inner_wall_far_counter = 0
        self.turn_count = 0
        self.wall_segment_index = 0
        self.approach_target_angle_deg = 0.0
        self.approach_target_dist = 0.0
        self.approach_base_yaw_deg = 0.0
        self.reverse_target_yaw_deg = 0.0
        self.last_valid_steer = 0.0
        self.last_inner = 10.0
        self.current_yaw_deg = 0.0

        self.latest_scan_msg = None
        self.is_passing_obstacle = False
        self.avoid_target_yaw_deg = 0.0
        self.can_start_new_turn = True

        self.last_avoidance_path_was_outer = True
        self.initial_position_is_near = True
        self.initial_path_is_left = True
        self.initial_path_is_straight = False
        self.start_area_avoidance_required = False
        self.camera_init_sent = False

        self.accept_new_frames = True
        self.waiting_for_first_safe_frame = False

        # Variables for color detection logic
        self.latest_frame = None
        self.detection_results = [] # List to store detection results for majority vote

        self.planning_camera_wait_timer = None
        self.is_sampling_for_planning = False 

        self.is_sampling_for_planning = False 
        self.planning_initiated = False
        self.post_planning_reverse_target_dist_m = 0.7

        self.min_inner_dist_planning_approach = float('inf')
        self.planning_scan_complete = False

        self.gain_straight_align_wall = 1.3
        self.gain_straight_pass_through = 1.0

        self.gain_turning_approach = 1.3
        self.gain_turning_reverse = 1.3

        # --- System Setup ---
        self.bridge = CvBridge()
        self._setup_ros_communications()
        
        self.get_logger().info(f'Course Detector Node started. Initial state: {self.state.name}')


    def _load_parameters(self):
        """Loads all ROS2 parameters into class variables."""
        self.correct_mirrored_scan = self.get_parameter('correct_mirrored_scan').get_parameter_value().bool_value

        self.pan_servo_id = self.get_parameter('pan_servo_id').get_parameter_value().integer_value
        self.tilt_servo_id = self.get_parameter('tilt_servo_id').get_parameter_value().integer_value
        self.initial_pan_position = self.get_parameter('initial_pan_position').get_parameter_value().integer_value
        self.camera_move_duration = self.get_parameter('camera_move_duration_sec').get_parameter_value().double_value
        self.planning_detection_threshold = self.get_parameter('planning_detection_threshold').get_parameter_value().integer_value 

        self.tilt_position_cw = self.get_parameter('tilt_position_cw').get_parameter_value().integer_value
        self.tilt_position_ccw = self.get_parameter('tilt_position_ccw').get_parameter_value().integer_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value

        self.roi_planning_universal_flat = self.get_parameter('roi_planning_universal').get_parameter_value().integer_array_value
        self.roi_planning_start_area_flat = self.get_parameter('roi_planning_start_area').get_parameter_value().integer_array_value

        self.course_detection_threshold_m = self.get_parameter('course_detection_threshold_m').get_parameter_value().double_value
        self.course_detection_speed = self.get_parameter('course_detection_speed').get_parameter_value().double_value
        self.course_detection_slow_speed = self.get_parameter('course_detection_slow_speed').get_parameter_value().double_value
        self.initial_approach_target_dist_m = self.get_parameter('initial_approach_target_dist_m').get_parameter_value().double_value
        self.initial_approach_target_dist_start_area_m = self.get_parameter('initial_approach_target_dist_start_area_m').get_parameter_value().double_value

        self.red_lower1 = np.array(self.get_parameter('red_lower1').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.red_upper1 = np.array(self.get_parameter('red_upper1').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.red_lower2 = np.array(self.get_parameter('red_lower2').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.red_upper2 = np.array(self.get_parameter('red_upper2').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.green_lower = np.array(self.get_parameter('green_lower').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.green_upper = np.array(self.get_parameter('green_upper').get_parameter_value().integer_array_value, dtype=np.uint8)
        self.roi_left = self.get_parameter('roi_left').get_parameter_value().integer_array_value
        self.roi_right = self.get_parameter('roi_right').get_parameter_value().integer_array_value
        self.detection_threshold = self.get_parameter('detection_pixel_threshold').get_parameter_value().integer_value
        self.detection_samples = self.get_parameter('detection_samples').get_parameter_value().integer_value

        self.yaw_correction_threshold_deg = self.get_parameter('yaw_correction_threshold_deg').get_parameter_value().double_value
        self.yaw_correction_wide_dist = self.get_parameter('yaw_correction_wide_dist').get_parameter_value().double_value
        # Maneuver
        self.turn_reverse_steer = self.get_parameter('turn_reverse_steer').get_parameter_value().double_value
        self.kp_approach_angle = self.get_parameter('kp_approach_angle').get_parameter_value().double_value
        self.reverse_yaw_tolerance_deg = self.get_parameter('reverse_yaw_tolerance_deg').get_parameter_value().double_value

        # Load fixed approach angles
        self.turn_approach_dist_from_outer_path = self.get_parameter('turn_approach_dist_from_outer_path').get_parameter_value().double_value
        self.turn_approach_dist_from_inner_path = self.get_parameter('turn_approach_dist_from_inner_path').get_parameter_value().double_value
        self.turn_approach_angle_from_outer_path_deg = self.get_parameter('turn_approach_angle_from_outer_path_deg').get_parameter_value().double_value
        self.turn_approach_angle_from_inner_path_deg = self.get_parameter('turn_approach_angle_from_inner_path_deg').get_parameter_value().double_value
        self.turn_approach_angle_from_outer_path_and_start_area_deg = self.get_parameter('turn_approach_angle_from_outer_path_and_start_area_deg').get_parameter_value().double_value
        self.turn_approach_dist_from_outer_path_and_start_area_m = self.get_parameter('turn_approach_dist_from_outer_path_and_start_area_m').get_parameter_value().double_value

        self.dynamic_approach_start_angle_deg = self.get_parameter('dynamic_approach_start_angle_deg').get_parameter_value().double_value
        self.dynamic_approach_start_dist_m = self.get_parameter('dynamic_approach_start_dist_m').get_parameter_value().double_value
        self.dynamic_approach_straighten_offset_m = self.get_parameter('dynamic_approach_straighten_offset_m').get_parameter_value().double_value

        # Driving
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.max_steer = self.get_parameter('max_steer').get_parameter_value().double_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().string_value
        
        self.obstacle_clearance_dist_m = self.get_parameter('obstacle_clearance_dist_m').get_parameter_value().double_value

        # Load avoidance parameters
        self.avoid_target_outer_dist_m = self.get_parameter('avoid_target_outer_dist_m').get_parameter_value().double_value
        self.avoid_kp_angle = self.get_parameter('avoid_kp_angle').get_parameter_value().double_value
        self.avoid_kp_dist = self.get_parameter('avoid_kp_dist').get_parameter_value().double_value
        self.avoid_speed = self.get_parameter('avoid_speed').get_parameter_value().double_value
        self.avoid_inner_pass_thresh_m = self.get_parameter('avoid_inner_pass_thresh_m').get_parameter_value().double_value

        self.avoid_turn_in_kp_angle = self.get_parameter('avoid_turn_in_kp_angle').get_parameter_value().double_value
        self.avoid_turn_in_yaw_tolerance_deg = self.get_parameter('avoid_turn_in_yaw_tolerance_deg').get_parameter_value().double_value

        self.avoid_approach_angle_deg = self.get_parameter('avoid_outer_approach_angle_deg').get_parameter_value().double_value
        self.avoid_approach_target_dist_m = self.get_parameter('avoid_outer_approach_target_dist_m').get_parameter_value().double_value
        self.avoid_outer_approach_target_dist_start_area_m = self.get_parameter('avoid_outer_approach_target_dist_start_area_m').get_parameter_value().double_value

        self.avoid_inner_approach_angle_deg = self.get_parameter('avoid_inner_approach_angle_deg').get_parameter_value().double_value
        self.avoid_inner_approach_target_dist_m = self.get_parameter('avoid_inner_approach_target_dist_m').get_parameter_value().double_value

        self.align_target_inner_dist_m = self.get_parameter('align_target_inner_dist_m').get_parameter_value().double_value
        self.align_dist_tolerance_m = self.get_parameter('align_dist_tolerance_m').get_parameter_value().double_value
        self.align_kp_angle = self.get_parameter('align_kp_angle').get_parameter_value().double_value
        self.align_kp_dist = self.get_parameter('align_kp_dist').get_parameter_value().double_value
        self.align_target_outer_dist_m = self.get_parameter('align_target_outer_dist_m').get_parameter_value().double_value
        self.align_target_outer_dist_start_area_m = self.get_parameter('align_target_outer_dist_start_area_m').get_parameter_value().double_value

        self.avoidance_path_plan = self.get_parameter('avoidance_path_plan').get_parameter_value().string_array_value
        self.entrance_obstacle_plan = self.get_parameter('entrance_obstacle_plan').get_parameter_value().bool_array_value

        self.save_debug_images = self.get_parameter('save_debug_images').get_parameter_value().bool_value
        self.debug_image_path = self.get_parameter('debug_image_path').get_parameter_value().string_value

        log_level_str = self.get_parameter('log_level').get_parameter_value().string_value
        log_level_map = {
            'DEBUG': rclpy.logging.LoggingSeverity.DEBUG,
            'INFO': rclpy.logging.LoggingSeverity.INFO,
            'WARN': rclpy.logging.LoggingSeverity.WARN,
            'ERROR': rclpy.logging.LoggingSeverity.ERROR,
        }
        log_level = log_level_map.get(log_level_str.upper(), rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)
        self.get_logger().info(f"Logger level set to {log_level_str.upper()}")

        self.get_logger().info("--- Parameters Initialized ---")
        self.get_logger().info(f"Direction: '{self.direction}'")
        self.get_logger().info(f"Base Speed: {self.forward_speed}, Gain: {self.gain}")
        self.get_logger().info(f"Inner Wall Threshold: {self.inner_wall_disappear_threshold} m")
        self.get_logger().info(f"Will stop after {self.max_turns} turns.")
        self.get_logger().info(f"Loaded avoidance path plan: {self.avoidance_path_plan}")
        self.get_logger().info("----------------------------")

    def _correct_mirrored_scan(self, msg: LaserScan) -> LaserScan:
        """
        Corrects a mirrored (clockwise) LaserScan message by reversing its data arrays.
        This function creates a new message and does not modify the original.
        """
        corrected_msg = LaserScan()
        # Copy all header and metadata fields
        corrected_msg.header = msg.header
        corrected_msg.angle_min = msg.angle_min
        corrected_msg.angle_max = msg.angle_max
        corrected_msg.angle_increment = msg.angle_increment
        corrected_msg.time_increment = msg.time_increment
        corrected_msg.scan_time = msg.scan_time
        corrected_msg.range_min = msg.range_min
        corrected_msg.range_max = msg.range_max
        
        # Reverse the main data arrays
        corrected_msg.ranges = list(reversed(msg.ranges))
        if msg.intensities:
            corrected_msg.intensities = list(reversed(msg.intensities))
            
        return corrected_msg

    def _setup_ros_communications(self):
        """Initializes all publishers, subscribers, and timers."""
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.servo_pub = self.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)

        # QoS profile for camera subscriber, matching the publisher
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriber for camera feed
        self.image_sub = self.create_subscription(
            Image, 
            '/ascamera/camera_publisher/rgb0/image', 
            self.image_callback, 
            qos_profile=qos_profile_img
        )
        
        # QoS profile for LiDAR
        qos_profile_lidar = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_lidar
        )
        self.yaw_subscriber = self.create_subscription(
            Float64, '/imu/yaw', self.yaw_callback, 10
        )
        rclpy.get_default_context().on_shutdown(self.shutdown_callback)

    # --- Main Callback and State Dispatcher ---
    def scan_callback(self, msg):
        """The main callback that dispatches logic based on the current state."""
        if not msg.ranges:
            return
        
        if self.correct_mirrored_scan:
            msg = self._correct_mirrored_scan(msg)
        
        self.latest_scan_msg = msg

        if self.state == State.DETERMINE_COURSE:
            self._handle_state_determine_course(msg)
        elif self.state == State.FINISHED:
            self._handle_state_finished()
        elif self.state == State.TURNING:
            self._handle_state_turning(msg)
        elif self.state == State.STRAIGHT:
            self._handle_state_straight(msg)

    def yaw_callback(self, msg):
        self.current_yaw_deg = msg.data

    def image_callback(self, msg):
        """Callback to receive and store the latest camera frame."""
        if not self.accept_new_frames:
            return

        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            # --- Trigger sampling on the first valid frame after camera movement ---
            if self.waiting_for_first_safe_frame:
                self.get_logger().info("First safe frame received. Starting sampling process.")
                self.waiting_for_first_safe_frame = False
                self.is_sampling_for_planning = True

        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge Error: {e}')

    def shutdown_callback(self):
        self.publish_twist_with_gain(0.0, 0.0)

    def _camera_initialization_complete_callback(self):
        """
        Called by a timer after the camera has had time to move.
        Transitions the state machine to the color detection step and destroys the timer.
        """
        if self.camera_wait_timer:
            self.camera_wait_timer.destroy()
            self.camera_wait_timer = None

        if self.determine_course_sub_state == DetermineCourseSubState.INITIALIZING_CAMERA:
            self.get_logger().info("Camera initialization complete. Transitioning to DETECTING_OBSTACLE_COLOR.")
            # Transition to the correct new state
            self.determine_course_sub_state = DetermineCourseSubState.DETECTING_OBSTACLE_COLOR

    # --- State Handler Functions ---
    def _handle_state_determine_course(self, msg: LaserScan):
        """Dispatches to the correct handler based on the determine_course_sub_state."""
        if self.determine_course_sub_state == DetermineCourseSubState.INITIALIZING_CAMERA:
            self._handle_determine_sub_initializing_camera()
        # Dispatch for the new color detection state
        elif self.determine_course_sub_state == DetermineCourseSubState.DETECTING_OBSTACLE_COLOR:
            self._handle_determine_sub_detecting_color(msg)
        elif self.determine_course_sub_state == DetermineCourseSubState.PREPARE_TO_START:
            self._handle_determine_sub_prepare_to_start(msg)
        elif self.determine_course_sub_state == DetermineCourseSubState.DECIDE_INITIAL_PATH:
            self._handle_determine_sub_decide_initial_path()
        elif self.determine_course_sub_state == DetermineCourseSubState.APPROACH_INITIAL_WALL:
            self._handle_determine_sub_approach_initial_wall(msg)
        elif self.determine_course_sub_state == DetermineCourseSubState.DETECTING_STRAIGHT:
            self._handle_determine_sub_detecting_straight(msg)

    def _handle_state_finished(self):
        """Action for the FINISHED state: stop the robot."""
        self.publish_twist_with_gain(0.0, 0.0)

    def _handle_state_straight(self, msg):
        """Dispatches to the correct handler based on the straight_sub_state."""
        if self.straight_sub_state == StraightSubState.ALIGN_WITH_OUTER_WALL:
            self._handle_straight_sub_align_with_outer_wall(msg)
        elif self.straight_sub_state == StraightSubState.ALIGN_WITH_INNER_WALL:
            self._handle_straight_sub_align_with_inner_wall(msg)
        elif self.straight_sub_state == StraightSubState.PRE_PLANNING_ADJUST:
            self._handle_straight_sub_pre_planning_adjust(msg)
        elif self.straight_sub_state == StraightSubState.PLAN_NEXT_AVOIDANCE:
            self._handle_straight_sub_plan_next_avoidance()
        elif self.straight_sub_state == StraightSubState.POST_PLANNING_REVERSE:
            self._handle_straight_sub_post_planning_reverse(msg)
        elif self.straight_sub_state == StraightSubState.AVOID_OUTER_TURN_IN:
            self._handle_straight_sub_avoid_outer_turn_in(msg)
        elif self.straight_sub_state == StraightSubState.AVOID_OUTER_PASS_THROUGH:
            self._handle_straight_sub_avoid_outer_pass_through(msg)
        elif self.straight_sub_state == StraightSubState.AVOID_INNER_TURN_IN:
            self._handle_straight_sub_avoid_inner_turn_in(msg)
        elif self.straight_sub_state == StraightSubState.AVOID_INNER_PASS_THROUGH:
            self._handle_straight_sub_avoid_inner_pass_through(msg)

    def _handle_state_turning(self, msg):
        """Handles the multi-step turning maneuver."""
        if self.turning_sub_state == TurningSubState.APPROACH:
            self._handle_turning_approach(msg)
        elif self.turning_sub_state == TurningSubState.REVERSE:
            self._handle_turning_reverse()
        elif self.turning_sub_state == TurningSubState.CREATING_SPACE:
            self._handle_turning_sub_creating_space(msg)
        elif self.turning_sub_state == TurningSubState.CHECK_AND_FINISH:
            self._handle_turning_sub_check_and_finish(msg)
        elif self.turning_sub_state == TurningSubState.FINALIZE_TURN:
            self._handle_turning_sub_finalize_turn()

    # Handler for the PREPARE_TO_START sub-state
    def _handle_determine_sub_initializing_camera(self):
        """
        Sub-state: Sends the command to move the camera to its initial angle
        and waits for the movement to complete before proceeding.
        """
        # This handler's only job is to send the command once and set a timer.
        # After that, it should do nothing until the timer callback changes the state.
        if not self.camera_init_sent:
            self.get_logger().info("INITIALIZING_CAMERA: Sending command to set initial camera angle.")
            
            msg = SetPWMServoState()
            msg.duration = self.camera_move_duration
            
            pan_state = PWMServoState()
            pan_state.id = [self.pan_servo_id]
            pan_state.position = [self.initial_pan_position]

            msg.state = [pan_state]

            self.servo_pub.publish(msg)
            self.camera_init_sent = True

            # The time to wait for the camera to finish moving.
            # Add a small buffer (e.g., 0.2s) to be safe.
            wait_time = self.camera_move_duration + 0.5
            
            # Create a one-shot timer that will trigger the state transition.
            self.camera_wait_timer = self.create_timer(
                wait_time, 
                self._camera_initialization_complete_callback
            )
            
            self.get_logger().info(f"Waiting {wait_time:.2f} seconds for camera to move...")

        # Crucially, ensure the robot remains stationary while in this state.
        self.publish_twist_with_gain(0.0, 0.0)

    def _handle_determine_sub_detecting_color(self, msg: LaserScan):
        """
        Sub-state: Samples multiple frames using ROIs to get a stable color 
        detection result for the initial starting procedure.
        """
        if len(self.detection_results) >= self.detection_samples:
            return

        if self.latest_frame is None:
            self.get_logger().warn('Waiting for camera frame...', throttle_duration_sec=2.0)
            self.publish_twist_with_gain(0.0, 0.0)
            return

        lx, ly, lw, lh = self.roi_left
        left_roi_points = [[lx, ly], [lx + lw, ly], [lx + lw, ly + lh], [lx, ly + lh]]
        rx, ry, rw, rh = self.roi_right
        right_roi_points = [[rx, ry], [rx + rw, ry], [rx + rw, ry + rh], [rx, ry + rh]]
        
        all_rois = {'left': left_roi_points, 'right': right_roi_points}
        detection_data = self._detect_obstacle_color_in_frame(self.latest_frame, rois_to_check=all_rois)
        left_areas = detection_data['areas']['left']
        right_areas = detection_data['areas']['right']
        
        # --- Store detailed detection results ---
        is_red_left = left_areas['RED'] > self.detection_threshold
        is_red_right = right_areas['RED'] > self.detection_threshold
        is_green_left = left_areas['GREEN'] > self.detection_threshold
        is_green_right = right_areas['GREEN'] > self.detection_threshold

        result_data = {'color': 'NONE', 'left': False, 'right': False}
        if is_red_left or is_red_right:
            result_data['color'] = 'RED'
            result_data['left'] = is_red_left
            result_data['right'] = is_red_right
        elif is_green_left or is_green_right:
            result_data['color'] = 'GREEN'
            result_data['left'] = is_green_left
            result_data['right'] = is_green_right
        
        self.detection_results.append(result_data)

        # --- Save debug images for the first sample in this state ---
        if self.save_debug_images and len(self.detection_results) == 1:
            self._save_annotated_image(
                base_name="initial_detection",
                turn_count=0,
                frame_bgr=detection_data['frame_bgr'],
                masks=detection_data['masks'],
                rois=all_rois
            )
        
        self.get_logger().debug(f"Initial detection sample #{len(self.detection_results)}: {result_data} (L:{left_areas}, R:{right_areas})", throttle_duration_sec=0.2)

        if len(self.detection_results) >= self.detection_samples:
            self.get_logger().info(f"Sample collection complete ({len(self.detection_results)} samples). Proceeding to PREPARE_TO_START.")
            self.determine_course_sub_state = DetermineCourseSubState.PREPARE_TO_START
            self._handle_determine_sub_prepare_to_start(msg)
        else:
            self.publish_twist_with_gain(0.0, 0.0)

    def _handle_determine_sub_prepare_to_start(self, msg: LaserScan):
        """
        Sub-state: Measures front wall distance and decides the next sub-state.
        - If wall is close (< 1.4m), transitions to DETECTING_STRAIGHT.
        - If wall is far (>= 1.4m) or obstacle is close (< 0.9m) transitions to DECIDE_INITIAL_PATH.
        """
        front_dist = self.get_distance_at_world_angle(msg, 0.0)

        if not math.isnan(front_dist):
            self.publish_twist_with_gain(0.0, 0.0)
            self.get_logger().debug(f"DETERMINE_COURSE (PREPARE): Front wall distance acquired: {front_dist:.2f}m")
            
            if front_dist > 0.9 and front_dist < 1.4:
                self.get_logger().info("Front wall is close. Transitioning to DETECTING_STRAIGHT.")
                self.initial_position_is_near = True
                self.determine_course_sub_state = DetermineCourseSubState.DETECTING_STRAIGHT
            else:
                self.get_logger().info("Front wall is far. Transitioning to DECIDE_INITIAL_PATH.")
                self.initial_position_is_near = False
                self.determine_course_sub_state = DetermineCourseSubState.DECIDE_INITIAL_PATH
        else:
            self.get_logger().warn(
                "DETERMINE_COURSE (PREPARE): Could not get valid front distance. Moving slowly...",
                throttle_duration_sec=1.0
            )
            self.publish_twist_with_gain(self.course_detection_slow_speed, 0.0)

    def _handle_determine_sub_decide_initial_path(self):
        """
        Sub-state: Decides path based on the majority vote from color detection,
        and determines if special start area avoidance is required.
        """
        # --- Vote based on the 'color' key in the dictionary ---
        color_votes = [res['color'] for res in self.detection_results]
        red_votes = color_votes.count("RED")
        green_votes = color_votes.count("GREEN")
        none_votes = color_votes.count("NONE")

        self.get_logger().info(f"Color detection vote - Red: {red_votes}, Green: {green_votes}, None: {none_votes}")
        
        self.start_area_avoidance_required = False # Reset flag

        if red_votes > green_votes and red_votes >= none_votes:
            self.get_logger().info("Red object has majority. Deciding to approach RIGHT wall.")
            self.initial_path_is_left = False
            self.initial_path_is_straight = False
            
            # --- Check if special avoidance is needed for RED ---
            left_red_detected_count = sum(1 for res in self.detection_results if res['color'] == 'RED' and res['left'])
            right_red_detected_count = sum(1 for res in self.detection_results if res['color'] == 'RED' and res['right'])
            if left_red_detected_count > right_red_detected_count:
                self.get_logger().warn("RED obstacle on the LEFT. Start area avoidance REQUIRED.")
                self.start_area_avoidance_required = True

            self.determine_course_sub_state = DetermineCourseSubState.APPROACH_INITIAL_WALL

        elif green_votes > red_votes and green_votes >= none_votes:
            self.get_logger().info("Green object has majority. Deciding to approach LEFT wall.")
            self.initial_path_is_left = True
            self.initial_path_is_straight = False

            # --- Check if special avoidance is needed for GREEN ---
            left_green_detected_count = sum(1 for res in self.detection_results if res['color'] == 'GREEN' and res['left'])
            right_green_detected_count = sum(1 for res in self.detection_results if res['color'] == 'GREEN' and res['right'])
            if right_green_detected_count > left_green_detected_count:
                self.get_logger().warn("GREEN obstacle on the RIGHT. Start area avoidance REQUIRED.")
                self.start_area_avoidance_required = True

            self.determine_course_sub_state = DetermineCourseSubState.APPROACH_INITIAL_WALL
            
        else: # This covers cases where 'NONE' is the majority, or there's a tie
            self.get_logger().info("No clear majority or no obstacle detected. Proceeding STRAIGHT.")
            self.initial_path_is_straight = True
            self.determine_course_sub_state = DetermineCourseSubState.DETECTING_STRAIGHT

    def _handle_determine_sub_approach_initial_wall(self, msg: LaserScan):
        """
        Sub-state: Approaches the initially chosen wall (left or right) until a
        target distance is reached, then transitions to DETECTING_STRAIGHT.
        If the wall is lost, it continues straight with the last valid steer.
        """
        if self.initial_path_is_left:
            wall_angle_deg = 90.0
            steer_direction = 1.0
        else: # Approaching right wall
            wall_angle_deg = -90.0
            steer_direction = -1.0
            
        wall_dist = self.get_distance_at_world_angle(msg, wall_angle_deg)
        
        # --- Handle lost wall situation gracefully ---
        if math.isnan(wall_dist):
            self.get_logger().warn(
                f"Lost sight of the target wall during initial approach. "
                f"Continuing straight with last steer ({self.last_valid_steer:.2f}).",
                throttle_duration_sec=1.0
            )
            # Continue moving with the last known good steering angle
            self.publish_twist_with_gain(self.course_detection_speed, self.last_valid_steer)
            return

        if self.start_area_avoidance_required:
            target_dist = self.initial_approach_target_dist_start_area_m
        else:
            target_dist = self.initial_approach_target_dist_m

        # Check for completion of this phase
        if wall_dist <= target_dist:
            self.get_logger().info(f"Initial approach complete (Dist: {wall_dist:.2f}m). Transitioning to DETECTING_STRAIGHT.")
            self.determine_course_sub_state = DetermineCourseSubState.DETECTING_STRAIGHT
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # Simple steering logic to turn towards the wall
        initial_steer = 0.4 * steer_direction
        
        # Store the current valid steer command
        self.last_valid_steer = initial_steer
        
        self.get_logger().debug(
            f"APPROACH_INITIAL_WALL | TargetDist: {self.initial_approach_target_dist_m:.2f}, "
            f"CurrentDist: {wall_dist:.2f}m | Steer: {initial_steer:.2f}",
            throttle_duration_sec=0.2
        )
        self.publish_twist_with_gain(self.course_detection_speed, initial_steer)
    
    def _handle_determine_sub_detecting_straight(self, msg: LaserScan):
        """
        Sub-state: Moves forward while maintaining yaw=0 and checks for an open side.
        Upon finding the course, it transitions to the dynamic planning state.
        """
        course_found, determined_direction = self._check_course_and_get_direction(msg)

        if course_found:
            self.get_logger().info(f"******* Course direction set to: {self.direction.upper()} *******")
            
            # --- Transition to the main STRAIGHT state for planning ---
            self.get_logger().info("Initial course detected. Transitioning to main state machine for planning.")

            # Set the base angle for the very first turn.
            # In DETERMINE_COURSE, the robot is aligned with yaw=0, so the base angle is 0.
            self.approach_base_yaw_deg = 0.0

            # Transition to the main state machine
            self.state = State.STRAIGHT
            self.straight_sub_state = StraightSubState.PRE_PLANNING_ADJUST
            self.publish_twist_with_gain(0.0, 0.0) # Stop briefly before planning
            
        else:
            # P-control to maintain yaw at 0 degrees
            target_yaw_deg = 0.0
            angle_error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
            angle_steer = self.align_kp_angle * angle_error_deg
            final_steer = max(min(angle_steer, self.max_steer), -self.max_steer)
            
            self.publish_twist_with_gain(self.course_detection_speed, final_steer)
            self.get_logger().debug(
                f"DETERMINE_COURSE (DETECTING): Moving straight (YawErr: {angle_error_deg:.1f}). Steer: {final_steer:.2f}",
                throttle_duration_sec=0.5
            )

    def _handle_straight_sub_align_with_outer_wall(self, msg: LaserScan):
        if self._check_for_finish_condition(msg):
            return
        """A PID-based wall follower that now calls a generic alignment executor."""
        base_angle_deg = self._calculate_base_angle()

        is_turning, _ = self._check_for_corner(msg, base_angle_deg)
        if is_turning:
            return # Corner detected, turning maneuver initiated.

        # --- REFACTORED: Delegate the actual PID control logic to a generic function ---
        self._execute_pid_alignment(msg, base_angle_deg, is_outer_wall=True)

    def _handle_straight_sub_align_with_inner_wall(self, msg: LaserScan):
        if self._check_for_finish_condition(msg):
            return
        """A PID-based wall follower that now calls a generic alignment executor."""
        base_angle_deg = self._calculate_base_angle()

        is_turning, _ = self._check_for_corner(msg, base_angle_deg)
        if is_turning:
            return # Corner detected, turning maneuver initiated.

        # --- REFACTORED: Delegate the actual PID control logic to a generic function ---
        self._execute_pid_alignment(msg, base_angle_deg, is_outer_wall=False)

    def _handle_straight_sub_pre_planning_adjust(self, msg: LaserScan):
        """
        State: Moves to planning distance, records min inner dist, scans on arrival,
        assesses entrance, and finally transitions to PLAN_NEXT_AVOIDANCE.
        """
        # If scan is already done, transition immediately.
        if self.planning_scan_complete:
            self.straight_sub_state = StraightSubState.PLAN_NEXT_AVOIDANCE
            return

        target_front_dist = 0.50
        forward_speed = self.course_detection_slow_speed
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)

        # Determine inner wall angle based on driving direction
        if self.direction == 'ccw':
            inner_wall_angle = self._angle_normalize(self.approach_base_yaw_deg + 90.0)
        else: # cw
            inner_wall_angle = self._angle_normalize(self.approach_base_yaw_deg - 90.0)
        inner_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)

        is_approaching = math.isnan(front_dist) or front_dist > target_front_dist

        if is_approaching:
            # --- Approaching Phase ---
            if not math.isnan(inner_dist) and inner_dist < self.min_inner_dist_planning_approach:
                self.min_inner_dist_planning_approach = inner_dist

            angle_error_deg = self._angle_diff(self.approach_base_yaw_deg, self.current_yaw_deg)
            angle_steer = self.align_kp_angle * angle_error_deg
            final_steer = max(min(angle_steer, self.max_steer), -self.max_steer)
            self.publish_twist_with_gain(forward_speed, final_steer)
            
            self.get_logger().debug(
                f"Planning Approach... Front: {front_dist:.2f}m | Min Inner: {self.min_inner_dist_planning_approach:.3f}m",
                throttle_duration_sec=0.2
            )
        else:
            # --- Scan and Assess Phase (runs once upon arrival) ---
            self.publish_twist_with_gain(0.0, 0.0) # Stop the robot
            self.get_logger().info(f"Approach for planning complete (Front Dist: {front_dist:.3f}m). Scanning...")

            scan_range_deg = 25.0
            clearance_threshold = 1.2

            if self.direction == 'ccw':
                outer_wall_angle = self._angle_normalize(self.approach_base_yaw_deg - 90.0)
            else: # cw
                outer_wall_angle = self._angle_normalize(self.approach_base_yaw_deg + 90.0)
            
            scan_info = self.get_closest_distance_in_range(msg, inner_wall_angle, scan_range_deg)
            min_dist_scan = scan_info['distance'] if scan_info else float('inf')
            outer_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
            
            closest_overall_dist = min(self.min_inner_dist_planning_approach, min_dist_scan)

            # --- Detailed Logging ---
            self.get_logger().info("--- Pre-planning Assessment ---")
            self.get_logger().info(f"- Min dist during APPROACH: {self.min_inner_dist_planning_approach:.4f} m")
            if scan_info:
                self.get_logger().info(f"- Min dist from SCAN (+/-{scan_range_deg}deg): {min_dist_scan:.4f} m at {scan_info['angle']:.1f} deg")
            else:
                self.get_logger().warn(f"- Min dist from SCAN (+/-{scan_range_deg}deg): Not found.")
            
            if closest_overall_dist == float('inf'):
                self.get_logger().warn("--> Final Closest Inner: Not detected.")
            else:
                self.get_logger().info(f"--> Final Closest Inner: {closest_overall_dist:.4f} m")
            # --- End of Detailed Logging ---

            obstacle_detected = False # Default to False
            if not math.isnan(outer_dist) and closest_overall_dist != float('inf'):
                total_dist = outer_dist + closest_overall_dist
                if total_dist < clearance_threshold:
                    self.get_logger().error(f"[ENTRANCE ASSESSMENT] OBSTACLE DETECTED! Clearance: {total_dist:.3f}m (< {clearance_threshold}m)")
                    obstacle_detected = True # Set to True if obstacle is found
            
            # --- Store the result in the plan ---
            next_segment_index = (self.wall_segment_index + 1) % len(self.entrance_obstacle_plan)
            self.entrance_obstacle_plan[next_segment_index] = obstacle_detected
            self.get_logger().warn(f"Updated entrance_obstacle_plan at index {next_segment_index} to: {obstacle_detected}")
            self.get_logger().info(f"Current plan: {self.entrance_obstacle_plan}")
            # --- End of NEW ---

            # Set flag to true and transition on the next callback
            self.planning_scan_complete = True
            self.get_logger().info("Entrance assessment complete. Proceeding to planning.")

    def _handle_straight_sub_plan_next_avoidance(self):
        """
        Sub-state handler for the dynamic planning sequence.
        - Sets camera to a fixed side-facing angle (left/right).
        - Initiates camera movement.
        - Performs multi-frame color sampling once the camera is settled.
        - Makes a decision and transitions to POST_PLANNING_REVERSE state upon completion.
        """
        if self.planning_initiated and not self.is_sampling_for_planning:
            self.publish_twist_with_gain(0.0, 0.0)
            return
        # --- Phase 1: Initiation and Fixed Angle Calculation ---
        if not self.is_sampling_for_planning:
            if self.planning_camera_wait_timer is not None and not self.planning_camera_wait_timer.is_canceled():
                self.publish_twist_with_gain(0.0, 0.0)
                return

            self.get_logger().warn(">>> PLAN_NEXT_AVOIDANCE: Initiating planning sequence. <<<")
            self.planning_initiated = True
            self.detection_results.clear() # <-- Ensure results are cleared at the start of every plan.

            # Step 1: Determine the fixed tilt position based on direction
            if self.direction == 'ccw':
                final_tilt_position = self.tilt_position_ccw
                self.get_logger().info("Direction is CCW, setting camera to left-facing angle.")
            else: # cw
                final_tilt_position = self.tilt_position_cw
                self.get_logger().info("Direction is CW, setting camera to right-facing angle.")
            
            # Step 2: Command the camera to move to the fixed position.
            target_pan_position = self.initial_pan_position
            camera_move_duration = self.camera_move_duration
            
            self.get_logger().info("Pausing frame updates for camera movement.")
            self.accept_new_frames = False
            self._set_camera_angle(pan_position=target_pan_position, tilt_position=final_tilt_position, duration_sec=camera_move_duration)

            # Step 3: Wait for the camera to settle before sampling.
            wait_time = camera_move_duration + 0.3
            self.planning_camera_wait_timer = self.create_timer(wait_time, self._on_camera_settled_for_planning)
            self.get_logger().info(f"Moving camera. Waiting {wait_time:.2f} seconds...")
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # --- Phase 2: Multi-frame Sampling (runs after camera has settled) ---
        if self.is_sampling_for_planning:
            if len(self.detection_results) < self.detection_samples:
                if self.latest_frame is None:
                    self.get_logger().warn('Waiting for a fresh camera frame to sample...', throttle_duration_sec=2.0)
                    self.publish_twist_with_gain(0.0, 0.0)
                    return

                # Step 1: Select the universal ROI preset.
                if self.turn_count == 3:
                    selected_roi_flat = self.roi_planning_start_area_flat
                else:
                    selected_roi_flat = self.roi_planning_universal_flat
                
                # Step 2: Reshape the flat array into a list of points
                roi_points = self._reshape_roi_points(selected_roi_flat)

                # Step 3: Mirror the ROI if driving CCW
                if self.direction == 'ccw' and roi_points:
                    self.get_logger().debug("CCW direction detected, mirroring ROI.")
                    roi_points = self._mirror_roi_points(roi_points)

                rois_to_check = {'planning_roi': roi_points} if roi_points else None

                # Step 4: Perform color detection with the final ROI
                detection_data = self._detect_obstacle_color_in_frame(self.latest_frame, rois_to_check=rois_to_check)
                
                area_dict = detection_data['areas'].get('planning_roi', detection_data['areas']['full_frame'])
                
                self.detection_results.append(area_dict)
                self.get_logger().debug(f"Planning sample #{len(self.detection_results)}: Areas R={area_dict['RED']}, G={area_dict['GREEN']}", throttle_duration_sec=0.2)

                if self.save_debug_images and len(self.detection_results) == 1:
                    self._save_annotated_image(
                        base_name="planning_detection",
                        turn_count=self.turn_count + 1,
                        frame_bgr=detection_data['frame_bgr'],
                        masks=detection_data['masks'],
                        rois=rois_to_check
                    )
                
                self.publish_twist_with_gain(0.0, 0.0)
                return

            # --- Phase 3: Final Decision ---
            self.get_logger().info("Sample collection complete. Finalizing plan...")

            final_red_area = 0
            final_green_area = 0
            if self.detection_results:
                red_areas = [result['RED'] for result in self.detection_results]
                green_areas = [result['GREEN'] for result in self.detection_results]
                
                final_red_area = np.median(red_areas)
                final_green_area = np.median(green_areas)
                
                self.get_logger().info(f"Collected RED areas: {red_areas}")
                self.get_logger().info(f"Collected GREEN areas: {green_areas}")
            else:
                self.get_logger().error("No detection results were collected.")

            self.get_logger().info(f"Median Representative Areas -> RED: {final_red_area:.1f}, GREEN: {final_green_area:.1f}")

            is_red_present = final_red_area > self.planning_detection_threshold
            is_green_present = final_green_area > self.planning_detection_threshold

            self.get_logger().warn(f"Final Decision -> RED Present: {is_red_present}, GREEN Present: {is_green_present}")
            
            self._update_avoidance_plan_based_on_vision(is_red_present, is_green_present, final_red_area, final_green_area)

            self.is_sampling_for_planning = False
            
            self.get_logger().info("Planning complete. Transitioning to POST_PLANNING_REVERSE.")
            self.straight_sub_state = StraightSubState.POST_PLANNING_REVERSE
            self.post_planning_reverse_target_dist_m = 0.7 # Reset to default for next approach
        
        else:
            self.publish_twist_with_gain(0.0, 0.0)

    def _handle_straight_sub_post_planning_reverse(self, msg: LaserScan):
        """
        Sub-state: After planning, reverse straight back until the robot is
        at a safe distance from the front wall before starting the turn.
        """
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)

        if math.isnan(front_dist):
            self.get_logger().warn("Cannot get front distance for reversing. Reversing slowly.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(-self.course_detection_slow_speed * 0.5, 0.0)
            return

        # Check for completion of the reverse maneuver
        if front_dist >= self.post_planning_reverse_target_dist_m:
            self.get_logger().info(f"Reverse complete (Front Dist: {front_dist:.3f}m).")

            if self._is_turn_maneuver_required():
                # Turn maneuver is necessary
                self.get_logger().info("Preparing for turn.")
                self.publish_twist_with_gain(0.0, 0.0)
                self._prepare_for_turning(base_angle_deg=self.approach_base_yaw_deg)
            else:
                # Turn maneuver can be skipped
                self.get_logger().info("Skipping turn maneuver.")
                self.state = State.TURNING
                self.turning_sub_state = TurningSubState.FINALIZE_TURN
                self.publish_twist_with_gain(0.0, 0.0) # Stop briefly before adjusting
        else:
            # Reverse straight back without turning
            self.get_logger().debug(
                f"Reversing... Target: > {self.post_planning_reverse_target_dist_m:.2f}m, "
                f"Current: {front_dist:.3f}m",
                throttle_duration_sec=0.2
            )
            self.publish_twist_with_gain(-self.course_detection_slow_speed, 0.0)

    def _handle_straight_sub_avoid_outer_turn_in(self, msg: LaserScan):
        """Phase 1: Approach the outer wall, while also checking if the obstacle is passed."""
        if self._check_for_finish_condition(msg):
            return
            
        base_angle_deg = self._calculate_base_angle()
        
        # Define target yaw and wall measurement angles
        if self.direction == 'ccw':
            target_yaw_deg = self._angle_normalize(base_angle_deg - self.avoid_approach_angle_deg)
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
            inner_wall_angle_for_pass_check = self._angle_normalize(base_angle_deg + 90.0)
        else: # cw
            target_yaw_deg = self._angle_normalize(base_angle_deg + self.avoid_approach_angle_deg)
            outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            inner_wall_angle_for_pass_check = self._angle_normalize(base_angle_deg - 90.0)

        outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle_for_pass_check)

        # Monitor for passing the obstacle during the turn-in phase
        path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='outer')

        if self.is_passing_obstacle:
            # Check if we have completed the pass MANEUVER during the turn-in
            if not math.isnan(inner_wall_dist) and inner_wall_dist > self.avoid_inner_pass_thresh_m:
                self.get_logger().warn("!!! Obstacle passed completely during TURN_IN phase. Skipping PASS_THROUGH. !!!")
                if path_type == 'outer_to_inner':
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.AVOID_INNER_TURN_IN,
                        path_was_outer=True,
                        enable_next_turn=False)
                else:
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.ALIGN_WITH_OUTER_WALL,
                        path_was_outer=True,
                        enable_next_turn=True)
                return
        else:
            # Check if we have started passing the obstacle
            is_oriented_correctly = self._check_yaw_alignment(base_angle_deg, 45.0)
            sum_side_walls_dist = inner_wall_dist + outer_wall_dist
            if(not math.isnan(inner_wall_dist) and not math.isnan(outer_wall_dist) and 
                inner_wall_dist < self.avoid_inner_pass_thresh_m and sum_side_walls_dist < 0.7 and 
                is_oriented_correctly):
                self.get_logger().warn(">>> Passing obstacle now (during TURN_IN)...")
                self.is_passing_obstacle = True
        # --- Relaxed the isnan check to continue steering ---
        if math.isnan(outer_wall_dist):
            self.get_logger().warn("Lost sight of outer wall during approach. Continuing with P-control.", throttle_duration_sec=1.0)
            # Do not return; allow P-control to continue steering
        
        if self.wall_segment_index == 0:
            approach_target_dist = self.avoid_outer_approach_target_dist_start_area_m
        else:
            approach_target_dist = self.avoid_approach_target_dist_m

        # --- Added condition to prevent premature completion from noise ---
        if not math.isnan(outer_wall_dist) and outer_wall_dist > 0.0 and outer_wall_dist <= approach_target_dist:
            self.get_logger().info(f"Approach complete (Dist: {outer_wall_dist:.2f}m). Transitioning to PASS_THROUGH.")
            self.straight_sub_state = StraightSubState.AVOID_OUTER_PASS_THROUGH
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # P-control to maintain the approach angle
        error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        angular_z = self.avoid_turn_in_kp_angle * error_deg
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)
        
        self.get_logger().debug(
            f"AVOID_APPROACH | TargetYaw: {target_yaw_deg:.1f}, "
            f"CurrentYaw: {self.current_yaw_deg:.1f}, "
            f"WallDist: {outer_wall_dist:.2f}m | Steer: {final_steer:.2f}",
            throttle_duration_sec=0.2
        )
        
        self.publish_twist_with_gain(self.avoid_speed, final_steer)

    def _handle_straight_sub_avoid_outer_pass_through(self, msg: LaserScan):
        if self._check_for_finish_condition(msg):
            return

        """Phase 2: Move straight, parallel to the wall, until the obstacle is passed."""
        base_angle_deg = self._calculate_base_angle()

        if self.direction == 'ccw':
            inner_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
        else: # cw
            inner_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
        outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
        
        path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='outer')

        # --- Completion Check ---
        if self.is_passing_obstacle:
            if not math.isnan(inner_wall_dist) and inner_wall_dist > self.avoid_inner_pass_thresh_m:
                
                # --- REFACTORED: Use helper for the completion logic ---
                if path_type == 'outer_to_inner':
                    self.get_logger().info("--- OUTER part of [outer_to_inner] complete. Transitioning to INNER part. ---")
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.AVOID_INNER_TURN_IN,
                        path_was_outer=True,
                        enable_next_turn=False)
                else:
                    self.get_logger().info("--- AVOIDANCE COMPLETE (obstacle passed) ---")
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.ALIGN_WITH_OUTER_WALL,
                        path_was_outer=True,
                        enable_next_turn=True)
                return
        else:
            is_oriented_correctly = self._check_yaw_alignment(base_angle_deg, 45.0)
            sum_side_walls_dist = inner_wall_dist + outer_wall_dist
            if(not math.isnan(inner_wall_dist) and not math.isnan(sum_side_walls_dist) and 
               inner_wall_dist < self.avoid_inner_pass_thresh_m and sum_side_walls_dist < 0.7 and is_oriented_correctly):
                self.get_logger().warn(">>> Passing obstacle now...")
                self.is_passing_obstacle = True

        # --- Driving Control Logic (kept separate for clarity) ---
        if math.isnan(outer_wall_dist):
            self.get_logger().error("Lost sight of outer wall during pass_through. Stopping.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(0.0, 0.0)
            return

        if self.wall_segment_index == 0:
            target_dist = self.align_target_outer_dist_start_area_m
        else:
            target_dist = self.align_target_outer_dist_m


        angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg

        dist_steer = 0.0
        dist_error = target_dist - outer_wall_dist
        
        if abs(dist_error) > self.align_dist_tolerance_m:
            direction_multiplier = 1.0 if self.direction == 'ccw' else -1.0
            dist_steer = self.align_kp_dist * dist_error * direction_multiplier

        angular_z = angle_steer + dist_steer
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)

        self.get_logger().debug(
            f"AVOID_PASS | Pass: {self.is_passing_obstacle} | "
            f"OuterD: {outer_wall_dist:.2f} (Err: {dist_error:.2f}) | "
            f"InnerD: {inner_wall_dist:.2f} | "
            f"YawErr: {angle_error_deg:.1f} | Steer: {final_steer:.2f}",
            throttle_duration_sec=0.2
        )
        self.publish_twist_with_gain(self.avoid_speed, final_steer)

    def _handle_straight_sub_avoid_inner_turn_in(self, msg: LaserScan):
        """Phase 1: Approach the INNER wall at a fixed angle.
        Completion is determined by the distance to the OUTER wall.
        """
        if self._check_for_finish_condition(msg):
            return

        base_angle_deg = self._calculate_base_angle()
        
        if self.direction == 'ccw':
            target_yaw_deg = self._angle_normalize(base_angle_deg + self.avoid_inner_approach_angle_deg)
            outer_wall_angle_for_approach_check = self._angle_normalize(base_angle_deg - 90.0)
            inner_wall_angle_for_pass_check = self._angle_normalize(base_angle_deg + 90.0)
        else: # cw
            target_yaw_deg = self._angle_normalize(base_angle_deg - self.avoid_inner_approach_angle_deg)
            outer_wall_angle_for_approach_check = self._angle_normalize(base_angle_deg + 90.0)
            inner_wall_angle_for_pass_check = self._angle_normalize(base_angle_deg - 90.0)
        
        outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle_for_approach_check)
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle_for_pass_check)
        
        # Monitor for passing the obstacle during the turn-in phase
        path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='inner')

        if self.is_passing_obstacle:
            # Check if we have completed the pass MANEUVER during the turn-in
            if(not math.isnan(inner_wall_dist) and not math.isnan(outer_wall_dist) and 
                outer_wall_dist < self.avoid_inner_pass_thresh_m):
                self.get_logger().warn("!!! Obstacle passed completely during INNER TURN_IN phase. Skipping PASS_THROUGH. !!!")
                if path_type == 'inner_to_outer':
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.AVOID_OUTER_TURN_IN,
                        path_was_outer=False,
                        enable_next_turn=False)
                else:
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.ALIGN_WITH_INNER_WALL,
                        path_was_outer=False,
                        enable_next_turn=True)
                return
        else:
            # Check if we have started passing the obstacle (based on outer wall getting close)
            is_oriented_correctly = self._check_yaw_alignment(base_angle_deg, 60.0)
            sum_side_walls_dist = inner_wall_dist + outer_wall_dist
            if(not math.isnan(outer_wall_dist) and not math.isnan(inner_wall_dist) 
               and outer_wall_dist < self.avoid_inner_pass_thresh_m and sum_side_walls_dist < 0.7 and 
               is_oriented_correctly):
                self.get_logger().warn(">>> Passing obstacle now (during INNER TURN_IN)...")
                self.is_passing_obstacle = True

        # Original approach logic
        if math.isnan(outer_wall_dist):
            self.get_logger().error("Lost sight of outer wall during inner approach. But Not Stopping.", throttle_duration_sec=1.0)
            # self.publish_twist_with_gain(0.0, 0.0)
            # Do not return here, let the P-control continue to steer towards the target yaw
        
        effective_dist = 1.0 - outer_wall_dist if not math.isnan(outer_wall_dist) else inner_wall_dist # float('inf')

        self.get_logger().debug(
            f"AVOID_INNER_APPROACH | TargetEffDist: {self.avoid_inner_approach_target_dist_m:.2f}, "
            f"CurrentEffDist: {effective_dist:.2f} (OuterWall: {outer_wall_dist:.2f}m)",
            throttle_duration_sec=0.2
        )
        
        # --- Added condition to prevent premature completion ---
        # The approach is complete only if the effective distance is within a valid range.
        if effective_dist >= 0 and effective_dist <= self.avoid_inner_approach_target_dist_m:
            self.get_logger().info(f"Approach complete (EffectiveDist: {effective_dist:.2f}m). Transitioning to PASS_THROUGH.")
            self.straight_sub_state = StraightSubState.AVOID_INNER_PASS_THROUGH
            self.publish_twist_with_gain(0.0, 0.0) # Stop briefly
            return

        # P-control to maintain the approach angle
        error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        angular_z = self.avoid_turn_in_kp_angle * error_deg
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)
        
        self.publish_twist_with_gain(self.avoid_speed, final_steer)

    def _handle_straight_sub_avoid_inner_pass_through(self, msg: LaserScan):
        if self._check_for_finish_condition(msg):
            return
        """Phase 2: Move straight, parallel to the wall, using conditional distance estimation."""
        base_angle_deg = self._calculate_base_angle()
        
        if self.direction == 'ccw':
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
        else: # cw
            outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            
        front_wall_dist = self.get_distance_at_world_angle(msg, base_angle_deg)
        inner_wall_dist_raw = self.get_distance_at_world_angle(msg, base_angle_deg + (90.0 if self.direction == 'ccw' else -90.0))
        outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)

        path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='inner')

        # --- Completion Check ---
        if self.is_passing_obstacle:
            if not math.isnan(outer_wall_dist) and outer_wall_dist > self.avoid_inner_pass_thresh_m:
                
                # --- REFACTORED: Use helper for the completion logic ---
                if path_type == 'inner_to_outer':
                    self.get_logger().info("--- INNER part of [inner_to_outer] complete. Transitioning to OUTER part. ---")
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.AVOID_OUTER_TURN_IN,
                        path_was_outer=False,
                        enable_next_turn=False)
                else: # 'inner' or the final part of 'outer_to_inner'
                    self.get_logger().info("--- AVOIDANCE COMPLETE (obstacle passed) ---")
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.ALIGN_WITH_INNER_WALL,
                        path_was_outer=False,
                        enable_next_turn=True)
                return
        else:
            is_oriented_correctly = self._check_yaw_alignment(base_angle_deg, 60.0)
            if not math.isnan(inner_wall_dist_raw) and not math.isnan(outer_wall_dist) :
                sum_side_walls_dist = inner_wall_dist_raw + outer_wall_dist
            else:
                sum_side_walls_dist = 1.0
            if(not math.isnan(inner_wall_dist_raw) and 
               outer_wall_dist < self.avoid_inner_pass_thresh_m and 
               (sum_side_walls_dist < 0.7 or (sum_side_walls_dist > 1.3 and front_wall_dist > 1.9) ) and 
               is_oriented_correctly):
                self.get_logger().warn(">>> Passing obstacle now (monitoring outer wall)...")
                self.is_passing_obstacle = True

        # --- Driving Control Logic (kept separate due to its unique complexity) ---
        dist_error = 0.0
        log_mode = "NORMAL"

        if math.isnan(inner_wall_dist_raw) or inner_wall_dist_raw > 1.0 or (not math.isnan(front_wall_dist) and front_wall_dist > 2.05):
            if not math.isnan(outer_wall_dist):
                effective_inner_dist = 1.0 - outer_wall_dist
                dist_error = self.align_target_inner_dist_m - effective_inner_dist
                log_mode = f"ESTIMATED (from Outer: {outer_wall_dist:.2f}m)"
            else:
                dist_error = 0.0
                log_mode = "IMU_ONLY (no valid wall)"
        elif not math.isnan(front_wall_dist) and front_wall_dist > 1.95:
            dist_error = 0.0
            log_mode = "IMU_ONLY (blind zone)"
        else:
            dist_error = self.align_target_inner_dist_m - inner_wall_dist_raw
            log_mode = "NORMAL"

        angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg
        dist_steer = 0.0
        if abs(dist_error) > self.align_dist_tolerance_m:
            direction_multiplier = -1.0 if self.direction == 'ccw' else 1.0
            dist_steer = self.align_kp_dist * dist_error * direction_multiplier
        angular_z = angle_steer + dist_steer
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)
        self.get_logger().debug(
            f"AVOID_INNER_PASS | Mode: {log_mode} | Pass: {self.is_passing_obstacle} | "
            f"InnerD_Raw: {inner_wall_dist_raw:.2f} | OuterD: {outer_wall_dist:.2f} | F_Dist: {front_wall_dist:.2f} | "
            f"YawErr: {angle_error_deg:.1f} | Steer: {final_steer:.2f}",
            throttle_duration_sec=0.2
        )
        self.publish_twist_with_gain(self.avoid_speed, final_steer)

    # --- Turning Sub-State Handlers ---
    def _handle_turning_approach(self, msg):
        """
        Sub-state: Approach the front wall.
        - For inner path entries, dynamically adjust the approach angle based on front distance.
        - For outer path entries, use a fixed approach angle.
        """
        front_wall_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)
        target_dist = self.approach_target_dist # This is set in _prepare_for_turning

        if math.isnan(front_wall_dist):
            # Move slowly straight for safety if distance is unknown
            self.get_logger().warn("Cannot get front distance for approach, moving slowly straight.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(self.course_detection_slow_speed * 0.5, 0.0)
            return

        # --- Completion Check (Common for both approach types) ---
        if front_wall_dist < target_dist:
            self.get_logger().info(f'Wall is close ({front_wall_dist:.2f}m < {target_dist:.2f}m). Starting REVERSE.')
            self.turning_sub_state = TurningSubState.REVERSE
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # --- Dynamic vs Fixed Angle Calculation ---
        direction_multiplier = 1.0 if self.direction == 'ccw' else -1.0
        
        # --- START OF CORE LOGIC ---
        if not self.last_avoidance_path_was_outer:
            # --- DYNAMIC APPROACH from INNER path (Lane Change Logic) ---
            start_angle = self.dynamic_approach_start_angle_deg
            start_dist = self.dynamic_approach_start_dist_m
            
            # Define the distance at which the robot should be straight
            straighten_dist = target_dist + self.dynamic_approach_straighten_offset_m

            # Calculate progress from 0.0 to 1.0 based on distance
            # Normalize so that progress is 0.0 at start_dist and 1.0 at straighten_dist
            if (start_dist - straighten_dist) > 0.001: # Avoid division by zero
                progress = (start_dist - front_wall_dist) / (start_dist - straighten_dist)
            else:
                progress = 1.0 # If start is already within straightening zone, consider it done
            
            progress = max(0.0, min(1.0, progress)) # Clamp progress between 0.0 and 1.0

            # Linearly interpolate the target angle using progress
            # progress=0 -> start_angle, progress=1 -> 0.0
            # Apply direction multiplier HERE to handle CW/CCW correctly
            current_target_angle_deg = (start_angle * (1.0 - progress)) * direction_multiplier
            
            log_approach_type = "DYNAMIC"

        else:
            # --- FIXED APPROACH from OUTER path (Conventional Logic) ---
            # self.approach_target_angle_deg is already correctly set with a multiplier in _prepare_for_turning
            current_target_angle_deg = self.approach_target_angle_deg
            log_approach_type = "FIXED"
        # --- END OF CORE LOGIC ---

        # For the fixed case, the multiplier is already applied.
        # For the dynamic case, it's applied above. So this variable is the final target.
        final_target_angle_deg = current_target_angle_deg
        
        # --- P-Control Execution (Common for both) ---
        target_yaw = self.approach_base_yaw_deg + final_target_angle_deg
        error_deg = self._angle_diff(target_yaw, self.current_yaw_deg)
        angular_vel = self.kp_approach_angle * error_deg
        angular_vel_clamped = max(min(angular_vel, self.max_steer), -self.max_steer)
        
        self.get_logger().debug(
            f"APP({log_approach_type}) | "
            f"Base: {self.approach_base_yaw_deg:6.1f}, "
            f"TgtRel: {final_target_angle_deg:5.1f} -> "
            f"TgtAbs: {target_yaw:6.1f} | "
            f"CurYaw: {self.current_yaw_deg:6.1f} -> "
            f"Err: {error_deg:6.1f} | "
            f"F_Dist: {front_wall_dist:.2f}m (Tgt: {target_dist:.2f}m)",
            throttle_duration_sec=0.2
        )
        
        self.publish_twist_with_gain(self.forward_speed, angular_vel_clamped)

    def _handle_turning_reverse(self):
        """Sub-state: Reverse with steering until a target yaw is reached."""
        yaw_error = self._angle_diff(self.reverse_target_yaw_deg, self.current_yaw_deg)
        self.get_logger().debug(f'TURNING (REVERSE): Yaw Error: {yaw_error:.1f} deg to target {self.reverse_target_yaw_deg:.1f}', throttle_duration_sec=0.2)
        
        if abs(yaw_error) < self.reverse_yaw_tolerance_deg:
            self.get_logger().info('Reverse maneuver complete. Now checking for obstacles before proceeding.')
            # Transition to the final check sub-state
            self.turning_sub_state = TurningSubState.CHECK_AND_FINISH
            self.publish_twist_with_gain(0.0, 0.0)
        else:
            steer_direction = 1.0 if self.direction == 'ccw' else -1.0
            self.publish_twist_with_gain(-self.forward_speed, self.turn_reverse_steer * steer_direction)

    def _handle_turning_sub_check_and_finish(self, msg: LaserScan):
        """
        Sub-state: Finalizes the turn. Obstacle detection is temporarily disabled.
        """
        # Obstacle detection logic is removed. Assume the path is clear.
        self.get_logger().info("No obstacle detected (detection disabled). Proceeding to finalize turn.")
        self.turning_sub_state = TurningSubState.FINALIZE_TURN

    def _handle_turning_sub_creating_space(self, msg: LaserScan):
        """
        Sub-state: The robot reverses straight back until the detected
        obstacle is at a safe distance.
        """
        current_obstacle_info = None

        stop_reversing = False
        if current_obstacle_info is None:
            self.get_logger().info("Obstacle is no longer detected. Re-checking.")
            stop_reversing = True
        elif current_obstacle_info['distance_m'] >= self.obstacle_clearance_dist_m:
            self.get_logger().info(f"Clearance achieved ({current_obstacle_info['distance_m']:.2f}m). Re-checking.")
            stop_reversing = True
        
        if stop_reversing:
            # Go back to the check state to ensure the path is clear before proceeding
            self.turning_sub_state = TurningSubState.CHECK_AND_FINISH
            self.publish_twist_with_gain(0.0, 0.0)
        else:
            self.get_logger().warn(
                f"Creating space... Obstacle at {current_obstacle_info['distance_m']:.2f}m. Reversing.", 
                throttle_duration_sec=0.5
            )
            self.publish_twist_with_gain(-self.forward_speed, 0.0)

    def _handle_turning_sub_finalize_turn(self):
        """
        Finalizes the turn and transitions to the avoidance sequence
        based on the avoidance_path_plan for the next segment.
        """
        self.turn_count += 1
        self.wall_segment_index = (self.wall_segment_index + 1) % 4 
        self.get_logger().info(f"Turn {self.turn_count} complete. Entering new segment index: {self.wall_segment_index}")

        self.state = State.STRAIGHT
        self.planning_initiated = False 

        # --- REFACTORED: Use a flexible helper to get the path type for the NEXT segment ---
        next_path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='outer')

        if next_path_type == 'inner' or next_path_type == 'inner_to_outer':
            self.get_logger().warn(f"Plan for segment {self.wall_segment_index}: Starts INNER. Transitioning to AVOID_INNER_TURN_IN.")
            self.straight_sub_state = StraightSubState.AVOID_INNER_TURN_IN
        else: # 'outer', 'outer_to_inner', or any other unexpected string
            log_start_type = "OUTER" if next_path_type in ['outer', 'outer_to_inner'] else f"UNKNOWN('{next_path_type}'), defaulting to OUTER"
            self.get_logger().warn(f"Plan for segment {self.wall_segment_index}: Starts {log_start_type}. Transitioning to AVOID_OUTER_TURN_IN.")
            self.straight_sub_state = StraightSubState.AVOID_OUTER_TURN_IN
        
        self.is_passing_obstacle = False
        self.inner_wall_far_counter = 0 
        
        self.turning_sub_state = None
        self.publish_twist_with_gain(0.0, 0.0)

    def _set_camera_angle(self, pan_position: int, tilt_position: int = 1500, duration_sec: float = 0.5):
        """
        Sends a command to set the pan and tilt servos to a specific position.
        
        Args:
            pan_position: The target position for the pan servo.
            tilt_position: The target position for the tilt servo (default is level).
            duration_sec: The time duration for the servo movement.
        """
        self.get_logger().info(f"Setting camera pan to {pan_position}, tilt to {tilt_position}.")
        
        msg = SetPWMServoState()
        msg.duration = duration_sec
        
        pan_state = PWMServoState()
        pan_state.id = [self.pan_servo_id]
        pan_state.position = [pan_position]

        tilt_state = PWMServoState()
        tilt_state.id = [self.tilt_servo_id]
        tilt_state.position = [tilt_position]

        msg.state = [pan_state, tilt_state]

        self.servo_pub.publish(msg)

    # --- Logic and Calculation Helper Functions ---
    def _execute_pid_alignment(self, msg: LaserScan, base_angle_deg: float, is_outer_wall: bool):
        """
        A generic PID controller for aligning the robot parallel to a specified wall.
        This function is called by the specific align_with_*_wall handlers.
        """
        if is_outer_wall:
            wall_offset_deg = -90.0 if self.direction == 'ccw' else 90.0
            if self.wall_segment_index == 0:
                target_dist = self.align_target_outer_dist_start_area_m
            else:
                target_dist = self.align_target_outer_dist_m

            dist_steer_multiplier = 1.0 if self.direction == 'ccw' else -1.0
            log_prefix = "ALIGN_OUTER"
        else: # Inner wall
            wall_offset_deg = 90.0 if self.direction == 'ccw' else -90.0
            target_dist = self.align_target_inner_dist_m
            dist_steer_multiplier = -1.0 if self.direction == 'ccw' else 1.0
            log_prefix = "ALIGN_INNER"

        wall_angle = self._angle_normalize(base_angle_deg + wall_offset_deg)
        wall_dist = self.get_distance_at_world_angle(msg, wall_angle)
        
        angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg

        dist_steer = 0.0
        if not math.isnan(wall_dist):
            if not is_outer_wall and wall_dist > 1.0:
                dist_error = 0.0
            else:
                dist_error = target_dist - wall_dist
            
            if abs(dist_error) > self.align_dist_tolerance_m:
                dist_steer = self.align_kp_dist * dist_error * dist_steer_multiplier

        angular_z = angle_steer + dist_steer
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)
        
        self.get_logger().debug(
            f"{log_prefix} | WallD: {wall_dist:.2f} | "
            f"YawErr: {angle_error_deg:.1f} | AngleSteer: {angle_steer:.2f} | DistSteer: {dist_steer:.2f} | "
            f"FinalSteer: {final_steer:.2f}",
            throttle_duration_sec=0.2
        )
        
        self.publish_twist_with_gain(self.forward_speed, final_steer)

    def _execute_avoid_turn_in(self, msg: LaserScan, is_outer_turn_in: bool):
        """
        Generic executor for the 'turn_in' phase of an avoidance maneuver.
        It approaches either the inner or outer wall at a fixed angle.

        Args:
            msg: The current LaserScan message.
            is_outer_turn_in: True to execute the turn-in towards the outer wall,
                            False for the inner wall.
        """
        base_angle_deg = self._calculate_base_angle()

        if is_outer_turn_in:
            target_approach_angle = self.avoid_approach_angle_deg
            wall_offset_deg = -90.0
            target_dist = self.avoid_approach_target_dist_m
            next_sub_state = StraightSubState.AVOID_OUTER_PASS_THROUGH
        else: # Inner turn-in
            target_approach_angle = self.avoid_inner_approach_angle_deg
            wall_offset_deg = 90.0 # Completion check is against the outer wall
            target_dist = self.avoid_inner_approach_target_dist_m
            next_sub_state = StraightSubState.AVOID_INNER_PASS_THROUGH

        direction_multiplier = 1.0 if self.direction == 'ccw' else -1.0
        target_yaw_deg = self._angle_normalize(base_angle_deg - (target_approach_angle * direction_multiplier))
        wall_angle_for_check = self._angle_normalize(base_angle_deg + (wall_offset_deg * direction_multiplier))

        check_wall_dist = self.get_distance_at_world_angle(msg, wall_angle_for_check)
        
        if math.isnan(check_wall_dist):
            self.get_logger().error(f"Lost sight of wall during AVOID_TURN_IN. Stopping.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(0.0, 0.0)
            return

        completion_met = False
        if is_outer_turn_in:
            completion_met = (check_wall_dist <= target_dist)
        else: # Inner turn-in's completion is based on effective distance
            effective_dist = 1.0 - check_wall_dist
            completion_met = (effective_dist <= target_dist)

        if completion_met:
            self.get_logger().info(f"AVOID_TURN_IN complete. Transitioning to PASS_THROUGH.")
            self.straight_sub_state = next_sub_state
            self.publish_twist_with_gain(0.0, 0.0)
            return

        error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        angular_z = self.avoid_turn_in_kp_angle * error_deg
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)
        
        self.publish_twist_with_gain(self.avoid_speed, final_steer)

    def _on_camera_settled_for_planning(self):
        """
        Callback triggered after the camera is assumed to have settled.
        This re-enables frame updates and sets a flag to wait for the next frame.
        """
        if self.planning_camera_wait_timer:
            self.planning_camera_wait_timer.destroy()
            self.planning_camera_wait_timer = None
        
        self.get_logger().info("Camera settled. Resuming frame updates and waiting for first safe frame.")
        self.accept_new_frames = True
        self.waiting_for_first_safe_frame = True

    def _detect_obstacle_color_in_frame(self, frame_rgb, rois_to_check=None):
        """
        Calculates the area of RED and GREEN pixels within specified ROIs.
        Returns a dictionary with raw data for debugging and further processing.

        Args:
            frame_rgb: The input RGB image frame.
            rois_to_check (dict, optional): A dict of ROIs to check, e.g.,
                                            {'left': [[...]], 'right': [[...]]}.
                                            If None, the entire image is processed.

        Returns:
            A dictionary containing processed images and detection results.
            e.g., {
                'frame_bgr': a_cv2_image,
                'masks': {'RED': red_mask, 'GREEN': green_mask},
                'areas': {'roi_name': {'RED': 123, 'GREEN': 456}, 'full_frame': ...}
            }
        """
        if frame_rgb is None:
            self.get_logger().error("Cannot detect color, frame is None.", throttle_duration_sec=5.0)
            return None
            
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        # Create base color masks for the entire image
        red_mask1 = self._calculate_color_area(hsv_frame, self.red_lower1, self.red_upper1)
        red_mask2 = self._calculate_color_area(hsv_frame, self.red_lower2, self.red_upper2)
        full_red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        full_green_mask = self._calculate_color_area(hsv_frame, self.green_lower, self.green_upper)

        areas = {}
        
        # --- Always calculate full_frame area ---
        red_area_full = cv2.countNonZero(full_red_mask)
        green_area_full = cv2.countNonZero(full_green_mask)
        areas['full_frame'] = {'RED': red_area_full, 'GREEN': green_area_full}

        if rois_to_check:
            for roi_name, roi_points in rois_to_check.items():
                roi_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
                roi_corners = np.array([roi_points], dtype=np.int32)
                cv2.fillPoly(roi_mask, roi_corners, 255)
                
                red_area_roi = cv2.countNonZero(cv2.bitwise_and(full_red_mask, roi_mask))
                green_area_roi = cv2.countNonZero(cv2.bitwise_and(full_green_mask, roi_mask))
                
                areas[roi_name] = {'RED': red_area_roi, 'GREEN': green_area_roi}
        
        return {
            'frame_bgr': frame_bgr,
            'masks': {'RED': full_red_mask, 'GREEN': full_green_mask},
            'areas': areas
        }

    def _update_avoidance_plan_based_on_vision(self, is_red_present, is_green_present, final_red_area, final_green_area):
        """
        Determines the next avoidance path based on visual detection and updates the plan.
        """
        found_obstacle = 'none'
        if is_red_present and is_green_present:
            found_obstacle = 'red_to_green' if final_red_area >= final_green_area else 'green_to_red'
        elif is_red_present:
            found_obstacle = 'red_only'
        elif is_green_present:
            found_obstacle = 'green_only'
        else:
            self.get_logger().warn("No obstacles found, defaulting to 'red_only' behavior (outer for CCW, inner for CW).")
            found_obstacle = 'red_only' # Safe default

        # Map obstacle configuration to path type based on driving direction
        if self.direction == 'ccw':
            path_mapping = {
                'red_only': 'outer',
                'green_only': 'inner',
                'red_to_green': 'outer_to_inner',
                'green_to_red': 'inner_to_outer'
            }
        else: # cw
            path_mapping = {
                'red_only': 'inner',
                'green_only': 'outer',
                'red_to_green': 'inner_to_outer',
                'green_to_red': 'outer_to_inner'
            }
        
        next_avoidance_path = path_mapping[found_obstacle]
        
        next_segment_index = (self.wall_segment_index + 1) % len(self.avoidance_path_plan)
        self.avoidance_path_plan[next_segment_index] = next_avoidance_path

    def _prepare_for_turning(self, base_angle_deg, specific_angle_deg=None, specific_target_dist=None):
        """
        Calculates all necessary turn parameters and transitions state to TURNING.
        Uses special parameters if turning from the start area segment's outer path.
        """
        self.approach_base_yaw_deg = base_angle_deg
        log_msg = ""
        direction_multiplier = 1.0 if self.direction == 'ccw' else -1.0

        if specific_angle_deg is not None:
            self.approach_target_angle_deg = specific_angle_deg
            self.approach_target_dist = specific_target_dist if specific_target_dist is not None else self.turn_approach_dist_from_outer_path
            log_msg = f"Using specific approach: Angle {self.approach_target_angle_deg:.1f} deg, Dist {self.approach_target_dist:.2f} m."
        
        else: # Standard turn logic for all other cases
                
            if self.last_avoidance_path_was_outer:
                if self.wall_segment_index == 0:
                    angle_magnitude = self.turn_approach_angle_from_outer_path_and_start_area_deg
                    self.approach_target_dist = self.turn_approach_dist_from_outer_path_and_start_area_m
                else:
                    angle_magnitude = self.turn_approach_angle_from_outer_path_deg
                    self.approach_target_dist = self.turn_approach_dist_from_outer_path
                self.approach_target_angle_deg = angle_magnitude * direction_multiplier
                log_msg = f"Last path was OUTER. Approaching INNER at {self.approach_target_angle_deg:.1f} deg."
            else:
                angle_magnitude = self.turn_approach_angle_from_inner_path_deg
                self.approach_target_angle_deg = angle_magnitude * direction_multiplier
                self.approach_target_dist = self.turn_approach_dist_from_inner_path
                log_msg = f"Last path was INNER. Approaching OUTER at {self.approach_target_angle_deg:.1f} deg."

        self.reverse_target_yaw_deg = self._calculate_reverse_yaw(base_angle_deg)
        
        self.get_logger().info("=====================================================")
        self.get_logger().info("===            TURN PREPARATION - DEBUG           ===")
        self.get_logger().info(f"Base Angle: {base_angle_deg:.2f} deg, Info: {log_msg}")
        self.get_logger().info(f"Approach Tgt Angle: {self.approach_target_angle_deg:.2f} deg")
        self.get_logger().info(f"Approach Tgt Dist: {self.approach_target_dist:.2f} m")
        self.get_logger().info(f"Reverse Tgt Yaw: {self.reverse_target_yaw_deg:.2f} deg")
        self.get_logger().info("=====================================================")

        self.state = State.TURNING
        self.turning_sub_state = TurningSubState.APPROACH
        self.inner_wall_far_counter = 0
        self.can_start_new_turn = False
        self.publish_twist_with_gain(0.0, 0.0)

    def _complete_avoidance_phase(self, next_sub_state: StraightSubState, path_was_outer: bool, enable_next_turn: bool):
        """
        A generic helper function to finalize an avoidance phase (e.g., pass_through)
        and transition to the next state. It centralizes the resetting of state variables.

        Args:
            next_sub_state: The StraightSubState to transition into.
            path_was_outer: A boolean indicating if the completed path was on the outer side.
            enable_next_turn: A boolean to set whether a new corner detection is allowed.
        """
        self.is_passing_obstacle = False
        self.last_avoidance_path_was_outer = path_was_outer
        self.straight_sub_state = next_sub_state

        if enable_next_turn:
            self.can_start_new_turn = True
            self.get_logger().warn("Segment avoidance complete. New turn detection is ENABLED.")
        
        self.publish_twist_with_gain(0.0, 0.0)

    def _check_for_finish_condition(self, msg: LaserScan) -> bool:
        """
        Checks if the robot should transition to the FINISHED state.
        This is called from all straight-driving sub-states.
        """
        if self.turn_count >= self.max_turns:
            base_angle_deg = self._calculate_base_angle()
            front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)
            
            if self.direction == 'ccw':
                inner_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
                outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            else: # cw
                inner_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
                outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
            outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)

            sum_side_walls_dist = inner_wall_dist + outer_wall_dist
            
            # --- Use the new helper function for the check ---
            is_oriented_correctly = self._check_yaw_alignment(base_angle_deg, 30.0)
            
            if (not math.isnan(front_dist) and front_dist < 1.5 and front_dist > 1.0 and
                not math.isnan(inner_wall_dist) and inner_wall_dist < 1.0 and 
                not math.isnan(sum_side_walls_dist) and sum_side_walls_dist < 1.1 and sum_side_walls_dist > 0.4 and
                is_oriented_correctly # Using the result of the new function
                ):
                self.get_logger().warn(
                    f"FINISH CONDITION MET: Turn count ({self.turn_count}) >= max_turns ({self.max_turns}) "
                    f"AND FrontDist ({front_dist:.2f}m) is within range "
                    f"AND InnerDist ({inner_wall_dist:.2f}m) < 1.0m."
                )
                self.state = State.FINISHED
                self.publish_twist_with_gain(0.0, 0.0)
                return True
            
            # --- Updated debug log ---
            elif (not math.isnan(front_dist) and front_dist < 1.5 and front_dist > 1.0 and
                    not math.isnan(inner_wall_dist) and inner_wall_dist < 1.0):
                if not is_oriented_correctly:
                    # Recalculate deviation here just for the log message
                    yaw_deviation_deg = abs(self._angle_diff(self.current_yaw_deg, base_angle_deg))
                    self.get_logger().debug(f"Finish conditions met, but yaw deviation {yaw_deviation_deg:.1f}deg is > 45deg. Not finishing.",
                                            throttle_duration_sec=1.0)

        return False

    def _check_course_and_get_direction(self, msg: LaserScan) -> tuple[bool, str]:
        """
        Checks left and right wall distances to determine course direction.
        
        Returns:
            A tuple containing:
            - bool: True if the course direction was found, False otherwise.
            - str: The determined direction ('cw', 'ccw', or an empty string).
        """
        left_wall_angle = self._angle_normalize(0.0 + 90.0)
        right_wall_angle = self._angle_normalize(0.0 - 90.0)

        left_dist = self.get_distance_at_world_angle(msg, left_wall_angle)
        right_dist = self.get_distance_at_world_angle(msg, right_wall_angle)
        
        self.get_logger().debug(
            f"Checking distances -> Left: {left_dist:.2f}m, Right: {right_dist:.2f}m",
            throttle_duration_sec=0.5
        )

        if not math.isnan(left_dist) and left_dist > self.course_detection_threshold_m:
            # Set the node's direction parameter immediately
            self.direction = "ccw"
            if not self.initial_path_is_left or self.initial_position_is_near:
                self.last_avoidance_path_was_outer = True
            else:
                self.last_avoidance_path_was_outer = False
            
            self.get_logger().warn(f"Left side is open (dist: {left_dist:.2f}m). Course is CCW.")
            return True, "ccw"
        
        if not math.isnan(right_dist) and right_dist > self.course_detection_threshold_m:
            # Set the node's direction parameter immediately
            self.direction = "cw"
            if self.initial_path_is_left or self.initial_position_is_near:
                self.last_avoidance_path_was_outer = True
            else:
                self.last_avoidance_path_was_outer = False
            
            self.get_logger().warn(f"Right side is open (dist: {right_dist:.2f}m). Course is CW.")
            return True, "cw"
            
        return False, ""

    def _check_for_corner(self, msg: LaserScan, base_angle_deg: float) -> tuple[bool, float]:
        """
        Checks for a corner. If the path plan is not yet complete, it transitions 
        to the planning state. Otherwise, it prepares for a normal turn.
        """
        if self.direction == 'ccw':
            inner_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
        else: # cw
            inner_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
        
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)

        # --- MODIFIED CORNER DETECTION LOGIC ---
        if not math.isnan(inner_wall_dist):
            if inner_wall_dist >= self.inner_wall_disappear_threshold:
                self.inner_wall_far_counter += 1
            else:
                # Reset counter only if the wall is confirmed to be close
                self.inner_wall_far_counter = 0
        # If inner_wall_dist is nan, do nothing and keep the counter's value.
        # This makes the detection robust against sporadic measurement failures.

        if self.inner_wall_far_counter >= self.inner_wall_disappear_count and self.can_start_new_turn:
            # --- Check if planning is still required ---
            is_planning_complete = '' not in self.avoidance_path_plan
            
            if not is_planning_complete:
                self.get_logger().info("Corner detected. Transitioning to PRE_PLANNING_ADJUST.")
                self.straight_sub_state = StraightSubState.PRE_PLANNING_ADJUST
                self.min_inner_dist_planning_approach = float('inf')
                self.planning_scan_complete = False
                self.publish_twist_with_gain(0.0, 0.0) # Stop briefly before adjusting
                self.approach_base_yaw_deg = base_angle_deg 
            else:
                # --- Planning is complete, decide whether to turn or skip ---
                if self._is_turn_maneuver_required():
                    # Turn maneuver is necessary
                    self.get_logger().info("Corner detected. Turn maneuver is required. Preparing for turn.")
                    self._prepare_for_turning(base_angle_deg)
                else:
                    # Turn maneuver can be skipped
                    self.get_logger().info("Corner detected. Skipping turn maneuver.")
                    self.state = State.TURNING
                    self.turning_sub_state = TurningSubState.FINALIZE_TURN
                    self.publish_twist_with_gain(0.0, 0.0) # Stop briefly before adjusting

            return True, inner_wall_dist
            
        elif self.inner_wall_far_counter >= self.inner_wall_disappear_count and not self.can_start_new_turn:
            self.get_logger().debug("Corner condition met, but waiting for turn permission.", throttle_duration_sec=1.0)
        
        return False, inner_wall_dist

    def _check_yaw_alignment(self, base_angle_deg: float, tolerance_deg: float) -> bool:
        """
        Checks if the current yaw is within a tolerance of a base angle.

        Args:
            base_angle_deg: The reference angle to compare against.
            tolerance_deg: The maximum allowed deviation in degrees.

        Returns:
            bool: True if the yaw is within the tolerance, False otherwise.
        """
        # Calculate the absolute deviation from the base angle
        yaw_deviation_deg = abs(self._angle_diff(self.current_yaw_deg, base_angle_deg))

        # Return True if the deviation is less than the tolerance
        return yaw_deviation_deg < tolerance_deg

    def _get_path_type_for_segment(self, segment_index: int, default_path: str = 'outer') -> str:
        """
        Retrieves the planned path type for a specific wall segment from the plan.

        Returns:
            The planned path type (e.g., 'inner', 'outer_to_inner') as a string.
        """
        try:
            num_segments_in_plan = len(self.avoidance_path_plan)
            path_type = self.avoidance_path_plan[segment_index % num_segments_in_plan]
            return path_type
        except (IndexError, TypeError, AttributeError):
            self.get_logger().error(f"Invalid or missing avoidance_path_plan. Defaulting to '{default_path}'.")
            return default_path

    def _calculate_base_angle(self):
        """Calculates the ideal angle of the current wall segment based on direction."""
        if self.direction == 'ccw':
            return self.wall_segment_index * 90.0
        else: # cw
            return self.wall_segment_index * -90.0

    def _calculate_reverse_yaw(self, base_angle_deg):
        """Calculates the target yaw for the end of the reverse maneuver."""
        if self.direction == 'ccw':
            target_yaw = base_angle_deg + 90.0
        else: # cw
            target_yaw = base_angle_deg - 90.0
        
        # Normalize to -180 to 180 range
        while target_yaw > 180.0: target_yaw -= 360.0
        while target_yaw <= -180.0: target_yaw += 360.0
        return target_yaw

    def _calculate_color_area(self, hsv_image, lower_bound, upper_bound, points=None):
        """
        Creates a binary mask for a specific color within a given region.

        Args:
            hsv_image: The input image in HSV color space.
            lower_bound: The lower bound of the color range.
            upper_bound: The upper bound of the color range.
            points (optional): A list of 4 points for a quadrilateral ROI.

        Returns:
            A binary mask (cv2 image) of the detected color.
        """
        color_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        if points is not None:
            region_mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
            roi_corners = np.array([points], dtype=np.int32)
            cv2.fillPoly(region_mask, roi_corners, 255)
            final_mask = cv2.bitwise_and(color_mask, region_mask)
        else:
            final_mask = color_mask
            
        return final_mask

    def _reshape_roi_points(self, flat_points: list[int]) -> list[list[int]]:
        """Converts a flat list of coordinates [x1,y1,x2,y2,...] to a list of points [[x1,y1],[x2,y2],...]."""
        if len(flat_points) != 8:
            self.get_logger().error(f"Invalid ROI definition. Expected 8 values, got {len(flat_points)}. Using empty ROI.")
            return []
        return [
            [flat_points[0], flat_points[1]],
            [flat_points[2], flat_points[3]],
            [flat_points[4], flat_points[5]],
            [flat_points[6], flat_points[7]],
        ]

    def _save_annotated_image(self, base_name: str, turn_count: int, frame_bgr, masks, rois):
        """
        Saves annotated debug images for color detection.

        Args:
            base_name (str): The base filename (e.g., 'initial' or 'planning').
            turn_count (int): The current turn number.
            frame_bgr: The original BGR frame.
            masks (dict): Dictionary of color masks {'RED': mask, 'GREEN': mask}.
            rois (dict): Dictionary of ROIs used for detection, or None.
        """
        if not self.save_debug_images:
            return

        try:
            # 1. Annotated ROI Image
            annotated_frame = frame_bgr.copy()
            if rois:
                for roi_name, roi_points in rois.items():
                    cv2.polylines(annotated_frame, [np.array(roi_points, dtype=np.int32)], True, (255, 255, 0), 2)
            
            filename_annotated = os.path.join(self.debug_image_path, f"{base_name}_turn{turn_count}_annotated.jpg")
            cv2.imwrite(filename_annotated, annotated_frame)

            # 2. Combined Color Mask Image
            red_mask = masks.get('RED', np.zeros(frame_bgr.shape[:2], dtype=np.uint8))
            green_mask = masks.get('GREEN', np.zeros(frame_bgr.shape[:2], dtype=np.uint8))
            
            combined_mask_viz = np.zeros_like(frame_bgr)
            combined_mask_viz[np.where(green_mask == 255)] = (0, 255, 0) # BGR for Green
            combined_mask_viz[np.where(red_mask == 255)] = (0, 0, 255) # BGR for Red

            # --- Draw ROIs on the mask image as well ---
            if rois:
                for roi_name, roi_points in rois.items():
                    cv2.polylines(combined_mask_viz, [np.array(roi_points, dtype=np.int32)], True, (255, 255, 0), 2)


            filename_mask = os.path.join(self.debug_image_path, f"{base_name}_turn{turn_count}_masks.jpg")
            cv2.imwrite(filename_mask, combined_mask_viz)

            self.get_logger().info(f"Saved debug images for turn {turn_count}: {filename_annotated}, {filename_mask}")

        except Exception as e:
            self.get_logger().error(f"Failed to save debug image: {e}")

    def _mirror_roi_points(self, roi_points: list[list[int]]) -> list[list[int]]:
        """
        Mirrors the ROI horizontally based on the image width.
        Args:
            roi_points: A list of [x, y] coordinates for the ROI vertices.
        Returns:
            A new list of mirrored [x, y] coordinates.
        """
        mirrored_points = []
        for point in roi_points:
            mirrored_x = self.image_width - point[0]
            mirrored_points.append([mirrored_x, point[1]])
        
        # The order of vertices might be clockwise after mirroring, which is usually fine for fillPoly.
        # If winding order matters for other functions, it might need to be reversed.
        # For simple drawing and masking, this is sufficient.
        return mirrored_points

    def publish_twist_with_gain(self, linear_x, angular_z):
        """
        Calculates the final velocity command by applying the base gain and any state-specific gain modifiers,
        and then publishes the Twist message.
        """
        # Determine the gain modifier based on the current state
        state_specific_gain = self._get_state_specific_gain()

        # Calculate final velocities
        final_linear_x = linear_x * self.gain * state_specific_gain
        final_angular_z = angular_z * self.gain * state_specific_gain

        # Publish the message
        twist_msg = Twist()
        twist_msg.linear.x = final_linear_x
        twist_msg.angular.z = final_angular_z
        self.publisher_.publish(twist_msg)

    def _get_state_specific_gain(self) -> float:
        """
        Returns a gain modifier based on the current state of the robot.
        This allows for fine-tuning speeds in specific situations (e.g., slowing down during a reverse turn).
        
        Returns:
            float: A multiplier for the base gain. Defaults to 1.0.
        """
        # --- Default value ---
        state_specific_gain = 1.0

        # --- State-specific adjustments ---
        if self.state == State.STRAIGHT:
            if self.straight_sub_state == StraightSubState.ALIGN_WITH_INNER_WALL or self.straight_sub_state == StraightSubState.ALIGN_WITH_OUTER_WALL:
                state_specific_gain = self.gain_straight_align_wall
            elif self.straight_sub_state == StraightSubState.AVOID_INNER_PASS_THROUGH or self.straight_sub_state == StraightSubState.AVOID_OUTER_PASS_THROUGH:
                state_specific_gain = self.gain_straight_pass_through

        elif self.state == State.TURNING:
            if self.turning_sub_state == TurningSubState.APPROACH:
                state_specific_gain = self.gain_turning_approach
            elif self.turning_sub_state == TurningSubState.REVERSE:
                state_specific_gain = self.gain_turning_reverse

        # Add more conditions here if needed in the future
        # elif self.state == State.STRAIGHT and self.straight_sub_state == ... :
        #     state_specific_gain = self.gain_some_other_state
            
        return state_specific_gain

    def get_distance_at_world_angle(self, scan_data, world_angle_deg):
        """
        Gets the LiDAR distance at a specific world angle, corrected by the IMU yaw.
        If the primary data point is invalid (nan/inf), it checks its immediate neighbors
        as a fallback.
        """
        # --- 1. Calculate Target Index (same as before) ---
        robot_local_angle_deg = world_angle_deg - self.current_yaw_deg
        target_angle_rad = math.radians(robot_local_angle_deg)
        
        # Normalize angle to fit within the scan data's range [angle_min, angle_max)
        while target_angle_rad < scan_data.angle_min: target_angle_rad += 2.0 * math.pi
        while target_angle_rad >= scan_data.angle_min + 2.0 * math.pi: target_angle_rad -= 2.0 * math.pi

        try:
            if scan_data.angle_increment <= 0.0:
                self.get_logger().error('LiDAR angle_increment is non-positive.', throttle_duration_sec=5.0)
                return float('nan')
            index = int((target_angle_rad - scan_data.angle_min) / scan_data.angle_increment)
        except ZeroDivisionError:
            self.get_logger().error('angle_increment is zero.')
            return float('nan')

        num_ranges = len(scan_data.ranges)
        if not (0 <= index < num_ranges):
            return float('nan') # Index is out of bounds

        # --- 2. Create a list of indices to check, in order of priority ---
        # The order is: [primary_index, next_neighbor, previous_neighbor]
        indices_to_check = [index]
        
        # Add next neighbor, handling wraparound at the end of the array
        indices_to_check.append((index + 1) % num_ranges)
        
        # Add previous neighbor, handling wraparound at the start of the array
        indices_to_check.append((index - 1 + num_ranges) % num_ranges)

        # --- 3. Iterate through the indices and return the first valid distance ---
        for i in indices_to_check:
            # A valid distance must not be nan or inf
            distance = scan_data.ranges[i]
            if not math.isnan(distance) and not math.isinf(distance):
                # For debugging, log if a fallback was used
                if i != index:
                    self.get_logger().debug(f"Used fallback LiDAR data at index {i} (original was {index})", throttle_duration_sec=2.0)
                return distance
        
        # --- 4. If all checks fail, return nan ---
        return float('nan')

    def get_closest_distance_in_range(self, scan_data, center_world_angle_deg, range_deg):
        """
        Finds the closest valid LiDAR measurement within a specified world angle range.
        """
        if self.current_yaw_deg is None: return None
        min_dist, angle_at_min_dist_deg = float('inf'), None
        min_local_angle_deg = center_world_angle_deg - range_deg - self.current_yaw_deg
        max_local_angle_deg = center_world_angle_deg + range_deg - self.current_yaw_deg
        current_angle_rad = scan_data.angle_min
        for distance in scan_data.ranges:
            angle_diff_to_min = self._angle_diff(math.degrees(current_angle_rad), min_local_angle_deg)
            angle_diff_to_max = self._angle_diff(max_local_angle_deg, math.degrees(current_angle_rad))
            if angle_diff_to_min >= 0 and angle_diff_to_max >= 0:
                if not math.isnan(distance) and distance > 0.01 and distance < min_dist:
                    min_dist = distance
                    angle_at_min_dist_deg = self._angle_normalize(math.degrees(current_angle_rad) + self.current_yaw_deg)
            current_angle_rad += scan_data.angle_increment
        if angle_at_min_dist_deg is not None: return {'distance': min_dist, 'angle': angle_at_min_dist_deg}
        return None

    def _is_turn_maneuver_required(self) -> bool:
        """
        Determines if the entire turning maneuver (APPROACH and REVERSE) is necessary.
        This allows skipping the turn for simple, wide-radius corners.
        
        Returns:
            bool: False if the turning state can be skipped, True otherwise.
        """
        # Get the plan for the NEXT wall segment
        next_segment_index = (self.wall_segment_index + 1) % len(self.avoidance_path_plan)
        next_path_type = self._get_path_type_for_segment(next_segment_index, default_path='outer')

        is_next_path_outer = next_path_type in ['outer', 'outer_to_inner']
        is_next_path_inner = next_path_type in ['inner', 'inner_to_outer']

        try:
            is_entrance_clear = not self.entrance_obstacle_plan[next_segment_index]
        except IndexError:
            is_entrance_clear = False # Failsafe in case of index issue

        # --- Condition 1: Outer-to-Outer Turn ---
        if self.last_avoidance_path_was_outer and is_next_path_outer:
            self.get_logger().warn("SKIPPING TURN MANEUVER: Outer-to-Outer corner detected.")
            return False

        # --- NEW Condition 2: Safe Outer-to-Inner Turn ---
        if self.last_avoidance_path_was_outer and is_next_path_inner and is_entrance_clear and not self.wall_segment_index == 0:
            self.get_logger().warn("SKIPPING TURN MANEUVER: Safe Outer-to-Inner corner detected.")
            return False
        
        # --- Condition 3: Safe Inner-to-Outer Turn ---
        if not self.last_avoidance_path_was_outer and is_next_path_outer and is_entrance_clear:
            self.get_logger().warn("SKIPPING TURN MANEUVER: Safe Inner-to-Outer corner detected.")
            return False

        # If no conditions to skip are met, reversing is required by default.
        return True

    def _angle_normalize(self, angle_deg):
        """Normalize an angle to the range [-180, 180)."""
        while angle_deg >= 180.0: angle_deg -= 360.0
        while angle_deg < -180.0: angle_deg += 360.0
        return angle_deg

    def _angle_diff(self, target_deg, current_deg):
        """Calculates the shortest angle difference between two angles."""
        diff = target_deg - current_deg
        while diff <= -180.0: diff += 360.0
        while diff > 180.0: diff -= 360.0
        return diff

    def _get_mid_angle(self, angle1_deg, angle2_deg):
        """Calculates the midpoint of two angles, handling wraparound."""
        angle1_rad = math.radians(angle1_deg)
        angle2_rad = math.radians(angle2_deg)
        mid_angle_rad = math.atan2(
            math.sin(angle1_rad) + math.sin(angle2_rad),
            math.cos(angle1_rad) + math.cos(angle2_rad)
        )
        return self._angle_normalize(math.degrees(mid_angle_rad))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNavigatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()