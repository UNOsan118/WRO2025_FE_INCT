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
from std_srvs.srv import Trigger
import threading
import time
import shutil
import statistics
import collections

# (Enums are the same)
class State(Enum):
    PREPARATION = auto() 
    UNPARKING = auto()
    DETERMINE_COURSE = auto() 
    FINISHED = auto()
    STRAIGHT = auto()
    TURNING = auto()
    PARKING = auto()

class PreparationSubState(Enum):
    WAITING_FOR_CONTROLLER = auto()
    INITIALIZING_CAMERA = auto()
    DETERMINE_DIRECTION = auto()

class UnparkingSubState(Enum):
    PRE_UNPARKING_DETECTION = auto()
    INITIAL_TURN = auto()
    AVOIDANCE_REVERSE = auto()
    EXIT_STRAIGHT = auto()

class UnparkingStrategy(Enum):
    STANDARD_EXIT_TO_OUTER_LANE = auto()
    STANDARD_EXIT_TO_INNER_LANE = auto()
    AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CW = auto()
    AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CCW = auto()
    UNDEFINED = auto() # Fallback for unexpected cases

class DetermineCourseSubState(Enum):
    WAITING_FOR_CONTROLLER = auto()
    INITIALIZING_CAMERA = auto()
    DETECTING_OBSTACLE_COLOR = auto()
    PREPARE_TO_START = auto()
    DECIDE_INITIAL_PATH = auto()
    APPROACH_INITIAL_WALL = auto()
    DETECTING_STRAIGHT = auto()

class StraightSubState(Enum):
    ALIGN_WITH_OUTER_WALL = auto()
    ALIGN_WITH_INNER_WALL = auto()
    PLAN_NEXT_AVOIDANCE = auto()
    PRE_SCANNING_REVERSE = auto()
    AVOID_OUTER_TURN_IN = auto()
    AVOID_INNER_TURN_IN = auto()

class TurningSubState(Enum):
    POSITIONING_REVERSE = auto()
    APPROACH_CORNER = auto()
    EXECUTE_PIVOT_TURN = auto()
    FINALIZE_TURN = auto()

class ParkingSubState(Enum):
    PREPARE_PARKING = auto()
    REORIENT_FOR_PARKING = auto()     # Sub-state for U-turn if direction is CW
    LANE_CHANGE_FOR_PARKING = auto()
    APPROACH_PARKING_START = auto()   # Sub-state to move to the parking start position
    EXECUTE_PARKING_MANEUVER = auto()

    # --- Old states ---
    # PRE_PARKING_ADJUST = auto()
    # REVERSE_INTO_SPACE = auto()

class ReorientStep(Enum):
    INITIAL_FORWARD_APPROACH = auto()

    # Steps for Inner Lane S-Turn
    INNER_TURN_1 = auto()
    INNER_STRAIGHT1 = auto()
    INNER_STRAIGHT2 = auto()
    INNER_TURN_2 = auto()

    # Steps for Outer Lane 3-Point Turn
    OUTER_TURN_1 = auto()
    OUTER_REVERSE_TURN = auto()

    # --- Steps for CCW Inner-to-Outer Lane Change ---
    LC_INITIAL_FORWARD = auto() # Lane Change
    LC_REVERSE_TURN_1 = auto()
    LC_STRAIGHT_1 = auto()
    LC_STRAIGHT_2 = auto()
    LC_REVERSE_TURN_2 = auto()

    # Common completion state
    COMPLETED = auto()

class ApproachStep(Enum):
    ALIGN_AND_FORWARD = auto()
    REVERSE_TO_FINAL_POS = auto()

class ParkingManeuverStep(Enum):
    STEP1_REVERSE_TURN = auto()
    STEP2_REVERSE_STRAIGHT = auto()
    STEP3_ALIGN_TURN = auto()
    STEP4_FINAL_ADJUST = auto()

class ObstacleNavigatorNode(Node):
    # --- Initialization & Lifecycle ---
    def __init__(self):
        super().__init__('obstacle_navigator_node')

        # 1. CORE NODE SETUP
        # =================
        self.start_time = self.get_clock().now()
        # Only publish commands at a maximum rate of ~30Hz (33ms interval)
        # to avoid flooding the serial port of the controller.
        self.cmd_pub_interval_ms = 33

        self.declare_parameter('log_level_str', 'DEBUG') # Options: 'DEBUG', 'INFO', 'WARN', 'ERROR'
        # Load log_level immediately to affect subsequent logs
        log_level_str = self.get_parameter('log_level_str').get_parameter_value().string_value
        log_level_map = {
            'DEBUG': rclpy.logging.LoggingSeverity.DEBUG,
            'INFO': rclpy.logging.LoggingSeverity.INFO,
            'WARN': rclpy.logging.LoggingSeverity.WARN,
            'ERROR': rclpy.logging.LoggingSeverity.ERROR,
        }
        log_level = log_level_map.get(log_level_str.upper(), rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)
        self.get_logger().info(f"Logger level set to {log_level_str.upper()}")


        # 2. STATE MACHINE VARIABLES
        # ==========================
        # Main states and sub-states
        self.state = State.PREPARATION
        # self.state = State.PARKING
        self.preparation_sub_state = PreparationSubState.WAITING_FOR_CONTROLLER
        self.unparking_sub_state = UnparkingSubState.PRE_UNPARKING_DETECTION
        self.determine_course_sub_state = DetermineCourseSubState.WAITING_FOR_CONTROLLER # Legacy
        self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
        self.turning_sub_state = None 
        self.parking_sub_state = ParkingSubState.PREPARE_PARKING
        self.parking_maneuver_step = ParkingManeuverStep.STEP1_REVERSE_TURN


        # Strategy and planning variables
        self.unparking_strategy = UnparkingStrategy.UNDEFINED
        self.avoidance_path_plan = ['', '', '', '']
        self.entrance_obstacle_plan = [False, False, False, False]


        # 3. TUNING PARAMETERS (by category)
        # ==================================
        # --- General & Debug ---
        self.start_from_parking = True
        self.correct_mirrored_scan = True
        self.save_debug_images = True
        self.debug_image_path = '/home/ubuntu/WRO2025_FE_Japan/src/chassis_v2_maneuver/images'
        self.max_valid_range_m = 3.0
        self.max_turns = 12 #12
        
        # --- Driving & Speed Control ---
        self.forward_speed = 0.2
        self.max_steer = 1.0
        self.declare_parameter('gain', 2.0)
        self.gain_straight_align_outer_wall = 1.0
        self.gain_straight_align_inner_wall = 1.0
        self.gain_straight_outer_turn_in = 1.0
        self.direction = 'cw' # Note: This will be overwritten by PREPARATION state
        # --- Rate Limiter (Smoother) Parameters & Variables ---
        self.max_linear_acceleration = 3000.0 # m/s^2 past:30
        self.max_angular_acceleration_rad = 7000.0 # rad/s^2 past:70

        # --- Unparking Sequence ---
        self.unparking_speed = 0.05
        self.unparking_initial_turn_deg = 55.0
        self.unparking_exit_straight_dist_m = 0.33

        # --- Camera & Vision ---
        self.pan_servo_id = 1
        self.tilt_servo_id = 2
        self.initial_pan_position = 1850
        self.camera_move_duration = 0.2
        self.enable_dynamic_tilt = True
        self.tilt_center_position = 1500
        self.tilt_max_position = 2500
        self.tilt_min_position = 500
        self.tilt_position_cw = 530
        self.tilt_position_ccw = 2500
        self.tilt_update_interval_ms = 100 # Update tilt every 100ms (10Hz)
        self.image_width = 640
        # Color Thresholds
        self.red_lower1 = [0, 100, 80]
        self.red_upper1 = [3, 255, 243]
        self.red_lower2 = [174, 100, 80]
        self.red_upper2 = [179, 255, 243]
        self.green_lower = [55, 100, 67]
        self.green_upper = [80, 255, 240]
        """
        self.roi_planning_ccw_inner_flat = [225, 30, 0, 480, 640, 480, 480, 30]
        self.roi_planning_ccw_inner_start_area_flat = [225, 30, 0, 480, 560, 480, 320, 30]
        self.roi_planning_ccw_outer_flat = [220, 15, 100, 260, 525, 260, 440, 15]
        self.roi_planning_ccw_outer_start_area_flat = [220, 15, 100, 260, 425, 260, 300, 15]
        self.roi_planning_cw_inner_flat = [160, 30, 0, 480, 640, 480, 400, 30]
        self.roi_planning_cw_inner_start_area_flat = [320, 30, 80, 480, 640, 480, 400, 30]
        self.roi_planning_cw_outer_flat = [160, 15, 120, 260, 545, 260, 380, 15]
        self.roi_planning_cw_outer_start_area_flat = [280, 15, 150, 260, 545, 260, 380, 15]
        """
        
        # --- Alignment (PID) ---
        self.align_kp_angle = 0.02 # 0.04
        self.align_kp_dist = 3.75 # 7.5
        self.align_target_outer_dist_m = 0.2
        self.align_target_outer_dist_start_area_m = 0.39
        self.align_target_inner_dist_m = 0.2
        self.align_dist_tolerance_m = 0.005

        # --- Turning ---
        self.pre_scanning_reverse_target_dist_m = 0.7
        self.turn_forward_speed = 0.15
        self.turn_speed = 0.15
        
        # Format: self.turn_[current_lane]_to_[next_lane]_[value]
        # For Outer -> Outer 
        self.turn_outer_to_outer_dist_m = 0.55
        self.turn_outer_to_outer_angle_deg = 40.0
        self.turn_outer_to_outer_approach_speed = 0.18
        self.turn_outer_to_outer_turn_speed = 0.18

        # For Outer -> Outer (Clear) 
        self.turn_outer_to_outer_clear_dist_m = 0.55
        self.turn_outer_to_outer_clear_angle_deg = 40.0
        self.turn_outer_to_outer_clear_approach_speed = 0.20
        self.turn_outer_to_outer_clear_turn_speed = 0.20

        # For Outer -> Inner 
        self.turn_outer_to_inner_dist_m = 0.85
        self.turn_outer_to_inner_angle_deg = 90.0
        self.turn_outer_to_inner_approach_speed = 0.18
        self.turn_outer_to_inner_turn_speed = 0.15

        # For Outer -> Inner (Clear) 
        self.turn_outer_to_inner_clear_dist_m = 0.8
        self.turn_outer_to_inner_clear_angle_deg = 90.0 
        self.turn_outer_to_inner_clear_approach_speed = 0.18
        self.turn_outer_to_inner_clear_turn_speed = 0.15

        # For Inner -> Outer 
        self.turn_inner_to_outer_dist_m = 0.3
        self.turn_inner_to_outer_angle_deg = 90.0
        self.turn_inner_to_outer_approach_speed = 0.15
        self.turn_inner_to_outer_turn_speed = 0.15

        # For Inner -> Outer (Clear) : Like Outer to Outer
        self.turn_inner_to_outer_clear_dist_m =  0.5
        self.turn_inner_to_outer_clear_angle_deg = 40.0
        self.turn_inner_to_outer_clear_approach_speed = 0.18
        self.turn_inner_to_outer_clear_turn_speed = 0.18

        # For Inner -> Inner 
        self.turn_inner_to_inner_dist_m = 0.85
        self.turn_inner_to_inner_angle_deg = 90.0
        self.turn_inner_to_inner_approach_speed = 0.1
        self.turn_inner_to_inner_turn_speed = 0.1

        # For Inner -> Inner (Clear)
        self.turn_inner_to_inner_clear_dist_m = 0.80
        self.turn_inner_to_inner_clear_angle_deg = 90.0
        self.turn_inner_to_inner_clear_approach_speed = 0.17
        self.turn_inner_to_inner_clear_turn_speed = 0.17

        # Additional offset for the first corner (approaching start/finish line area)
        self.turn_start_area_dist_offset_m = 0.2
        
        # This parameter is now dynamically determined, but we keep it for fallback/other uses
        self.turn_completion_yaw_threshold_deg = 75.0 
        self.inner_wall_disappear_threshold = 1.3
        self.inner_wall_disappear_count = 3

        # --- Avoidance ---
        self.avoid_speed = 0.2
        self.avoid_kp_angle = 0.03 # P-gain for yaw control
        self.avoid_kp_dist = 1.5  # P-gain for distance control
        self.avoid_turn_in_kp_angle = 0.04
        self.avoid_outer_approach_angle_deg = 45.0
        self.avoid_outer_approach_target_dist_m = 0.23
        self.avoid_outer_approach_target_dist_start_area_m = 0.45
        self.avoid_inner_approach_angle_deg = 40.0
        self.avoid_inner_approach_target_dist_m = 0.25
        self.avoid_inner_pass_thresh_m = 0.6

        # --- Parking ---
        # --- Prepare (U-Turn/Lane Change) ---
        self.parking_approach_ccw_dist_min_m = 0.6
        self.parking_approach_ccw_dist_max_m = 0.7
        self.parking_approach_cw_overshoot_dist_m = 1.0
        self.parking_approach_cw_final_dist_min_m = 1.15
        self.parking_approach_cw_final_dist_max_m = 1.25
        self.parking_reverse_in_target_angle_deg = 50.0
        self.parking_reverse_in_yaw_tolerance_deg = 2.0
        # --- Reorientation (U-Turn) Maneuver ---
        self.reorient_initial_approach_dist_outer_m = 1.4
        self.reorient_initial_approach_dist_inner_m = 0.94
        self.reorient_yaw_tolerance_deg = 5.0
        self.reorient_turn_kp = 0.02
        self.reorient_forward_speed = 0.15
        self.reorient_reverse_speed = -0.15
        self.reorient_inner_s_turn_straight_dist_m = 0.1
        # --- Reorientation (Lane Change) Maneuver ---
        self.reorient_s_turn_fwd_dist_m = 0.1 # Distance for the first straight part of S-turn/LaneChange
        self.reorient_s_turn_rev_dist_m = 0.11 # Distance for the second straight (reverse) part
        self.lane_change_initial_approach_dist_m = 1.52

        # --- Approach ---
        self.parking_approach_target_outer_dist_m = 0.32
        self.parking_approach_slowdown_dist_m = 1.3  # Distance to start slowing down
        self.parking_approach_final_stop_dist_m = 0.78  # Final target distance
        self.parking_approach_slow_speed = 0.05     # Slower speed for final approach

        # --- Final Parking ---
        self.parking_step1_reverse_speed = -0.05
        self.parking_step1_target_angle_deg = 60.0
        self.parking_step2_reverse_speed = -0.05
        self.parking_step2_front_dist_trigger_m = 0.97
        self.parking_step3_reverse_speed = -0.05
        self.parking_step4_forward_speed = 0.05

        # --- Legacy Determine Course ---
        self.course_detection_threshold_m = 1.5
        self.course_detection_slow_speed = 0.1
        self.course_detection_speed = 0.17
        self.roi_left = [150, 50, 90, 430]
        self.roi_right = [290, 50, 90, 430]
        self.detection_pixel_threshold = 500
        self.detection_samples = 5 # Number of frames to sample for majority vote
        self.initial_approach_target_dist_inner_ccw_m = 0.3
        self.initial_approach_target_dist_inner_cw_m = 0.25
        self.initial_approach_target_dist_outer_m = 0.6
        self.initial_approach_target_dist_inner_m = 0.275

        # 4. INTERNAL STATE VARIABLES
        # ===========================
        # These are modified during runtime
        self.current_yaw_deg = 0.0
        self.latest_scan_msg = None
        self.latest_frame = None

        # Rate Limiter state
        self.last_published_linear = 0.0
        self.last_published_angular = 0.0
        self.last_cmd_pub_time = self.get_clock().now()

        # Dynamic Tilt state
        self.servo_units_per_degree = 1000.0 / 90.0
        self.last_tilt_update_time = self.get_clock().now()
        self.last_sent_tilt_position = -1 # Initialize with an invalid value

        # General state flags and counters
        self.turn_count = 0
        self.wall_segment_index = 0
        self.is_passing_obstacle = False
        self.can_start_new_turn = True
        self.final_approach_lane_is_outer = None
        self.camera_init_sent = False
        self.inner_wall_far_counter = 0
        self.stable_alignment_counter = 0
        self.lane_change_stability_counter = 0
        self.last_avoidance_path_was_outer = True
        self.initial_position_is_near = True
        self.initial_path_is_left = True
        self.initial_path_is_straight = False
        self.start_area_avoidance_required = False
        self.last_valid_steer = 0.0
        self.is_in_avoidance_alignment = False
        self.lane_change_stability_counter = 0

        # State-specific variables
        self.unparking_base_yaw_deg = 0.0
        self.pre_detection_step = 0 # 0=start, 1=waiting
        self.pre_detection_timer = None
        self.direction_detection_patience_counter = 0
        self.has_obstacle_at_parking_exit = False
        self.image_acquisition_retries = 0
        self.approach_base_yaw_deg = 0.0
        self.execute_turn_phase = 0 # 0: Forward, 1: Turning
        self.pre_parking_step = 0
        self.detection_results = []

        # Parking state variables
        self.reorient_step = None
        self.reorient_base_yaw_deg = 0.0
        self.approach_step = None
        self.parking_maneuver_step = None
        self.parking_base_yaw_deg = 0.0

        # Planning state variables
        self.planning_initiated = False
        self.planning_detection_threshold = 570
        self.planning_detection_threshold_outer_path_multiplier = 0.7
        self.post_planning_reverse_target_dist_m = 0.7 
        self.planning_scan_roi_flat = [200, 0, 120, 480, 440, 480, 360, 0] # [x, y, width, height]
        self.planning_scan_interval = 2
        self.planning_scan_stop_dist = 0.25
        self.planning_scan_stop_dist_start_area = 0.38
        self.planning_scan_start_dist_m = 0.5
        self.lidar_entrance_scan_start_dist_m = 0.7
        self.planning_camera_wait_timer = None
        self.is_sampling_for_planning = False 
        self.is_sampling_for_planning = False 
        self.max_red_blob_area = 0.0
        self.max_green_blob_area = 0.0
        self.max_red_blob_sample_num = 0
        self.max_green_blob_sample_num = 0

        # 5. CORE COMPONENTS
        # ==================
        self.state_lock = threading.RLock()
        self.bridge = CvBridge()


        # 6. SYSTEM INITIALIZATION CALLS
        # ==============================
        self._load_parameters() # Load all declared parameters into self.xxx variables
        self._initialize_variables()

        # Set initial state AFTER loading parameters
        if self.start_from_parking:
            self.get_logger().info("###########################################")
            self.get_logger().info("##  START MODE: From Parking Area        ##")
            self.get_logger().info("###########################################")
            self.state = State.PREPARATION
            self.preparation_sub_state = PreparationSubState.WAITING_FOR_CONTROLLER
        else:
            self.get_logger().info("###########################################")
            self.get_logger().info("##  START MODE: From Center (Legacy)     ##")
            self.get_logger().info("###########################################")
            self.state = State.DETERMINE_COURSE
            # For the legacy start, we need to wait for the controller inside DETERMINE_COURSE
            self.determine_course_sub_state = DetermineCourseSubState.WAITING_FOR_CONTROLLER
        
        self._initialize_debug_directory()
        self._setup_ros_communications()
        self.controller_ready_client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        self.yaw_publisher_ready_client = self.create_client(Trigger, '/yaw_publisher_node/init_finish')
        

        # 7. MAIN LOOP TIMER
        # ==================

        # --- FOR DEBUGGING: Force start from PARKING state ---
        force_start_from_parking = False # True -> Parking mode
        if force_start_from_parking:
            self.get_logger().warn("############################################################")
            self.get_logger().warn("##  DEBUG MODE: Forcing start from PARKING in 5 seconds...  ##")
            self.get_logger().warn("############################################################")

            # Temporarily set the state to FINISHED to prevent any movement
            self.state = State.FINISHED
            
            # Create a one-shot timer to start the parking debug mode after a delay
            self.debug_start_timer = self.create_timer(
                5.0, # 5-second delay
                self.start_parking_debug_mode 
            )
            
            # Disable the normal start_from_parking logic to avoid conflicts
            self.start_from_parking = False
        # --- END OF DEBUGGING BLOCK ---

        control_loop_rate = 50.0 # Hz
        self.control_loop_timer = self.create_timer(
            1.0 / control_loop_rate,
            self.control_loop_callback
        )

        self.get_logger().info(f'Course Detector Node started. Initial state: {self.state.name}')

    def start_parking_debug_mode(self):
        """
        Callback for a one-shot timer to activate the parking debug mode.
        This is called after a delay to allow other nodes (like IMU) to initialize.
        """
        with self.state_lock:
            self.get_logger().warn("--- STARTING PARKING DEBUG MODE NOW ---")
            
            # Destroy the timer so it doesn't run again
            if self.debug_start_timer:
                self.debug_start_timer.destroy()
                self.debug_start_timer = None

            # --- Manually set the state as if the robot just finished a CCW lap ---
            self.state = State.PARKING
            self.parking_sub_state = ParkingSubState.APPROACH_PARKING_START
            
            # --- Simulate the state after a CCW lap ---
            self.direction = "ccw"
            self.wall_segment_index = 0
            self.last_avoidance_path_was_outer = True # Assume we are on the outer lane
            
            # Initialize parking-specific state variables
            self.approach_step = None
            
            # --- Set yaw to a known value to simulate final orientation ---
            # Using the latest value from the now-stable IMU is better than forcing 0.
            self.get_logger().info(f"Using current yaw {self.current_yaw_deg:.2f} as starting orientation.")

    def _load_parameters(self):
        self.gain = self.get_parameter('gain').get_parameter_value().double_value

    def _initialize_variables(self):
        """
        Performs post-processing on variables after they are defined.
        e.g., converting lists to numpy arrays for OpenCV.
        """
        # Convert color threshold lists to numpy arrays
        self.red_lower1 = np.array(self.red_lower1, dtype=np.uint8)
        self.red_upper1 = np.array(self.red_upper1, dtype=np.uint8)
        self.red_lower2 = np.array(self.red_lower2, dtype=np.uint8)
        self.red_upper2 = np.array(self.red_upper2, dtype=np.uint8)
        self.green_lower = np.array(self.green_lower, dtype=np.uint8)
        self.green_upper = np.array(self.green_upper, dtype=np.uint8)

        self.get_logger().info("Internal variables initialized (e.g., numpy conversions).")

    def _initialize_debug_directory(self):
        """
        Initializes the debug image directory by deleting it if it exists
        and then recreating it. This ensures a clean state for each run.
        """
        if not self.save_debug_images:
            return

        try:
            self.get_logger().info(f"Initializing debug directory: {self.debug_image_path}")
            # Check if the directory exists
            if os.path.exists(self.debug_image_path):
                # Recursively delete the entire directory tree
                shutil.rmtree(self.debug_image_path)
                self.get_logger().info("... existing directory removed.")
            
            # Recreate the directory
            os.makedirs(self.debug_image_path, exist_ok=True)
            self.get_logger().info("... directory created successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize debug directory at {self.debug_image_path}: {e}")

    def _setup_ros_communications(self):
        # --- QoS profile for reliable command velocity publishing ---
        qos_profile_cmd_vel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(
            Twist, '/controller/cmd_vel', qos_profile=qos_profile_cmd_vel
        )

        # --- NEW: Define a QoS profile for reliable servo commands ---
        qos_profile_servo = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            # Use TRANSIENT_LOCAL to ensure the last command is delivered to late-joining subscribers
            durability=DurabilityPolicy.TRANSIENT_LOCAL 
        )
        self.servo_pub = self.create_publisher(
            SetPWMServoState, 
            '/ros_robot_controller/pwm_servo/set_state',
            qos_profile=qos_profile_servo # Apply the new QoS profile
        )

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

    def shutdown_callback(self):
        with self.state_lock:
            self.publish_twist_with_gain(0.0, 0.0)


    # --- Main Loop & Sensor Callbacks ---
    def control_loop_callback(self):
        """
        Main logic loop, running at a high frequency (e.g., 50Hz).
        It uses the latest available sensor data to make decisions.
        """
        # Acquire the lock at the beginning of the control cycle
        with self.state_lock:
            # Wait until the first valid scan message has been received.
            if self.latest_scan_msg is None:
                return

            # Use the stored message for all logic in this loop iteration.
            # This prevents data from changing mid-calculation.
            msg = self.latest_scan_msg

            # The logic from the old scan_callback is moved here.
            if self.state == State.PREPARATION: 
                self._handle_state_preparation(msg) 
            elif self.state == State.UNPARKING:
                self._handle_state_unparking(msg) 
            elif self.state == State.DETERMINE_COURSE:
                self._handle_state_determine_course(msg)
            elif self.state == State.FINISHED:
                self._handle_state_finished()
            elif self.state == State.TURNING:
                self._handle_state_turning(msg)
            elif self.state == State.STRAIGHT:
                self._handle_state_straight(msg)
            elif self.state == State.PARKING:
                self._handle_state_parking(msg)

    def scan_callback(self, msg):
        """Callback for LaserScan data. Only saves the latest message."""
        with self.state_lock:
            if not msg.ranges:
                return

            # msg.ranges is often a tuple, so we convert to a list for modification
            ranges = list(msg.ranges)
            for i in range(len(ranges)):
                if ranges[i] > self.max_valid_range_m:
                    ranges[i] = math.inf
            # Assign the modified list back to the message
            msg.ranges = ranges

            if self.correct_mirrored_scan:
                self.latest_scan_msg = self._correct_mirrored_scan(msg)
            else:
                self.latest_scan_msg = msg

    def yaw_callback(self, msg):
        with self.state_lock:
            self.current_yaw_deg = msg.data

    def image_callback(self, msg):
        """Callback to receive and store the latest camera frame."""
        with self.state_lock:
            try:
                self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge Error: {e}')


    # --- Main State Handlers ---
    def _handle_state_preparation(self, msg: LaserScan):
        """Dispatches to the correct handler based on the preparation_sub_state."""
        if self.preparation_sub_state == PreparationSubState.WAITING_FOR_CONTROLLER:
            self._handle_preparation_sub_waiting_for_controller()
        elif self.preparation_sub_state == PreparationSubState.INITIALIZING_CAMERA:
            self._handle_preparation_sub_initializing_camera()
        elif self.preparation_sub_state == PreparationSubState.DETERMINE_DIRECTION:
            self._handle_preparation_sub_determine_direction(msg)

    def _handle_state_unparking(self, msg: LaserScan):
        """Dispatches to the correct handler based on the unparking_sub_state."""
        if self.unparking_sub_state == UnparkingSubState.PRE_UNPARKING_DETECTION: 
            self._handle_unparking_sub_pre_unparking_detection() 
        elif self.unparking_sub_state == UnparkingSubState.INITIAL_TURN:
            self._handle_unparking_sub_initial_turn()
        elif self.unparking_sub_state == UnparkingSubState.AVOIDANCE_REVERSE: 
            self._handle_unparking_sub_avoidance_reverse(msg) 
        elif self.unparking_sub_state == UnparkingSubState.EXIT_STRAIGHT:
            self._handle_unparking_sub_exit_straight(msg)

    def _handle_state_determine_course(self, msg: LaserScan):
        """Dispatches to the correct handler based on the determine_course_sub_state."""
        if self.determine_course_sub_state == DetermineCourseSubState.WAITING_FOR_CONTROLLER:
            self._handle_determine_sub_waiting_for_controller()
        elif self.determine_course_sub_state == DetermineCourseSubState.INITIALIZING_CAMERA:
            self._handle_determine_sub_initializing_camera()
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

    def _handle_state_straight(self, msg):
        """Dispatches to the correct handler based on the straight_sub_state."""
        if self.straight_sub_state == StraightSubState.ALIGN_WITH_OUTER_WALL:
            self._handle_straight_sub_align_with_outer_wall(msg)
        elif self.straight_sub_state == StraightSubState.ALIGN_WITH_INNER_WALL:
            self._handle_straight_sub_align_with_inner_wall(msg)
        elif self.straight_sub_state == StraightSubState.PLAN_NEXT_AVOIDANCE:
            self._handle_straight_sub_plan_next_avoidance(msg)
        elif self.straight_sub_state == StraightSubState.PRE_SCANNING_REVERSE:
            self._handle_straight_sub_pre_scanning_reverse(msg)
        elif self.straight_sub_state == StraightSubState.AVOID_OUTER_TURN_IN:
            self._handle_straight_sub_avoid_outer_turn_in(msg)
        elif self.straight_sub_state == StraightSubState.AVOID_INNER_TURN_IN:
            self._handle_straight_sub_avoid_inner_turn_in(msg)

    def _handle_state_turning(self, msg):
        """Handles the multi-step turning maneuver."""
        if self.turning_sub_state == TurningSubState.POSITIONING_REVERSE:
            self._handle_turning_sub_positioning_reverse(msg)
        elif self.turning_sub_state == TurningSubState.APPROACH_CORNER:
            self._handle_turning_sub_approach_corner(msg)
        elif self.turning_sub_state == TurningSubState.EXECUTE_PIVOT_TURN:
            self._handle_turning_sub_execute_pivot_turn(msg)
        elif self.turning_sub_state == TurningSubState.FINALIZE_TURN:
            self._handle_turning_sub_finalize_turn()

    def _handle_state_parking(self, msg: LaserScan):
        """Dispatches to the correct handler based on the parking_sub_state."""
        if self.parking_sub_state == ParkingSubState.PREPARE_PARKING:
            self._handle_parking_sub_prepare_parking()
        elif self.parking_sub_state == ParkingSubState.REORIENT_FOR_PARKING:
            self._handle_parking_sub_reorient_for_parking(msg)
        elif self.parking_sub_state == ParkingSubState.LANE_CHANGE_FOR_PARKING:
            self._handle_parking_sub_lane_change_for_parking(msg)
        elif self.parking_sub_state == ParkingSubState.APPROACH_PARKING_START:
            self._handle_parking_sub_approach_parking_start(msg)
        elif self.parking_sub_state == ParkingSubState.EXECUTE_PARKING_MANEUVER:
            self._handle_parking_sub_execute_parking_maneuver(msg)
        
        # --- Old handlers (deactivated for now) ---
        # if self.parking_sub_state == ParkingSubState.PRE_PARKING_ADJUST:
        #     self._handle_parking_sub_pre_parking_adjust(msg)
        # elif self.parking_sub_state == ParkingSubState.REVERSE_INTO_SPACE:
        #     self._handle_parking_sub_reverse_into_space(msg)

    def _handle_state_finished(self):
        """Action for the FINISHED state: stop the robot."""
        self.publish_twist_with_gain(0.0, 0.0)

        # Calculate and log elapsed time, ensuring it only runs once.
        if self.start_time is not None:
            end_time = self.get_clock().now()
            duration_total_seconds = (end_time - self.start_time).nanoseconds / 1e9

            # --- MODIFIED: Calculation for minutes and seconds with one decimal place ---
            minutes = int(duration_total_seconds // 60)
            # Use floating point for seconds and then format it
            seconds = duration_total_seconds % 60

            self.get_logger().info("=============================================")
            self.get_logger().info("               R U N   F I N I S H E D               ")
            # Use an f-string with formatting to show one decimal place for seconds
            self.get_logger().info(f"    Elapsed Time: {minutes} minutes, {seconds:.1f} seconds")
            self.get_logger().info("=============================================")

            # Set start_time to None to prevent this from running again
            self.start_time = None


    # --- Sub-State Handlers ---
    # --- Preparation Sub-States ---
    def _handle_preparation_sub_waiting_for_controller(self):
        """
        Sub-state: Waits for all critical services (Controller, Yaw Publisher) to become available.
        This is a non-blocking check within the main control loop.
        """
        controller_ready = self.controller_ready_client.service_is_ready()
        yaw_publisher_ready = self.yaw_publisher_ready_client.service_is_ready()

        if not controller_ready:
            self.get_logger().info('PREPARATION: Controller service not available, waiting...', throttle_duration_sec=1.0)
            return
            
        if not yaw_publisher_ready:
            self.get_logger().info('PREPARATION: Yaw Publisher service not available, waiting...', throttle_duration_sec=1.0)
            return

        # All services are ready, proceed to the next state.
        self.get_logger().info("PREPARATION: All dependency services are ready. Transitioning to INITIALIZING_CAMERA.")
        self.preparation_sub_state = PreparationSubState.INITIALIZING_CAMERA

    def _handle_preparation_sub_initializing_camera(self):
        """
        Sub-state: Sends the command to move the camera to its initial angle
        and waits for the movement to complete before proceeding to the UNPARKING state.
        This function is a modified copy of _handle_determine_sub_initializing_camera.
        """
        # Publish the command periodically until the timer callback fires.
        msg = SetPWMServoState()
        msg.duration = self.camera_move_duration
        
        pan_state = PWMServoState()
        pan_state.id = [self.pan_servo_id]
        pan_state.position = [self.initial_pan_position]
        msg.state = [pan_state]

        self.servo_pub.publish(msg)
        self.get_logger().debug("PREPARATION: Sending initial camera angle command...", throttle_duration_sec=0.2)

        if not self.camera_init_sent:
            self.get_logger().info("PREPARATION: Starting timer for initial camera movement.")
            self.camera_init_sent = True
            
            wait_time = self.camera_move_duration + 1.5
            
            # This timer will call a new callback to transition out of the PREPARATION state.
            self.camera_wait_timer = self.create_timer(
                wait_time, 
                self._preparation_complete_callback # Use a new callback for clarity
            )
            self.get_logger().info(f"PREPARATION: Waiting {wait_time:.2f} seconds for camera to move...")

        # Keep the robot stationary during this state.
        self.publish_twist_with_gain(0.0, 0.0)

    def _handle_preparation_sub_determine_direction(self, msg: LaserScan):
        """
        Sub-state: Determines the course direction (CW/CCW) based on LiDAR readings
        from the specific parking start position.
        """
        # Get distances at +90 (left) and -90 (right) degrees relative to the robot's front
        left_dist = self.get_distance_at_world_angle(msg, self.current_yaw_deg + 90.0)
        right_dist = self.get_distance_at_world_angle(msg, self.current_yaw_deg - 90.0)

        # --- Define conditions for CW and CCW ---
        # Condition for Clockwise (CW)
        is_left_very_close = not math.isnan(left_dist) and left_dist < 0.30
        is_right_mid_range = not math.isnan(right_dist) and 0.30 <= right_dist < 1.0

        # Condition for Counter-Clockwise (CCW)
        is_right_very_close = not math.isnan(right_dist) and right_dist < 0.30
        is_left_mid_range = not math.isnan(left_dist) and 0.30 <= left_dist < 1.0

        direction_determined = False
        if is_left_very_close or is_right_mid_range:
            self.direction = "cw"
            self.get_logger().warn(
                f"Direction determined: CW. (L:{left_dist:.2f}m, R:{right_dist:.2f}m)"
            )
            direction_determined = True
        elif is_right_very_close or is_left_mid_range:
            self.direction = "ccw"
            self.get_logger().warn(
                f"Direction determined: CCW. (L:{left_dist:.2f}m, R:{right_dist:.2f}m)"
            )
            direction_determined = True

        if direction_determined:
            # --- Direction is set, preparation is fully complete ---
            self.get_logger().info("PREPARATION: All preparation steps complete.")

            self.unparking_base_yaw_deg = self.current_yaw_deg
            self.get_logger().info(f"UNPARKING: Stored base yaw: {self.unparking_base_yaw_deg:.2f} deg")

            self.get_logger().info("--- Transitioning to UNPARKING state ---")
            self.state = State.UNPARKING
            self.publish_twist_with_gain(0.0, 0.0) # Stop the robot if it was moving
        else:
            # --- Insurance: If no valid data, move forward slowly ---
            patience_threshold = 100 # Approx. 2 seconds at 50Hz
            if self.direction_detection_patience_counter < patience_threshold:
                self.direction_detection_patience_counter += 1
                self.get_logger().debug(
                    f"Waiting for valid LiDAR for direction detection... (L:{left_dist}, R:{right_dist})",
                    throttle_duration_sec=1.0
                )
                self.publish_twist_with_gain(0.0, 0.0)
            else:
                self.get_logger().warn(
                    "Could not determine direction from stationary position. Moving forward slowly.",
                    throttle_duration_sec=1.0
                )
                # A very slow speed just to get a clear reading
                self.publish_twist_with_gain(0.05, 0.0)

    # --- Unparking Sub-States ---
    def _handle_unparking_sub_pre_unparking_detection(self):
        """
        Sub-state: Aims camera 45 degrees and starts a timer to wait for movement.
        """
        # --- Step 0: Aim the camera and start the wait timer ---
        if self.pre_detection_step == 0:
            self.get_logger().info("PRE-UNPARKING DETECT (Step 0): Aiming camera 45 deg...")

            # Determine target camera angle
            angle_offset_deg = 45.0
            if self.direction == 'ccw':
                target_world_angle_deg = self._angle_normalize(self.unparking_base_yaw_deg + angle_offset_deg)
            else: # cw
                target_world_angle_deg = self._angle_normalize(self.unparking_base_yaw_deg - angle_offset_deg)

            # Calculate servo position and command the move
            camera_relative_angle_deg = self._angle_diff(target_world_angle_deg, self.current_yaw_deg)
            tilt_offset = camera_relative_angle_deg * self.servo_units_per_degree
            target_tilt_pos = self.tilt_center_position + tilt_offset
            clamped_tilt = int(max(self.tilt_min_position, min(self.tilt_max_position, target_tilt_pos)))
            
            self._set_camera_angle(
                pan_position=self.initial_pan_position,
                tilt_position=clamped_tilt,
                duration_sec=self.camera_move_duration + 0.5
            )

            # --- Start a timer that waits for the camera move to complete ---
            # Increase the buffer slightly to be safer
            stabilization_buffer_sec = 1.5
            wait_time_sec = self.camera_move_duration + 0.5 + stabilization_buffer_sec
            self.get_logger().info(f"PRE-UNPARKING DETECT: Waiting {wait_time_sec:.2f} seconds for camera movement.")

            self.pre_detection_timer = self.create_timer(
                wait_time_sec,
                self._process_pre_unparking_image_callback 
            )
            
            # Advance to the "waiting" step to prevent this block from running again
            self.pre_detection_step = 1

        # While step is 1 (waiting), do nothing but keep the robot still.
        self.publish_twist_with_gain(0.0, 0.0)

    def _handle_unparking_sub_initial_turn(self):
        """
        Sub-state: Executes a parameterized turn. After completion, transitions
        to the next state based on the pre-determined unparking strategy.
        """
        # --- 1. Determine target yaw and steer direction (No changes here) ---
        if self.direction == 'ccw':
            target_yaw_deg = self._angle_normalize(self.unparking_base_yaw_deg + self.unparking_initial_turn_deg)
            steer = self.max_steer
        else: # cw
            target_yaw_deg = self._angle_normalize(self.unparking_base_yaw_deg - self.unparking_initial_turn_deg)
            steer = -self.max_steer

        # --- 2. Check for completion ---
        yaw_error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        completion_threshold_deg = 10.0

        self.get_logger().debug(
            f"UNPARKING_TURN: TargetYaw:{target_yaw_deg:.1f}, CurrentYaw:{self.current_yaw_deg:.1f}, Err:{yaw_error_deg:.1f}",
            throttle_duration_sec=0.2
        )

        if abs(yaw_error_deg) < completion_threshold_deg:
            self.get_logger().info(f"UNPARKING_TURN: Turn complete. Current strategy is '{self.unparking_strategy.name}'.")
            self.publish_twist_with_gain(0.0, 0.0)

            # --- 3. ADDED: Transition based on the decided strategy ---
            if self.unparking_strategy == UnparkingStrategy.STANDARD_EXIT_TO_OUTER_LANE:
                self.get_logger().info("Strategy requires outer lane. Transitioning to STRAIGHT state.")
                self.state = State.STRAIGHT
                self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
            
            elif self.unparking_strategy == UnparkingStrategy.STANDARD_EXIT_TO_INNER_LANE:
                self.get_logger().info("Strategy requires inner lane. Transitioning to STRAIGHT state.")
                self.state = State.STRAIGHT
                self.straight_sub_state = StraightSubState.AVOID_INNER_TURN_IN

            elif self.unparking_strategy == UnparkingStrategy.AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CW:
                self.get_logger().info("Strategy requires obstacle avoidance. Transitioning to EXIT_STRAIGHT.")
                self.unparking_sub_state = UnparkingSubState.EXIT_STRAIGHT
            
            elif self.unparking_strategy == UnparkingStrategy.AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CCW:
                self.get_logger().warn("Strategy for CCW obstacle avoidance. Transitioning to AVOIDANCE_REVERSE.")
                self.unparking_sub_state = UnparkingSubState.AVOIDANCE_REVERSE
            
            else: # UNDEFINED or any other case
                self.get_logger().error("Undefined unparking strategy! Strategy requires outer lane. Transitioning to STRAIGHT state.")
                self.state = State.STRAIGHT
                self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
            
            return # IMPORTANT: Exit after handling the state transition

        # --- 4. Execute the turn (No changes here) ---
        turn_speed = self.unparking_speed
        self.publish_twist_with_gain(turn_speed, steer)

    def _handle_unparking_sub_avoidance_reverse(self, msg: LaserScan):
        """
        Sub-state: Reverses straight back to create space from a close obstacle
        at the parking exit.
        """
        # --- 1. Determine the front direction (relative to the parking spot) ---
        # The "front" in this context is the original starting orientation.
        front_angle_deg = self.unparking_base_yaw_deg
        front_dist = self.get_distance_at_world_angle(msg, front_angle_deg)

        # --- 2. Check for completion ---
        target_dist = 1.1
        if not math.isnan(front_dist) and front_dist > target_dist:
            self.get_logger().info(
                f"AVOIDANCE_REVERSE: Reverse complete. Front distance is now {front_dist:.2f}m."
            )
            self.get_logger().info("--- Transitioning to EXIT_STRAIGHT sub-state ---")
            self.unparking_sub_state = UnparkingSubState.EXIT_STRAIGHT
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # --- 3. Execute reverse movement ---
        # Reverse straight back. Steer should be 0 to maintain the current heading.
        self.get_logger().debug(
            f"AVOIDANCE_REVERSE: Reversing... FrontDist:{front_dist:.2f}m, Target: < {target_dist}m",
            throttle_duration_sec=0.2
        )
        
        # A slow and controlled reverse speed.
        reverse_speed = -self.unparking_speed 
        self.publish_twist_with_gain(reverse_speed, 0.0)

    def _handle_unparking_sub_exit_straight(self, msg: LaserScan):
        """
        Sub-state: Moves straight forward until the side wall is at a specific distance.
        """
        # --- 1. Determine target yaw and wall measurement angle ---
        # The target yaw is the one we achieved in the previous INITIAL_TURN sub-state.
        # This MUST match the calculation in _handle_unparking_sub_initial_turn.
        if self.direction == 'ccw':
            target_yaw_deg = self._angle_normalize(self.unparking_base_yaw_deg + 90)
        else: # cw
            target_yaw_deg = self._angle_normalize(self.unparking_base_yaw_deg - 90)
        wall_angle_deg = self._angle_normalize(target_yaw_deg)

        # --- 2. Check for completion ---
        wall_dist = self.get_distance_at_world_angle(msg, wall_angle_deg)
        target_dist = self.unparking_exit_straight_dist_m

        if not math.isnan(wall_dist) and wall_dist <= target_dist:
            self.get_logger().info(
                f"UNPARKING_STRAIGHT: Target wall distance reached ({wall_dist:.2f}m <= {target_dist:.2f}m)."
            )
            self.state = State.STRAIGHT
            self.straight_sub_state = StraightSubState.ALIGN_WITH_INNER_WALL
            # Stop the robot to get a clear image for detection
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # --- 3. Execute PID-controlled straight movement (similar to _execute_pid_alignment) ---
        # We only use the angle part of the PID control to go straight.
        angle_error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg
        final_steer = max(min(angle_steer, self.max_steer), -self.max_steer)

        self.get_logger().debug(
            f"UNPARKING_STRAIGHT: TargetYaw:{target_yaw_deg:.1f}, CurrentYaw:{self.current_yaw_deg:.1f}, "
            f"WallDist:{wall_dist:.2f}m, Steer:{final_steer:.2f}",
            throttle_duration_sec=0.2
        )

        straight_speed = self.unparking_speed
        self.publish_twist_with_gain(straight_speed, final_steer)

    # --- Determine Course Sub-States (Legacy) ---
    def _handle_determine_sub_waiting_for_controller(self):
        """
        Sub-state: Waits for all critical services for legacy start mode.
        """
        controller_ready = self.controller_ready_client.service_is_ready()
        yaw_publisher_ready = self.yaw_publisher_ready_client.service_is_ready()

        if not controller_ready:
            self.get_logger().info('DETERMINE_COURSE: Controller service not available, waiting...', throttle_duration_sec=1.0)
            return

        if not yaw_publisher_ready:
            self.get_logger().info('DETERMINE_COURSE: Yaw Publisher service not available, waiting...', throttle_duration_sec=1.0)
            return

        self.get_logger().info("DETERMINE_COURSE: All dependency services are ready. Transitioning to INITIALIZING_CAMERA.")
        self.determine_course_sub_state = DetermineCourseSubState.INITIALIZING_CAMERA

    def _handle_determine_sub_initializing_camera(self):
        """
        Sub-state: Sends the command to move the camera to its initial angle
        and waits for the movement to complete before proceeding.
        """
        # Publish the command periodically until the timer callback fires.
        msg = SetPWMServoState()
        msg.duration = self.camera_move_duration
        
        pan_state = PWMServoState()
        pan_state.id = [self.pan_servo_id]
        pan_state.position = [self.initial_pan_position]
        msg.state = [pan_state]

        self.servo_pub.publish(msg)
        self.get_logger().debug("Sending initial camera angle command...", throttle_duration_sec=0.2)

        if not self.camera_init_sent:
            self.get_logger().info("INITIALIZING_CAMERA: Starting timer for initial camera movement.")
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
        
        # --- MODIFIED: Store detailed detection results ---
        is_red_left = left_areas['RED'] > self.detection_pixel_threshold
        is_red_right = right_areas['RED'] > self.detection_pixel_threshold
        is_green_left = left_areas['GREEN'] > self.detection_pixel_threshold
        is_green_right = right_areas['GREEN'] > self.detection_pixel_threshold

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

        # --- NEW: Save debug images for the first sample in this state ---
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
        - If wall is close (< 1.35m), transitions to DETECTING_STRAIGHT.
        - If wall is far (>= 1.35m) or obstacle is close (< 0.9m) transitions to DECIDE_INITIAL_PATH.
        """
        front_dist = self.get_distance_at_world_angle(msg, 0.0)

        if not math.isnan(front_dist):
            self.publish_twist_with_gain(0.0, 0.0)
            self.get_logger().debug(f"DETERMINE_COURSE (PREPARE): Front wall distance acquired: {front_dist:.2f}m")
            
            if front_dist > 0.9 and front_dist < 1.35:
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
        determines the driving direction (cw/ccw) early, and flags if special
        start area avoidance is required.
        """
        color_votes = [res['color'] for res in self.detection_results]
        red_votes = color_votes.count("RED")
        green_votes = color_votes.count("GREEN")
        none_votes = color_votes.count("NONE")

        self.get_logger().info(f"Color detection vote - Red: {red_votes}, Green: {green_votes}, None: {none_votes}")
        
        self.start_area_avoidance_required = False
        self.initial_path_is_straight = True # Default to straight if no obstacle

        if red_votes > green_votes and red_votes >= none_votes:
            self.get_logger().info("Red object has majority. Determining initial path.")
            self.initial_path_is_straight = False

            left_red_detected_count = sum(1 for res in self.detection_results if res['color'] == 'RED' and res['left'])
            right_red_detected_count = sum(1 for res in self.detection_results if res['color'] == 'RED' and res['right'])
            is_red_on_right = right_red_detected_count > left_red_detected_count

            if is_red_on_right:
                # Case 1: Red on RIGHT -> Drive CW, follow INNER wall.
                self.direction = "cw"
                # To follow INNER wall in CW, we need is_following_outer_wall=False.
                # In CW, is_following_outer_wall = initial_path_is_left.
                # So, set initial_path_is_left = False.
                self.initial_path_is_left = False
                self.start_area_avoidance_required = False
                self.initial_approach_target_dist_inner_m = self.initial_approach_target_dist_inner_cw_m
                self.get_logger().warn(f"Red on RIGHT -> CW, Inner Path (Normal). Approaching RIGHT wall.")
            else:
                # Case 2: Red on LEFT -> Drive CCW, follow OUTER wall.
                self.direction = "ccw"
                # To follow OUTER wall in CCW, we need is_following_outer_wall=True.
                # In CCW, is_following_outer_wall = not initial_path_is_left.
                # So, set initial_path_is_left = False.
                self.initial_path_is_left = False
                self.start_area_avoidance_required = True
                self.get_logger().warn(f"Red on LEFT -> CCW, Outer Path (Tricky). Approaching RIGHT wall.")
            
            self.determine_course_sub_state = DetermineCourseSubState.APPROACH_INITIAL_WALL

        elif green_votes > red_votes and green_votes >= none_votes:
            self.get_logger().info("Green object has majority. Determining initial path.")
            self.initial_path_is_straight = False

            left_green_detected_count = sum(1 for res in self.detection_results if res['color'] == 'GREEN' and res['left'])
            right_green_detected_count = sum(1 for res in self.detection_results if res['color'] == 'GREEN' and res['right'])
            is_green_on_left = left_green_detected_count > right_green_detected_count

            if is_green_on_left:
                # Case 4: Green on LEFT -> Drive CCW, follow INNER wall.
                self.direction = "ccw"
                # To follow INNER wall in CCW, we need is_following_outer_wall=False.
                # In CCW, is_following_outer_wall = not initial_path_is_left.
                # So, set initial_path_is_left = True.
                self.initial_path_is_left = True
                self.start_area_avoidance_required = False
                self.initial_approach_target_dist_inner_m = self.initial_approach_target_dist_inner_ccw_m
                self.get_logger().warn(f"Green on LEFT -> CCW, Inner Path (Normal). Approaching LEFT wall.")
            else:
                # Case 3: Green on RIGHT -> Drive CW, follow OUTER wall.
                self.direction = "cw"
                # To follow OUTER wall in CW, we need is_following_outer_wall=True.
                # In CW, is_following_outer_wall = initial_path_is_left.
                # So, set initial_path_is_left = True.
                self.initial_path_is_left = True
                self.start_area_avoidance_required = True
                self.get_logger().warn(f"Green on RIGHT -> CW, Outer Path (Tricky). Approaching LEFT wall.")

            self.determine_course_sub_state = DetermineCourseSubState.APPROACH_INITIAL_WALL
            
        else:
            self.get_logger().info("No clear majority or no obstacle detected. Proceeding STRAIGHT.")
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
            target_dist = self.initial_approach_target_dist_outer_m
        else:
            target_dist = self.initial_approach_target_dist_inner_m

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
            f"APPROACH_INITIAL_WALL | TargetDist: {target_dist:.2f}, "
            f"CurrentDist: {wall_dist:.2f}m | Steer: {initial_steer:.2f}",
            throttle_duration_sec=0.2
        )
        self.publish_twist_with_gain(self.course_detection_speed, initial_steer)
    
    def _handle_determine_sub_detecting_straight(self, msg: LaserScan):
        """
        Sub-state: Moves forward until the first corner is detected.
        Also checks for the initial course direction if not already found.
        """
        # The base angle for the first straight section is always 0.0
        base_angle_deg = 0.0

        # --- Primary Logic: Check for the first corner ---
        is_turning, _ = self._check_for_corner(msg, base_angle_deg)
        if is_turning:
            # The corner check logic already handles the transition to PRE_SCANNING_REVERSE
            # or the appropriate next state. So we just need to exit.
            return
        
        # --- Secondary Logic (runs only if a corner is not found yet) ---
        # This part is for the initial moments to confirm the course direction.
        # It's less likely to be triggered now but serves as a backup.
        course_found, _ = self._check_course_and_get_direction(msg)
        if course_found:
            self.get_logger().info(f"******* Course direction set to: {self.direction.upper()} (during straight run) *******")
            # Do not transition here. Let the corner check handle the transition.
            # The direction is now set, so subsequent PID alignment will work correctly.
        
        # --- Continue moving straight using PID alignment ---
        if (self.direction == 'cw' and self.initial_path_is_left) or \
           (self.direction == 'ccw' and not self.initial_path_is_left):
            is_following_outer_wall = True
            self.last_avoidance_path_was_outer = True
        else:
            is_following_outer_wall = False
            self.last_avoidance_path_was_outer = False
        
        self._execute_pid_alignment(
            msg, 
            base_angle_deg, 
            is_outer_wall=is_following_outer_wall,
            speed=self.course_detection_speed
        )

    # --- Straight Sub-States ---
    def _handle_straight_sub_align_with_outer_wall(self, msg: LaserScan):
        if self._check_for_finish_condition(msg):
            return
        self.last_avoidance_path_was_outer = True
        """A PID-based wall follower that now includes avoidance completion logic."""
        base_angle_deg = self._calculate_base_angle()

        # --- NEW: Logic to enable next turn based on stable alignment ---
        self._update_turn_permission_counter(msg, base_angle_deg)

        # --- Avoidance Completion Logic ---
        if self.is_in_avoidance_alignment:
            # Ask the helper function if we have passed the obstacle.
            if self._is_safe_for_lane_change(msg, base_angle_deg):
                path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='outer')
                
                # Only transition state if a lane change is required.
                if path_type == 'outer_to_inner' and self.turn_count != self.max_turns:
                    self.get_logger().info("--- (Align) Obstacle cleared. Initiating lane change [O->I]. ---")
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.AVOID_INNER_TURN_IN,
                        path_was_outer=True)
                    return # A state transition occurred, so exit this loop iteration.
                
                # For a simple 'outer' avoidance, just reset the flag and continue alignment.
                self.get_logger().info("--- (Align) Outer avoidance complete. Resuming normal alignment. ---")
                self.is_in_avoidance_alignment = False
                self.is_passing_obstacle = False # Ensure this is also reset

        # --- Standard Behavior ---
        is_turning, _ = self._check_for_corner(msg, base_angle_deg)
        if is_turning:
            self.is_in_avoidance_alignment = False # Reset flag on new turn
            return

        # Driving logic is common for all scenarios.
        disable_dist_control = False
        yaw_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
        yaw_deviation_threshold_deg = 45.0

        is_cw_deviation = self.direction == 'cw' and yaw_error_deg > yaw_deviation_threshold_deg
        is_ccw_deviation = self.direction == 'ccw' and yaw_error_deg < -yaw_deviation_threshold_deg

        if is_cw_deviation or is_ccw_deviation:
            disable_dist_control = True
            self.get_logger().warn(
                f"Large inward deviation detected (YawErr: {yaw_error_deg:.1f}deg). Disabling distance control.",
                throttle_duration_sec=1.0
            )

        # Driving logic is common for all scenarios.
        speed = self.forward_speed * 0.7 if self.turn_count == 0 else self.forward_speed
        self._execute_pid_alignment(
            msg, 
            base_angle_deg, 
            is_outer_wall=True, 
            speed=speed,
            disable_dist_control=disable_dist_control # Pass the dynamically set flag
        )

    def _handle_straight_sub_align_with_inner_wall(self, msg: LaserScan):
        if self._check_for_finish_condition(msg):
            return
        self.last_avoidance_path_was_outer = False
        """A PID-based wall follower that now includes avoidance completion logic."""
        base_angle_deg = self._calculate_base_angle()

        # --- NEW: Logic to enable next turn based on stable alignment ---
        self._update_turn_permission_counter(msg, base_angle_deg)

        # --- Avoidance Completion Logic ---
        if self.is_in_avoidance_alignment:
            # Ask the helper function if we have passed the obstacle.
            if self._is_safe_for_lane_change(msg, base_angle_deg):
                path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='inner')
                
                # Only transition state if a lane change is required.
                if path_type == 'inner_to_outer' and self.turn_count != self.max_turns:
                    self.get_logger().info("--- (Align) Obstacle cleared. Initiating lane change [I->O]. ---")
                    self._complete_avoidance_phase(
                        next_sub_state=StraightSubState.AVOID_OUTER_TURN_IN,
                        path_was_outer=False)
                    return # A state transition occurred, so exit this loop iteration.

                # For a simple 'inner' avoidance, just reset the flag and continue alignment.
                self.get_logger().info("--- (Align) Inner avoidance complete. Resuming normal alignment. ---")
                self.is_in_avoidance_alignment = False
                self.is_passing_obstacle = False # Ensure this is also reset

        # --- Standard Behavior ---
        is_turning, _ = self._check_for_corner(msg, base_angle_deg)
        if is_turning:
            self.is_in_avoidance_alignment = False # Reset flag on new turn
            return

        # Driving logic is common for all scenarios.
        speed = self.forward_speed * 0.7 if self.turn_count == 0 else self.forward_speed
        self._execute_pid_alignment(msg, base_angle_deg, is_outer_wall=False, speed=speed)

    def _handle_straight_sub_avoid_outer_turn_in(self, msg: LaserScan):
        """Phase 1: Approach the outer wall, while also checking if the obstacle is passed."""
        if self._check_for_finish_condition(msg):
            return
            
        base_angle_deg = self._calculate_base_angle()
    
        # --- NEW: Update turn permission counter during TURN_IN phase ---
        self._update_turn_permission_counter(msg, base_angle_deg)

        # Define target yaw and wall measurement angles
        if self.direction == 'ccw':
            target_yaw_deg = self._angle_normalize(base_angle_deg - self.avoid_outer_approach_angle_deg)
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
            inner_wall_angle_for_pass_check = self._angle_normalize(base_angle_deg + 90.0)
        else: # cw
            target_yaw_deg = self._angle_normalize(base_angle_deg + self.avoid_outer_approach_angle_deg)
            outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            inner_wall_angle_for_pass_check = self._angle_normalize(base_angle_deg - 90.0)

        outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle_for_pass_check)

        # Monitor for passing the obstacle during the turn-in phase
        path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='outer')

        if path_type != 'inner_to_outer':
            if self.is_passing_obstacle:
                # Check if we have completed the pass MANEUVER during the turn-in
                if not math.isnan(inner_wall_dist) and inner_wall_dist > self.avoid_inner_pass_thresh_m:
                    self.get_logger().warn("!!! Obstacle passed completely during TURN_IN phase.")
                    if path_type == 'outer_to_inner':
                        self._complete_avoidance_phase(
                            next_sub_state=StraightSubState.AVOID_INNER_TURN_IN,
                            path_was_outer=True)
                    else:
                        self._complete_avoidance_phase(
                            next_sub_state=StraightSubState.ALIGN_WITH_OUTER_WALL,
                            path_was_outer=True)
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
        
        # --- MODIFIED: Relaxed the isnan check to continue steering ---
        if math.isnan(outer_wall_dist):
            self.get_logger().warn("Lost sight of outer wall during approach. Continuing with P-control.", throttle_duration_sec=1.0)
            # Do not return; allow P-control to continue steering
        
        if self.wall_segment_index == 0:
            approach_target_dist = self.avoid_outer_approach_target_dist_start_area_m
        else:
            approach_target_dist = self.avoid_outer_approach_target_dist_m

        is_distance_met = not math.isnan(outer_wall_dist) and outer_wall_dist > 0.0 and outer_wall_dist <= approach_target_dist

        # Check if this maneuver is specifically an 'inner_to_outer' lane change
        path_type = self._get_path_type_for_segment(self.wall_segment_index)
        is_specific_lane_change = (path_type == 'inner_to_outer')

        if is_specific_lane_change:
            # For this specific lane change, we require BOTH distance and stability.
            is_lane_change_stable = False
            if not math.isnan(inner_wall_dist) and not math.isnan(outer_wall_dist) and \
               (inner_wall_dist + outer_wall_dist) < 1.1:
                self.lane_change_stability_counter += 1
            else:
                pass

            # --- THROTTLED LOGGING ---
            self.get_logger().debug(
                f"Lane change [I->O] stability counter: {self.lane_change_stability_counter}",
                throttle_duration_sec=0.1 # Log every 0.5 seconds
            )

            if self.lane_change_stability_counter > 25:
                is_lane_change_stable = True
            
            if is_distance_met and is_lane_change_stable:
                self.get_logger().info(f"Lane change [I->O] complete (Dist & Stable). Transitioning.")
                self.lane_change_stability_counter = 0
                self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
                self.is_in_avoidance_alignment = True
                self.is_passing_obstacle = False
                self.publish_twist_with_gain(0.0, 0.0)
                return
        else:
            # For simple avoidance (e.g., 'outer'), only the distance matters.
            if is_distance_met:
                self.get_logger().info(f"Approach complete (Simple Avoidance, Dist: {outer_wall_dist:.2f}m). Transitioning.")
                self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
                self.is_in_avoidance_alignment = True
                self.is_passing_obstacle = False
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

    def _handle_straight_sub_avoid_inner_turn_in(self, msg: LaserScan):
        """Phase 1: Approach the INNER wall at a fixed angle.
        Completion is determined by the distance to the OUTER wall.
        """
        if self._check_for_finish_condition(msg):
            return

        base_angle_deg = self._calculate_base_angle()

        # --- NEW: Update turn permission counter during TURN_IN phase ---
        self._update_turn_permission_counter(msg, base_angle_deg)

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

        if path_type != 'outer_to_inner':
            if self.is_passing_obstacle:
                # Check if we have completed the pass MANEUVER during the turn-in
                if(not math.isnan(inner_wall_dist) and not math.isnan(outer_wall_dist) and 
                    outer_wall_dist < self.avoid_inner_pass_thresh_m):
                    self.get_logger().warn("!!! Obstacle passed completely during INNER TURN_IN phase.")
                    if path_type == 'inner_to_outer':
                        self._complete_avoidance_phase(
                            next_sub_state=StraightSubState.AVOID_OUTER_TURN_IN,
                            path_was_outer=False)
                    else:
                        self._complete_avoidance_phase(
                            next_sub_state=StraightSubState.ALIGN_WITH_INNER_WALL,
                            path_was_outer=False)
                    return
            else:
                # Check if we have started passing the obstacle (based on outer wall getting close)
                is_oriented_correctly = self._check_yaw_alignment(base_angle_deg, 60.0)
                sum_side_walls_dist = inner_wall_dist + outer_wall_dist
                if(not math.isnan(outer_wall_dist) and not math.isnan(inner_wall_dist) 
                and outer_wall_dist < self.avoid_inner_pass_thresh_m and sum_side_walls_dist < 0.7 and 
                is_oriented_correctly and self.turn_count != 0):
                    self.get_logger().warn(">>> Passing obstacle now (during INNER TURN_IN)...")
                    self.is_passing_obstacle = True

        # Original approach logic
        if math.isnan(outer_wall_dist):
            self.get_logger().error("Lost sight of outer wall during inner approach. But Not Stopping.", throttle_duration_sec=1.0)
            # self.publish_twist_with_gain(0.0, 0.0)
            # Do not return here, let the P-control continue to steer towards the target yaw
        
        effective_dist = 1.0 - outer_wall_dist if not math.isnan(outer_wall_dist) else float('inf')
        is_distance_met = effective_dist >= 0 and effective_dist <= self.avoid_inner_approach_target_dist_m

        # Check if this maneuver is specifically an 'outer_to_inner' lane change
        path_type = self._get_path_type_for_segment(self.wall_segment_index)
        is_specific_lane_change = (path_type == 'outer_to_inner')

        if is_specific_lane_change:
            # For this specific lane change, we require BOTH distance and stability.
            is_lane_change_stable = False
            if not math.isnan(inner_wall_dist) and not math.isnan(outer_wall_dist) and \
               (inner_wall_dist + outer_wall_dist) < 1.1:
                self.lane_change_stability_counter += 1
            else:
                pass

            # --- THROTTLED LOGGING ---
            self.get_logger().debug(
                f"Lane change [O->I] stability counter: {self.lane_change_stability_counter}",
                throttle_duration_sec=0.1 # Log every 0.5 seconds
            )
            
            if self.lane_change_stability_counter > 25:
                is_lane_change_stable = True

            if is_distance_met and is_lane_change_stable:
                self.get_logger().info(f"Lane change [O->I] complete (Dist & Stable). Transitioning.")
                self.lane_change_stability_counter = 0
                self.straight_sub_state = StraightSubState.ALIGN_WITH_INNER_WALL
                self.is_in_avoidance_alignment = True
                self.is_passing_obstacle = False
                self.publish_twist_with_gain(0.0, 0.0)
                return
        else:
            # For simple avoidance (e.g., 'inner'), only the distance matters.
            if is_distance_met:
                self.get_logger().info(f"Approach complete (Simple Avoidance, EffectiveDist: {effective_dist:.2f}m). Transitioning.")
                self.straight_sub_state = StraightSubState.ALIGN_WITH_INNER_WALL
                self.is_in_avoidance_alignment = True
                self.is_passing_obstacle = False
                self.publish_twist_with_gain(0.0, 0.0)
                return


        # P-control to maintain the approach angle
        error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        angular_z = self.avoid_turn_in_kp_angle * error_deg
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)
        
        self.publish_twist_with_gain(self.avoid_speed, final_steer)

    def _handle_straight_sub_pre_scanning_reverse(self, msg: LaserScan):
        """
        Sub-state: Reverses straight back to create space before the move-and-scan phase.
        """
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)

        if math.isnan(front_dist):
            self.get_logger().warn("Cannot get front distance for reversing. Reversing slowly.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(-self.forward_speed * 0.8, 0.0)
            return

        # --- Reverted to check against a fixed absolute distance ---
        if front_dist >= self.post_planning_reverse_target_dist_m:
            self.get_logger().info(f"Reverse complete (Front Dist: {front_dist:.3f}m).")
            self.get_logger().info("Transitioning to move-and-scan planning phase.")
            self.state = State.STRAIGHT
            self.straight_sub_state = StraightSubState.PLAN_NEXT_AVOIDANCE
            self.publish_twist_with_gain(0.0, 0.0)
        else:
            self.get_logger().debug(
                f"Reversing... Target: > {self.post_planning_reverse_target_dist_m:.2f}m, "
                f"Current: {front_dist:.3f}m",
                throttle_duration_sec=0.2
            )
            # --- MODIFIED: Use PID alignment for stable reverse movement ---
            # The driving side depends on the last path taken to reach the corner.
            if self.last_avoidance_path_was_outer:
                # If last path was outer, simply follow the outer wall.
                self._execute_pid_alignment(
                    msg, 
                    self.approach_base_yaw_deg, 
                    is_outer_wall=True,
                    speed=-self.forward_speed,
                    disable_dist_control=True
                )
            else:
                # If last path was inner, the inner wall is gone.
                # Follow the OUTER wall instead, but maintain the ideal inner path distance.
                override_dist = 1.0 - self.align_target_inner_dist_m
                self._execute_pid_alignment(
                    msg,
                    self.approach_base_yaw_deg,
                    is_outer_wall=True, # Forcibly use the outer wall for alignment
                    speed=-self.forward_speed,
                    override_target_dist=override_dist,
                    disable_dist_control=True
                )

    def _handle_straight_sub_plan_next_avoidance(self, msg: LaserScan):
        """
        Manages the move-and-scan sequence for planning the next path.
        This function is structured in three phases:
        1. INITIALIZATION: Prepares the robot and camera for the sequence.
        2. DRIVING & SCANNING: Moves the robot forward and conditionally scans for obstacles.
        3. FINALIZATION: Stops the robot, analyzes collected data, and transitions to the TURNING state.
        """
        # --- PHASE 1: INITIALIZATION (runs only once per planning sequence) ---
        if not self.planning_initiated:
            self.get_logger().warn(">>> PLAN_NEXT_AVOIDANCE: Initiating move-and-scan sequence. <<<")
            self.planning_initiated = True
            self.detection_results.clear()

            self.max_red_blob_area = 0.0
            self.max_green_blob_area = 0.0
            self.max_red_blob_sample_num = 0
            self.max_green_blob_sample_num = 0
            self.scan_frame_count = 0

            # Instead of setting a static center position, calculate the correct
            # initial viewing angle and set it immediately.
            # This prevents the camera from briefly pointing forward.
            if self.enable_dynamic_tilt:
                # Determine the target viewing angle in the world frame.
                base_path_angle_deg = self.approach_base_yaw_deg
                
                if self.direction == 'ccw':
                    # Look left
                    viewing_offset_deg = 90.0
                else: # cw
                    # Look right
                    viewing_offset_deg = -90.0
                
                target_world_angle_deg = self._angle_normalize(base_path_angle_deg + viewing_offset_deg)

                # Calculate the initial servo position based on the current yaw
                # The logic is duplicated from _update_dynamic_tilt for immediate use.
                camera_relative_angle_deg = self._angle_diff(target_world_angle_deg, self.current_yaw_deg)
                tilt_offset = camera_relative_angle_deg * self.servo_units_per_degree
                initial_tilt_pos = self.tilt_center_position + tilt_offset
                clamped_initial_tilt = int(max(self.tilt_min_position, min(self.tilt_max_position, initial_tilt_pos)))

                self.get_logger().info(f"Setting initial dynamic tilt to {clamped_initial_tilt}.")
                self._set_camera_angle(
                    pan_position=self.initial_pan_position,
                    tilt_position=clamped_initial_tilt,
                    duration_sec=self.camera_move_duration
                )
                self.last_sent_tilt_position = clamped_initial_tilt # Update last sent position
            else:
                # Fallback to old static logic if dynamic tilt is disabled
                if self.direction == 'ccw':
                    static_tilt_pos = self.tilt_position_ccw
                else:
                    static_tilt_pos = self.tilt_position_cw
                self._set_camera_angle(
                    pan_position=self.initial_pan_position,
                    tilt_position=static_tilt_pos,
                    duration_sec=self.camera_move_duration
                )

        # --- PHASE 2: DRIVING & SCANNING ---
        next_segment_index = (self.wall_segment_index + 1) % len(self.entrance_obstacle_plan)
        if next_segment_index == 0:
            target_stop_dist = self.planning_scan_stop_dist_start_area
        else:
            target_stop_dist = self.planning_scan_stop_dist
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)

        # If the robot is still approaching the final stopping point, continue driving.
        if math.isnan(front_dist) or front_dist > target_stop_dist:
            # --- NEW: Update dynamic tilt to look at the side wall ---
            if self.enable_dynamic_tilt:
                # Determine the target viewing angle in the world frame.
                base_path_angle_deg = self.approach_base_yaw_deg
                
                if self.direction == 'ccw':
                    # Look left (perpendicular to the path)
                    viewing_offset_deg = 90.0
                else: # cw
                    # Look right (perpendicular to the path)
                    viewing_offset_deg = -90.0
                
                target_world_angle_deg = self._angle_normalize(base_path_angle_deg + viewing_offset_deg)
                
                # Call the generalized update function with the target angle
                self._update_dynamic_tilt(target_world_angle_deg)
            
            # Drive straight forward using IMU for heading correction
            # --- MODIFIED: Use PID alignment for stable forward movement ---
            if self.last_avoidance_path_was_outer:
                # If we were on the outer path, continue following the outer wall.
                self._execute_pid_alignment(msg, self.approach_base_yaw_deg, is_outer_wall=True,
                                            speed=self.course_detection_slow_speed)
            else:
                # If we were on the inner path, the inner wall is gone.
                # Align using the OUTER wall, but maintain the ideal INNER path distance.
                outer_wall_angle = self._angle_normalize(self.approach_base_yaw_deg - (90.0 if self.direction == 'ccw' else -90.0))
                outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
                
                override_dist = None
                if not math.isnan(outer_wall_dist):
                    # Target distance for the OUTER wall should be: CourseWidth - IdealInnerDist
                    # Assuming course width is approx 1.0m
                    override_dist = 1.0 - self.align_target_inner_dist_m
                
                self._execute_pid_alignment(msg, self.approach_base_yaw_deg, is_outer_wall=True,
                                            speed=self.course_detection_slow_speed,
                                            override_target_dist=override_dist)

            # Conditionally perform scanning only when close enough to the corner
            if not math.isnan(front_dist) and front_dist < self.planning_scan_start_dist_m:
                self._scan_and_collect_data()
            
            # --- LiDAR-based Entrance Blockage Detection (starts earlier) ---
            if not math.isnan(front_dist) and front_dist < self.lidar_entrance_scan_start_dist_m:
                next_segment_index = (self.wall_segment_index + 1) % len(self.entrance_obstacle_plan)
                # Only check if the plan for this segment is not already 'obstructed'
                if not self.entrance_obstacle_plan[next_segment_index]:
                    clearance_threshold_min = 0.85
                    clearance_threshold_max = 1.1
                    if self.direction == 'ccw':
                        inner_wall_angle = self._angle_normalize(self.approach_base_yaw_deg + 90.0)
                        outer_wall_angle = self._angle_normalize(self.approach_base_yaw_deg - 90.0)
                    else: # cw
                        inner_wall_angle = self._angle_normalize(self.approach_base_yaw_deg - 90.0)
                        outer_wall_angle = self._angle_normalize(self.approach_base_yaw_deg + 90.0)
                    
                    # Use a narrower scan for safety while moving
                    inner_dist = self.get_closest_distance_in_range(msg, inner_wall_angle, 1.0)
                    # inner_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
                    outer_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
                    
                    if inner_dist is not None and not math.isnan(outer_dist):
                        total_dist = inner_dist['distance'] + outer_dist
                        if clearance_threshold_min < total_dist < clearance_threshold_max:
                            self.get_logger().error(
                                f"[ENTRANCE BLOCKED DETECTED WHILE MOVING] "
                                f"Clearance: {total_dist:.3f}m. Flagging for careful turn.f:{front_dist:.3f} i:{inner_dist['distance']:.3f} o:{outer_dist:.3f}"
                            )
                            self.entrance_obstacle_plan[next_segment_index] = True
            
            return

        # --- PHASE 3: FINALIZATION (executes once the robot stops) ---
        self.publish_twist_with_gain(0.0, 0.0)
        self.get_logger().info(f"Move-and-scan complete. Analyzed {len(self.detection_results)} frames. Analyzing max blob areas found...")

        # --- MODIFICATION: Analyze based on max blob areas, not majority vote ---
        effective_threshold = self._get_effective_detection_threshold(msg)
        
        final_pattern = self._get_obstacle_pattern_from_areas(
            self.max_red_blob_area,
            self.max_green_blob_area,
            effective_threshold
        )
        
        log_msg = (
            f"\n--- Planning Analysis Results (Turn {self.turn_count + 1}) ---\n"
            f"  - Frames Analyzed        : {len(self.detection_results)}\n"
            f"  - Max Red Blob Area Found  : {self.max_red_blob_area:.1f} (at sample #{self.max_red_blob_sample_num})\n"
            f"  - Max Green Blob Area Found: {self.max_green_blob_area:.1f} (at sample #{self.max_green_blob_sample_num})\n"
            f"  - Detection Threshold      : {effective_threshold:.1f}\n"
            f"  - Final Decision           : '{final_pattern}'\n"
            f"-----------------------------------------"
        )
        self.get_logger().info(log_msg)

        # Update the avoidance plan based on the single, final pattern
        self._update_avoidance_plan_based_on_vision(final_pattern)

        # If this scan was for the parking segment, decide the final approach lane now.
        # Assuming parking is on segment 0, this scan happens when approaching corner 3 (wall_segment_index == 3).
        # Therefore, the *next* segment index will be 0.
        next_segment_index = (self.wall_segment_index + 1) % len(self.avoidance_path_plan)
        parking_segment_index = self.max_turns % len(self.avoidance_path_plan) # Assuming parking is always on segment 0

        if next_segment_index == parking_segment_index:
            self.get_logger().warn("This scan is for the parking segment. Determining final approach lane.")
            has_entrance_obstacle = self.entrance_obstacle_plan[parking_segment_index]
            
            if has_entrance_obstacle:
                # Follow the vision-based plan
                path_type = self._get_path_type_for_segment(parking_segment_index)
                if path_type in ['inner', 'inner_to_outer']:
                    self.final_approach_lane_is_outer = False
                else: # 'outer', 'outer_to_inner'
                    self.final_approach_lane_is_outer = True
            else:
                # Follow the special rule
                if self.direction == 'ccw':
                    self.final_approach_lane_is_outer = True
                else: # cw
                    self.final_approach_lane_is_outer = True # False
            
            self.get_logger().warn(f"Final approach lane decision: {'OUTER' if self.final_approach_lane_is_outer else 'INNER'}")

        # Prepare for the turning maneuver based on the new plan
        self._prepare_for_turning(base_angle_deg=self.approach_base_yaw_deg)

        # Decide if the full turning maneuver can be skipped
        self.get_logger().info("Planning complete. Proceeding with full turn maneuver.")
        plan_status_str = ", ".join([f"Seg{i}:{'Blocked' if blocked else 'Clear'}" for i, blocked in enumerate(self.entrance_obstacle_plan)])
        self.get_logger().warn(f"[Entrance Plan Status] {plan_status_str}")
        self.turning_sub_state = TurningSubState.POSITIONING_REVERSE
        
        self.state = State.TURNING

    # --- Turning Sub-States ---
    def _handle_turning_sub_positioning_reverse(self, msg: LaserScan):
        """
        Sub-state: Before approaching, reverse straight until the robot is in
        a standard position to start the turn maneuver.
        """
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)

        if math.isnan(front_dist):
            self.get_logger().warn("Positioning Reverse: Cannot get front distance. Reversing slowly.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(-self.course_detection_slow_speed * 0.8, 0.0)
            return

        # --- Check the two completion conditions ---
        condition1 = 0.85 < front_dist < 0.95

        if self.direction == 'ccw':
            left_angle = self._angle_normalize(self.approach_base_yaw_deg + 90.0)
            right_angle = self._angle_normalize(self.approach_base_yaw_deg - 90.0)
        else: # cw
            left_angle = self._angle_normalize(self.approach_base_yaw_deg - 90.0)
            right_angle = self._angle_normalize(self.approach_base_yaw_deg + 90.0)
        
        left_dist = self.get_distance_at_world_angle(msg, left_angle)
        right_dist = self.get_distance_at_world_angle(msg, right_angle)

        condition2 = False
        if not math.isnan(left_dist) and not math.isnan(right_dist):
            side_dist_sum = left_dist + right_dist
            if side_dist_sum <= 1.0 and front_dist > 0.8:
                condition2 = True

        # --- If either condition is met, move to APPROACH_CORNER ---
        if condition1 or condition2:
            self.get_logger().info(f"Positioning reverse complete (Front: {front_dist:.2f}m). Transitioning to APPROACH_CORNER.")
            self.publish_twist_with_gain(0.0, 0.0)
            self.turning_sub_state = TurningSubState.APPROACH_CORNER
        else:
            self.get_logger().debug(f"Positioning Reverse... (Front: {front_dist:.2f}m)", throttle_duration_sec=0.2)
            # --- MODIFIED: Use PID alignment for stable reverse movement ---
            # The wall to follow is determined by the path taken before this turn.
            if self.last_avoidance_path_was_outer:
                # If last path was outer, simply follow the outer wall.
                self._execute_pid_alignment(
                    msg,
                    self.approach_base_yaw_deg,
                    is_outer_wall=True,
                    speed=-self.forward_speed * 0.7,
                    disable_dist_control=True
                )
            else:
                # If last path was inner, follow the OUTER wall with an adjusted target distance.
                override_dist = 1.0 - self.align_target_inner_dist_m
                self._execute_pid_alignment(
                    msg,
                    self.approach_base_yaw_deg,
                    is_outer_wall=True, # Forcibly use the outer wall
                    speed=-self.forward_speed * 0.7,
                    override_target_dist=override_dist,
                    disable_dist_control=True
                )

    def _handle_turning_sub_approach_corner(self, msg: LaserScan):
        """
        Sub-state: Moves forward towards the corner wall until the dynamically
        determined trigger distance for turning is reached.
        """
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)
        if math.isnan(front_dist):
            self.get_logger().warn("Cannot get front distance for turn approach, stopping.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # --- MODIFIED: Get trigger distance dynamically from the helper ---
        trigger_dist, _, approach_speed, _ = self._get_turn_strategy()

        # Check for completion of this sub-state
        if front_dist <= trigger_dist:
            self.get_logger().info(f"Front wall is close ({front_dist:.2f}m <= {trigger_dist:.2f}m). Transitioning to EXECUTE_PIVOT_TURN.")
            self.turning_sub_state = TurningSubState.EXECUTE_PIVOT_TURN
            self.publish_twist_with_gain(0.0, 0.0) # Stop briefly before turning
            return

        # --- Continue approaching the wall ---
        self.get_logger().debug(f"Approaching wall for turn... Dist: {front_dist:.2f}m", throttle_duration_sec=0.2)
        
        if self.last_avoidance_path_was_outer:
            self._execute_pid_alignment(
                msg, self.approach_base_yaw_deg, is_outer_wall=True, speed=approach_speed
            )
        else:
            override_dist = 1.0 - self.align_target_inner_dist_m
            self._execute_pid_alignment(
                msg, self.approach_base_yaw_deg, is_outer_wall=True,
                speed=approach_speed, override_target_dist=override_dist
            )

    def _handle_turning_sub_execute_pivot_turn(self, msg: LaserScan):
        """
        Sub-state: Executes a pivot turn with Proportional control.
        The turn amount is dynamically determined by the turn strategy.
        """
        # --- 1. Get turn strategy (unchanged) ---
        _, turn_amount_deg, _, base_turn_speed = self._get_turn_strategy()
        
        # --- 2. Calculate remaining angle (error) ---
        angle_turned_deg = abs(self._angle_diff(self.current_yaw_deg, self.approach_base_yaw_deg))
        remaining_angle_deg = turn_amount_deg - angle_turned_deg

        # --- 3. Check for completion ---
        completion_threshold_deg = 2.0 # Allow for a small error
        if remaining_angle_deg <= completion_threshold_deg:
            self.get_logger().info(f"Pivot turn complete (Rem Angle: {remaining_angle_deg:.1f} deg). Finalizing turn.")
            self.turning_sub_state = TurningSubState.FINALIZE_TURN
            self.publish_twist_with_gain(0.0, 0.0) # Stop motion
            return
        
        # --- 4. Proportional Steering Control ---
        # P-gain for turning. Needs tuning. A larger value makes it turn sharper.
        turn_kp = 0.02
        
        # Calculate steering based on remaining angle
        proportional_steer = turn_kp * remaining_angle_deg
        
        # Apply direction (left/right)
        if self.direction == 'ccw':
            steer_direction = 1.0
        else: # cw
            steer_direction = -1.0
            
        final_steer = proportional_steer * steer_direction
        
        # Limit the steering to the maximum possible value
        final_steer = np.clip(final_steer, -self.max_steer, self.max_steer)

        # --- Add Proportional Speed Control ---
        min_turn_speed = base_turn_speed * 0.7 # Minimum speed (e.g., 40% of original)
        
        # Calculate speed based on how close we are to the target
        # (angle_turned_deg / turn_amount_deg) goes from 0 to 1 as the turn progresses
        speed_reduction_factor = max(0, 1.0 - (angle_turned_deg / turn_amount_deg))
        
        final_speed = min_turn_speed + (base_turn_speed - min_turn_speed) * speed_reduction_factor
        # ------------------------------------

        # --- 5. Continue turning with adjusted speed and steer ---
        self.publish_twist_with_gain(final_speed, final_steer)

    def _handle_turning_sub_finalize_turn(self):
        """
        Finalizes the turn and transitions to the next state.
        Uses a pre-determined flag for the final parking approach lane.
        """
        # Determine if this is the final turn before the parking segment
        is_final_turn = self.turn_count == (self.max_turns - 1)

        # Increment turn count and update segment index for the upcoming straight section
        self.turn_count += 1
        self.wall_segment_index = (self.wall_segment_index + 1) % 4
        self.state = State.STRAIGHT

        if is_final_turn:
            self.get_logger().warn(f"Final turn ({self.turn_count}) complete. Using pre-determined lane for parking approach.")
            
            if self.final_approach_lane_is_outer is None:
                # Failsafe: if the flag was not set for some reason, default to a safe option (outer lane)
                self.get_logger().error("Final approach lane flag was not set! Defaulting to OUTER lane.")
                self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
            elif self.final_approach_lane_is_outer:
                self.get_logger().info("-> Executing plan: Align with OUTER wall.")
                self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
            else: # self.final_approach_lane_is_outer is False
                self.get_logger().info("-> Executing plan: Align with INNER wall.")
                self.straight_sub_state = StraightSubState.ALIGN_WITH_INNER_WALL

        else:
            # --- Original logic for all other turns ---
            self.get_logger().info(f"Turn {self.turn_count} complete. Entering new segment index: {self.wall_segment_index}")

            self.state = State.STRAIGHT
            # Use a flexible helper to get the path type for the NEXT segment
            next_path_type = self._get_path_type_for_segment(self.wall_segment_index, default_path='outer')

            if next_path_type == 'inner' or next_path_type == 'inner_to_outer':
                self.get_logger().warn(f"Plan for segment {self.wall_segment_index}: Starts INNER. Transitioning to ALIGN_WITH_INNER_WALL")
                self.straight_sub_state = StraightSubState.ALIGN_WITH_INNER_WALL

            else: # 'outer', 'outer_to_inner', or any other unexpected string
                log_start_type = "OUTER" if next_path_type in ['outer', 'outer_to_inner'] else f"UNKNOWN('{next_path_type}'), defaulting to OUTER"
                self.get_logger().warn(f"Plan for segment {self.wall_segment_index}: Starts {log_start_type}. Transitioning to ALIGN_WITH_OUTER_WALL.")
                self.straight_sub_state = StraightSubState.ALIGN_WITH_OUTER_WALL
        
        # --- Common reset logic for any turn completion ---
        self.planning_initiated = False
        self.is_in_avoidance_alignment = True
        self.is_passing_obstacle = False
        self.inner_wall_far_counter = 0 
        self.turning_sub_state = None
        self.publish_twist_with_gain(0.0, 0.0)

    # --- Parking Sub-States ---
    def _handle_parking_sub_prepare_parking(self):
        """
        Sub-state: First step of the parking sequence.
        Logs the vehicle's state and decides the next sub-state based on direction.
        """
        # (Logging part is unchanged)
        log_message = (
            f"\n"
            f"--- PREPARING PARKING SEQUENCE ---\n"
            f"  - Final Lap Direction : {self.direction.upper()}\n"
            f"  - Final Approach Lane : {'OUTER' if self.final_approach_lane_is_outer else 'INNER'}\n"
            f"  - Obstacle at Parking : {self.has_obstacle_at_parking_exit}\n"
            f"------------------------------------"
        )
        self.get_logger().warn(log_message)

        # Branch to the next sub-state based on the situation.
        if self.direction == "cw":
            self.get_logger().info("Dir: CW -> Action: U-Turn. Transitioning to REORIENT_FOR_PARKING.")
            self.parking_sub_state = ParkingSubState.REORIENT_FOR_PARKING
        else: # "ccw"
            if not self.final_approach_lane_is_outer:
                self.get_logger().info("Dir: CCW, Lane: INNER -> Action: Lane Change. Transitioning to LANE_CHANGE_FOR_PARKING.")
                self.parking_sub_state = ParkingSubState.LANE_CHANGE_FOR_PARKING
            else: # CCW and on the Outer lane
                self.get_logger().info("Dir: CCW, Lane: OUTER -> Action: Proceed. Transitioning to APPROACH_PARKING_START.")
                self.parking_sub_state = ParkingSubState.APPROACH_PARKING_START

    def _handle_parking_sub_reorient_for_parking(self, msg: LaserScan):
        """
        Sub-state: Executes a U-turn maneuver to face the CCW direction.
        Acts as a dispatcher for the reorientation steps.
        """
        # --- Initialize on first entry ---
        if self.reorient_step is None:
            self.reorient_base_yaw_deg = self._calculate_base_angle()
            self.reorient_step = ReorientStep.INITIAL_FORWARD_APPROACH
        
        # --- Dispatch to the correct step handler ---
        if self.reorient_step == ReorientStep.INITIAL_FORWARD_APPROACH:
            self._reorient_step_initial_forward(msg)
        
        elif self.reorient_step in [ReorientStep.INNER_TURN_1, ReorientStep.INNER_STRAIGHT1, ReorientStep.INNER_STRAIGHT2, ReorientStep.INNER_TURN_2]:
            self._reorient_inner_lane_maneuver(msg)
        
        elif self.reorient_step in [ReorientStep.OUTER_TURN_1, ReorientStep.OUTER_REVERSE_TURN]:
            self._reorient_outer_lane_maneuver(msg)

        # --- Handle Completion ---
        # This check is done after any of the above steps might have completed.
        if self.reorient_step == ReorientStep.COMPLETED:
            self.get_logger().warn("Reorientation maneuver complete. Now facing CCW direction.")
            self.direction = "ccw" # CRITICAL: Update the global direction
            self.last_avoidance_path_was_outer = True
            self.can_start_new_turn = False
            self.stable_alignment_counter = 0
            self.reorient_step = None # Reset for next time
            self.parking_sub_state = ParkingSubState.APPROACH_PARKING_START

    def _handle_parking_sub_lane_change_for_parking(self, msg: LaserScan):
        """
        Sub-state: Executes a maneuver to move from inner to outer lane for parking.
        This is a self-contained maneuver, using common helper functions.
        """
        yaw_tolerance_deg = self.reorient_yaw_tolerance_deg
        
        # --- Initialize on first entry ---
        if self.reorient_step is None:
            self.get_logger().info("Lane Change for Parking: Initializing maneuver.")
            self.reorient_base_yaw_deg = self._calculate_base_angle()
            self.reorient_step = ReorientStep.LC_INITIAL_FORWARD
        
        # --- Dispatch to the correct step ---
        if self.reorient_step == ReorientStep.LC_INITIAL_FORWARD:
            self._execute_parking_initial_forward(
                msg=msg,
                target_dist=self.lane_change_initial_approach_dist_m, # Target distance specific to this maneuver
                is_outer=False,   # Lane change always starts from the inner lane
                next_step=ReorientStep.LC_REVERSE_TURN_1
            )
        
        elif self.reorient_step == ReorientStep.LC_REVERSE_TURN_1:
            target_yaw = self._angle_normalize(self.reorient_base_yaw_deg + 90.0)
            is_turn_done = self._execute_p_controlled_turn(
                target_yaw_deg=target_yaw,
                tolerance_deg=yaw_tolerance_deg,
                base_yaw_deg=self.reorient_base_yaw_deg,
                turn_angle_deg=90.0,
                base_speed=self.reorient_reverse_speed
            )
            if is_turn_done:
                self.get_logger().info("Lane Change: Reverse 90-degree turn complete.")
                self.reorient_step = ReorientStep.LC_STRAIGHT_1

        elif self.reorient_step in [ReorientStep.LC_STRAIGHT_1, ReorientStep.LC_STRAIGHT_2]:
            self._execute_s_turn_straight_leg(
                msg=msg,
                base_yaw_for_wall_angle=(self.reorient_base_yaw_deg + 90.0),
                next_step_on_completion=ReorientStep.LC_REVERSE_TURN_2
            )
            
        elif self.reorient_step == ReorientStep.LC_REVERSE_TURN_2:
            turn2_base_yaw = self._angle_normalize(self.reorient_base_yaw_deg + 90.0)
            target_yaw = self._angle_normalize(self.reorient_base_yaw_deg)
            is_turn_done = self._execute_p_controlled_turn(
                target_yaw_deg=target_yaw,
                tolerance_deg=yaw_tolerance_deg,
                base_yaw_deg=turn2_base_yaw,
                turn_angle_deg=-90.0,
                base_speed=self.reorient_reverse_speed
            )
            if is_turn_done:
                self.get_logger().info("Lane Change: Reverse turn 2 (CW) complete.")
                self.reorient_step = ReorientStep.COMPLETED

        # --- Handle Completion ---
        if self.reorient_step == ReorientStep.COMPLETED:
            self.get_logger().warn("Lane Change for Parking complete. Now on the outer lane.")
            self.last_avoidance_path_was_outer = True # CRITICAL: Update lane status
            self.reorient_step = None # Reset for next time
            self.parking_sub_state = ParkingSubState.APPROACH_PARKING_START

    def _handle_parking_sub_approach_parking_start(self, msg: LaserScan):
        """
        Sub-state: Moves the robot to a precise starting position for parking
        by proportionally slowing down as it approaches the front wall.
        """
        base_angle_deg = self._calculate_base_angle()
        front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)

        # --- Check for completion ---
        if not math.isnan(front_dist) and front_dist < self.parking_approach_final_stop_dist_m:
            self.get_logger().warn(f"Approach Parking: Final position reached (Dist: {front_dist:.3f}m).")
            self.publish_twist_with_gain(0.0, 0.0)

            # Store the current yaw as the base for the final parking maneuver
            self.parking_base_yaw_deg = self.current_yaw_deg
            
            self.get_logger().info("Transitioning to EXECUTE_PARKING_MANEUVER.")
            self.approach_step = None # Reset for next time
            self.parking_sub_state = ParkingSubState.EXECUTE_PARKING_MANEUVER
            return

        # --- Proportional Speed Control ---
        fast_speed = self.forward_speed * 0.7
        slow_speed = self.parking_approach_slow_speed
        start_slowdown_dist = self.parking_approach_slowdown_dist_m
        end_slowdown_dist = self.parking_approach_final_stop_dist_m
        
        approach_speed = fast_speed # Default to fast speed

        if not math.isnan(front_dist):
            if front_dist <= start_slowdown_dist:
                # We are in the slowdown zone, calculate speed proportionally.
                
                # This value goes from 0.0 (at start_slowdown_dist) to 1.0 (at end_slowdown_dist)
                slowdown_range = start_slowdown_dist - end_slowdown_dist
                if slowdown_range <= 0: slowdown_range = 0.1 # Avoid division by zero
                
                progress_ratio = (start_slowdown_dist - front_dist) / slowdown_range
                progress_ratio = np.clip(progress_ratio, 0.0, 1.0) # Ensure it's between 0 and 1
                
                # Linearly interpolate speed between fast_speed and slow_speed
                approach_speed = fast_speed - (fast_speed - slow_speed) * progress_ratio

        # --- Safety Checks (use_imu_only) ---
        use_imu_only = False
        # If we are in the slowdown zone, it's safer to use IMU only for alignment.
        if not math.isnan(front_dist) and front_dist < self.parking_approach_slowdown_dist_m:
            use_imu_only = True
        
        # Additional safety check for abnormal course width
        if not use_imu_only:
            inner_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
            inner_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
            outer_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)

            if not math.isnan(inner_dist) and not math.isnan(outer_dist):
                course_width = inner_dist + outer_dist
                if not (0.9 < course_width < 1.1):
                    use_imu_only = True
                    self.get_logger().warn(f"Approach Fwd: Abnormal width ({course_width:.2f}m), using IMU_ONLY.", throttle_duration_sec=1.0)
            else:
                use_imu_only = True
                self.get_logger().warn("Approach Fwd: One side wall not visible, using IMU_ONLY.", throttle_duration_sec=1.0)

        # --- Execute PID alignment ---
        self._execute_pid_alignment(
            msg=msg,
            base_angle_deg=base_angle_deg,
            is_outer_wall=True,
            speed=approach_speed,
            override_target_dist=self.parking_approach_target_outer_dist_m,
            disable_dist_control=use_imu_only
        )

    def _handle_parking_sub_execute_parking_maneuver(self, msg: LaserScan):
        """
        Sub-state: Executes the final 3-step parallel parking maneuver.
        """
        # --- Initialize on first entry ---
        if self.parking_maneuver_step is None:
            self.get_logger().warn("--- Executing Final Parking Maneuver ---")
            self.parking_maneuver_step = ParkingManeuverStep.STEP1_REVERSE_TURN
        # --- Dispatch to the correct step handler ---
        if self.parking_maneuver_step == ParkingManeuverStep.STEP1_REVERSE_TURN:
            self._parking_step1_reverse_turn(msg)
        elif self.parking_maneuver_step == ParkingManeuverStep.STEP2_REVERSE_STRAIGHT:
            self._parking_step2_reverse_straight(msg)
        elif self.parking_maneuver_step == ParkingManeuverStep.STEP3_ALIGN_TURN:
            self._parking_step3_align_turn(msg)
        elif self.parking_maneuver_step == ParkingManeuverStep.STEP4_FINAL_ADJUST:
            self._parking_step4_final_adjust(msg)

    # --- Reorientation Helper Functions ---
    def _reorient_step_initial_forward(self, msg: LaserScan):
        """
        Helper for the INITIAL_FORWARD_APPROACH step. 
        Calls the generic forward function and handles the state transition.
        """
        # Determine target distance for this specific maneuver (U-Turn)
        # Decide and transition to the next step
        if not self.final_approach_lane_is_outer: # Inner Lane
            target_dist = self.reorient_initial_approach_dist_inner_m
            next_step = ReorientStep.INNER_TURN_1
        else: # Outer Lane
            target_dist = self.reorient_initial_approach_dist_outer_m
            next_step = ReorientStep.OUTER_TURN_1

        # Execute the forward movement
        is_complete = self._execute_parking_initial_forward(
            msg=msg,
            target_dist=target_dist,
            is_outer=self.final_approach_lane_is_outer,
            next_step=next_step
        )

    def _reorient_inner_lane_maneuver(self, msg: LaserScan):
        """Helper for Reorientation: S-Turn from the inner lane to reorient."""
        yaw_tolerance_deg = self.reorient_yaw_tolerance_deg
        
        if self.reorient_step == ReorientStep.INNER_TURN_1:
            target_yaw = self._angle_normalize(self.reorient_base_yaw_deg - 90.0)
            is_turn_done = self._execute_p_controlled_turn(
                target_yaw_deg=target_yaw,
                tolerance_deg=yaw_tolerance_deg,
                base_yaw_deg=self.reorient_base_yaw_deg,
                turn_angle_deg=-90.0,
                base_speed=self.reorient_reverse_speed
            )
            if is_turn_done:
                self.get_logger().info("Reorientation (Inner): Turn 1 complete.")
                self.reorient_step = ReorientStep.INNER_STRAIGHT1

        elif self.reorient_step in [ReorientStep.INNER_STRAIGHT1, ReorientStep.INNER_STRAIGHT2]:
            self._execute_s_turn_straight_leg(
                msg=msg,
                base_yaw_for_wall_angle=(self.reorient_base_yaw_deg - 90.0),
                next_step_on_completion=ReorientStep.INNER_TURN_2
            )

        elif self.reorient_step == ReorientStep.INNER_TURN_2:
            turn2_base_yaw = self._angle_normalize(self.reorient_base_yaw_deg - 90.0)
            target_yaw = self._angle_normalize(self.reorient_base_yaw_deg - 180.0)
            is_turn_done = self._execute_p_controlled_turn(
                target_yaw_deg=target_yaw,
                tolerance_deg=yaw_tolerance_deg,
                base_yaw_deg=turn2_base_yaw,
                turn_angle_deg=-90.0,
                base_speed=self.reorient_reverse_speed
            )
            if is_turn_done:
                self.get_logger().info("Reorientation (Inner): Turn 2 complete.")
                self.reorient_step = ReorientStep.COMPLETED

    def _reorient_outer_lane_maneuver(self, msg: LaserScan):
        """Helper for Reorientation: 3-Point Turn from the outer lane."""
        yaw_tolerance_deg = self.reorient_yaw_tolerance_deg

        if self.reorient_step == ReorientStep.OUTER_TURN_1:
            target_yaw = self._angle_normalize(self.reorient_base_yaw_deg - 90.0)
            is_turn_done = self._execute_p_controlled_turn(
                target_yaw_deg=target_yaw,
                tolerance_deg=yaw_tolerance_deg,
                base_yaw_deg=self.reorient_base_yaw_deg,
                turn_angle_deg=90.0 # This seems incorrect, should be -90.0 for CW turn
            )
            if is_turn_done:
                self.get_logger().info("Reorientation (Outer): Forward turn complete.")
                self.reorient_step = ReorientStep.OUTER_REVERSE_TURN

        elif self.reorient_step == ReorientStep.OUTER_REVERSE_TURN:
            reverse_base_yaw = self._angle_normalize(self.reorient_base_yaw_deg - 90.0)
            target_yaw = self._angle_normalize(self.reorient_base_yaw_deg - 180.0)
            
            is_turn_done = self._execute_p_controlled_turn(
                target_yaw_deg=target_yaw,
                tolerance_deg=yaw_tolerance_deg,
                base_yaw_deg=reverse_base_yaw,
                turn_angle_deg=-90.0,
                base_speed=self.reorient_reverse_speed
            )
            if is_turn_done:
                self.get_logger().info("Reorientation (Outer): Reverse turn complete.")
                self.reorient_step = ReorientStep.COMPLETED

    def _parking_step1_reverse_turn(self, msg: LaserScan):
        """Parking Step 1: Reverse while turning 45 degrees into the space."""
        # Target yaw is 45 degrees CCW from the starting orientation
        target_yaw = self._angle_normalize(self.parking_base_yaw_deg + self.parking_step1_target_angle_deg)
        yaw_error_deg = self._angle_diff(target_yaw, self.current_yaw_deg)
        
        # Check for completion
        if abs(yaw_error_deg) < self.reorient_yaw_tolerance_deg:
            self.get_logger().info("Parking Step 1 (Reverse Turn): Complete.")
            self.publish_twist_with_gain(0.0, 0.0)
            self.parking_maneuver_step = ParkingManeuverStep.STEP2_REVERSE_STRAIGHT
            return
            
        # P-control for steering while reversing
        # To turn the rear to the left while reversing, we need to steer right (positive steer).
        # A positive error (we need to turn left) should result in a positive steer.
        turn_kp = 0.05 # Use a positive gain
        steer = np.clip(turn_kp * yaw_error_deg, -self.max_steer, self.max_steer)
        
        self.publish_twist_with_gain(self.parking_step1_reverse_speed, steer)

    def _parking_step2_reverse_straight(self, msg: LaserScan):
        """
        Parking Step 2: Reverse straight until the front wall is no longer detected.
        """
        # The orientation should be maintained at 45 degrees
        target_yaw = self._angle_normalize(self.parking_base_yaw_deg + self.parking_step1_target_angle_deg)
        
        # --- Completion Check using front LiDAR ---
        # "Front" is relative to the original orientation before parking
        front_wall_angle = self.parking_base_yaw_deg
        front_dist = self.get_distance_at_world_angle(msg, front_wall_angle)
        
        # Trigger when the front wall becomes distant (i.e., we've entered the space)
        if not math.isnan(front_dist) and front_dist > self.parking_step2_front_dist_trigger_m:
            self.get_logger().info(f"Parking Step 2 (Reverse Straight): Complete (Front wall lost, dist: {front_dist:.3f}m).")
            self.publish_twist_with_gain(0.0, 0.0)
            self.parking_maneuver_step = ParkingManeuverStep.STEP3_ALIGN_TURN
            return
            
        # --- Drive straight back, maintaining the 45-degree angle ---
        yaw_error_deg = self._angle_diff(target_yaw, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * yaw_error_deg
        final_steer = np.clip(angle_steer, -self.max_steer, self.max_steer)
        
        self.get_logger().debug(f"Parking Step 2: Reversing straight... Front dist: {front_dist:.3f}m", throttle_duration_sec=0.2)
        self.publish_twist_with_gain(self.parking_step2_reverse_speed, final_steer)
        
    def _parking_step3_align_turn(self, msg: LaserScan):
        """Parking Step 3: Reverse while turning to become parallel to the wall."""
        # Target yaw is the original straight orientation
        target_yaw = self.parking_base_yaw_deg
        yaw_error_deg = self._angle_diff(target_yaw, self.current_yaw_deg)
        
        # Check for completion
        if abs(yaw_error_deg) < self.reorient_yaw_tolerance_deg:
            self.get_logger().info("Parking Step 3 (Align Turn): Complete.")
            self.publish_twist_with_gain(0.0, 0.0)
            self.parking_maneuver_step = ParkingManeuverStep.STEP4_FINAL_ADJUST
            return
            
        # Reverse with opposite steering (full steer to the right)
        self.publish_twist_with_gain(self.parking_step3_reverse_speed, -self.max_steer)

    def _parking_step4_final_adjust(self, msg: LaserScan):
        """
        Parking Step 4: Send a single forward command with zero steer to straighten the wheels,
        then immediately finish the maneuver.
        """
        self.get_logger().info("Parking Step 4: Sending final command to straighten wheels.")

        # Send a single, brief forward command with zero steering.
        # This ensures the wheels are pointing straight when parking is complete.
        self.publish_twist_with_gain(self.parking_step4_forward_speed, 0.0)

        # After sending the command, immediately transition to the finished state.
        # A very short delay might help ensure the command is sent before shutdown.
        # time.sleep(0.1) # Optional, usually not needed.

        self.get_logger().warn("--- PARKING MANEUVER COMPLETE ---")
        self.state = State.FINISHED

    def _execute_parking_initial_forward(self, msg: LaserScan, target_dist: float, is_outer: bool, next_step: Enum):
        """
        Generic helper to move forward to a target distance before a parking maneuver.
        Returns True when complete.
        """
        base_yaw = self.reorient_base_yaw_deg
        front_dist = self.get_distance_at_world_angle(msg, base_yaw)

        if not math.isnan(front_dist) and front_dist < target_dist:
            self.get_logger().info(f"Initial Forward: Approach complete (Dist: {front_dist:.2f}m).")
            self.publish_twist_with_gain(0.0, 0.0)
            self.reorient_step = next_step
            return True # Complete
        
        # --- Drive forward with safety checks ---
        use_imu_only = False
        if not math.isnan(front_dist) and front_dist < 1.1 : # Check a bit before the target
            use_imu_only = True
        if not use_imu_only:
            # Get side wall distances. Note: self.direction is still 'cw' here.
            inner_wall_angle = self._angle_normalize(base_yaw - 90.0)
            outer_wall_angle = self._angle_normalize(base_yaw + 90.0)
            
            inner_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
            outer_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)

            if not math.isnan(inner_dist) and not math.isnan(outer_dist):
                course_width = inner_dist + outer_dist
                if not (0.9 < course_width < 1.1):
                    use_imu_only = True
                    self.get_logger().warn(f"Initial Fwd: Abnormal width ({course_width:.2f}m), using IMU_ONLY.", throttle_duration_sec=1.0)
            else:
                use_imu_only = True
                self.get_logger().warn("Initial Fwd: One side wall not visible, using IMU_ONLY.", throttle_duration_sec=1.0)
        
        self._execute_pid_alignment(
            msg, base_yaw, is_outer, 
            speed=self.forward_speed * 0.7, 
            disable_dist_control=use_imu_only
        )
        self.get_logger().info(f"Initial Forward: Approach complete (Dist: {front_dist:.2f}m, Target:{target_dist:.2f}).", throttle_duration_sec=0.1)
        return False # Ongoing

    def _execute_p_controlled_turn(self, target_yaw_deg, tolerance_deg, base_yaw_deg, turn_angle_deg, base_speed=0.2):
        """
        A generic helper to perform a P-controlled turn (forward or reverse).
        Gradually reduces speed as it approaches the target.
        - To reverse, provide a negative base_speed.
        Returns True when the turn is complete.
        """
        yaw_error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)

        if abs(yaw_error_deg) < tolerance_deg:
            self.publish_twist_with_gain(0.0, 0.0)
            return True # Turn is complete

        # --- Proportional Steering Control ---
        turn_kp = self.reorient_turn_kp
        steer = np.clip(turn_kp * yaw_error_deg, -self.max_steer, self.max_steer)

        # --- Proportional Speed Control ---
        # The minimum speed maintains the same sign as the base speed.
        min_speed = base_speed * 0.5
        
        # Calculate how much of the turn has been completed
        angle_turned_deg = abs(self._angle_diff(self.current_yaw_deg, base_yaw_deg))
        
        total_turn_angle = abs(turn_angle_deg)
        if total_turn_angle < 1.0:
            total_turn_angle = 1.0
        
        # Calculate speed reduction based on progress
        progress_ratio = min(1.0, angle_turned_deg / total_turn_angle)
        final_speed = base_speed - (base_speed - min_speed) * progress_ratio
        
        # Execute the turn
        self.publish_twist_with_gain(final_speed, steer)
        return False # Turn is ongoing
    
    def _execute_s_turn_straight_leg(self, msg: LaserScan, base_yaw_for_wall_angle, next_step_on_completion):
        """
        Helper for S-Turn maneuvers: Handles the straight forward/reverse parts.
        This function contains two internal steps: moving forward, then reversing.
        """
        # Wall angle is perpendicular to the original approach direction
        wall_angle = self._angle_normalize(base_yaw_for_wall_angle)
        wall_dist = self.get_distance_at_world_angle(msg, wall_angle)

        # STEP 1: Move forward until very close to the wall
        if self.reorient_step in [ReorientStep.INNER_STRAIGHT1, ReorientStep.LC_STRAIGHT_1]:
            if not math.isnan(wall_dist) and wall_dist < self.reorient_s_turn_fwd_dist_m:
                self.get_logger().info(f"S-Turn Straight: Forward leg complete (Dist: {wall_dist:.2f}m).")
                self.publish_twist_with_gain(0.0, 0.0)
                # Transition to the reverse part of this straight leg
                if self.reorient_step == ReorientStep.INNER_STRAIGHT1:
                    self.reorient_step = ReorientStep.INNER_STRAIGHT2
                else: # LC_STRAIGHT_1
                    self.reorient_step = ReorientStep.LC_STRAIGHT_2
            else:
                self.publish_twist_with_gain(self.forward_speed * 0.7, 0.0)
        
        # STEP 2: Reverse until a bit further from the wall
        elif self.reorient_step in [ReorientStep.INNER_STRAIGHT2, ReorientStep.LC_STRAIGHT_2]:
            if not math.isnan(wall_dist) and wall_dist > self.reorient_s_turn_rev_dist_m:
                self.get_logger().info(f"S-Turn Straight: Reverse leg complete (Dist: {wall_dist:.2f}m).")
                self.publish_twist_with_gain(0.0, 0.0)
                # This straight leg is fully complete, transition to the next turn
                self.reorient_step = next_step_on_completion
            else:
                self.publish_twist_with_gain(-self.forward_speed * 0.7, 0.0)

    # --- Old Parking Sub-States (deactivated) ---
    def _handle_parking_sub_pre_parking_adjust(self, msg: LaserScan):
        """
        Sub-state: Final approach to the pre-parking position.
        For CW, it overshoots and then reverses to the target for better precision.
        For CCW, it approaches directly.
        """
        base_angle_deg = self._calculate_base_angle()
        front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)
        
        if self.direction == 'ccw':
            # --- Standard single-step approach for CCW ---
            target_dist_min = self.parking_approach_ccw_dist_min_m
            target_dist_max = self.parking_approach_ccw_dist_max_m
            
            self.get_logger().debug(
                f"PRE_PARKING_ADJUST (CCW): FrontDist: {front_dist:.3f}m, TargetRange: {target_dist_min:.2f}-{target_dist_max:.2f}m",
                throttle_duration_sec=0.5
            )

            if not math.isnan(front_dist) and target_dist_min < front_dist < target_dist_max:
                self.get_logger().info("PRE_PARKING_ADJUST: Target position reached.")
                # (Same completion logic as before)
                self.publish_twist_with_gain(0.0, 0.0)
                self.parking_base_yaw_deg = self.current_yaw_deg
                self.get_logger().info(f"Storing parking base yaw: {self.parking_base_yaw_deg:.2f} deg")
                self.parking_sub_state = ParkingSubState.REVERSE_INTO_SPACE
                return
            
            # Driving logic is the same wall-following
            is_outer_wall = True
            original_speed = self.forward_speed
            parking_approach_speed = self.forward_speed * 0.8
            self._execute_pid_alignment(msg, base_angle_deg, is_outer_wall=is_outer_wall, 
                                        speed=parking_approach_speed)
            self.forward_speed = original_speed
            
        else: # cw
            # --- Special two-step approach for CW ---
            overshoot_target_dist = self.parking_approach_cw_overshoot_dist_m
            final_target_dist_min = self.parking_approach_cw_final_dist_min_m
            final_target_dist_max = self.parking_approach_cw_final_dist_max_m

            if self.pre_parking_step == 0:
                # --- Step 0: Overshoot Forward ---
                self.get_logger().debug(
                    f"PRE_PARKING_ADJUST (CW-Forward): FrontDist: {front_dist:.3f}m, Overshoot Target: < {overshoot_target_dist:.2f}m",
                    throttle_duration_sec=0.2
                )
                if not math.isnan(front_dist) and front_dist < overshoot_target_dist:
                    self.get_logger().info("Overshoot position reached. Starting reverse adjustment.")
                    self.publish_twist_with_gain(0.0, 0.0)
                    self.pre_parking_step = 1 # Move to the next step
                    return
                
                # Drive forward along the wall
                is_outer_wall = True
                original_speed = self.forward_speed
                parking_overshoot_speed = self.forward_speed * 1.2
                self._execute_pid_alignment(msg, base_angle_deg, is_outer_wall=is_outer_wall,
                                            speed=parking_overshoot_speed)
                self.forward_speed = original_speed

            elif self.pre_parking_step == 1:
                # --- Step 1: Reverse to Final Position ---
                self.get_logger().debug(
                    f"PRE_PARKING_ADJUST (CW-Reverse): FrontDist: {front_dist:.3f}m, Final Target: {final_target_dist_min:.2f}-{final_target_dist_max:.2f}m",
                    throttle_duration_sec=0.2
                )
                if not math.isnan(front_dist) and final_target_dist_min < front_dist < final_target_dist_max:
                    self.get_logger().info("PRE_PARKING_ADJUST: Final target position reached from reverse.")
                    # (Same completion logic as before)
                    self.publish_twist_with_gain(0.0, 0.0)
                    self.parking_base_yaw_deg = self.current_yaw_deg
                    self.get_logger().info(f"Storing parking base yaw: {self.parking_base_yaw_deg:.2f} deg")
                    self.parking_sub_state = ParkingSubState.REVERSE_INTO_SPACE
                    return
                    
                # Reverse straight back (using PID to maintain heading)
                angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
                angle_steer = self.align_kp_angle * angle_error_deg
                final_steer = max(min(angle_steer, self.max_steer), -self.max_steer)
                self.publish_twist_with_gain(-self.forward_speed * 1.2, final_steer)

    def _handle_parking_sub_reverse_into_space(self, msg: LaserScan):
        """
        Sub-state: Reverse into the parking space at a sharp angle.
        """
        # --- Define targets ---
        target_relative_angle_deg = self.parking_reverse_in_target_angle_deg
        yaw_tolerance_deg = self.parking_reverse_in_yaw_tolerance_deg

        if self.direction == 'ccw':
            target_yaw_deg = self._angle_normalize(self.parking_base_yaw_deg + target_relative_angle_deg)
            steer = self.max_steer # Turn left
        else: # cw
            target_yaw_deg = self._angle_normalize(self.parking_base_yaw_deg - target_relative_angle_deg)
            steer = -self.max_steer # Turn right

        # --- Check for completion ---
        yaw_error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)

        self.get_logger().debug(
            f"REVERSE_INTO_SPACE: TargetYaw: {target_yaw_deg:.2f}, CurrentYaw: {self.current_yaw_deg:.2f}, Error: {yaw_error_deg:.2f}",
            throttle_duration_sec=0.2
        )

        if abs(yaw_error_deg) <= yaw_tolerance_deg:
            self.get_logger().info("REVERSE_INTO_SPACE complete. Target angle reached.")
            self.publish_twist_with_gain(0.0, 0.0)

            # self.parking_sub_state = ParkingSubState.FORWARD_ADJUST_1
            self.state = State.FINISHED
            return

        # --- Driving logic: Reverse with max steering ---
        reverse_speed = -self.forward_speed * 1.2
        self.publish_twist_with_gain(reverse_speed, steer)

    # --- Timer Callbacks ---
    def _preparation_complete_callback(self):
        """
        Called by a timer after the camera movement is complete.
        Transitions the preparation sub-state to DETERMINE_DIRECTION.
        """
        with self.state_lock:
            if self.camera_wait_timer:
                self.camera_wait_timer.destroy()
                self.camera_wait_timer = None

            if self.preparation_sub_state == PreparationSubState.INITIALIZING_CAMERA:
                self.get_logger().info("PREPARATION: Camera initialization complete.")
                self.get_logger().info("--- Transitioning to DETERMINE_DIRECTION sub-state ---")
                self.preparation_sub_state = PreparationSubState.DETERMINE_DIRECTION
                self.camera_init_sent = False

    def _process_pre_unparking_image_callback(self):
        """
        Callback for pre-unparking detection. Processes the image, logs the result,
        and transitions to the next sub-state (INITIAL_TURN).
        Includes a retry mechanism for image acquisition.
        """
        with self.state_lock:
            # Clean up the primary timer if it exists.
            if self.pre_detection_timer:
                self.pre_detection_timer.destroy()
                self.pre_detection_timer = None

            if self.unparking_sub_state != UnparkingSubState.PRE_UNPARKING_DETECTION:
                return
            
            if self.pre_detection_step != 1:
                return

            # --- ADDED: Frame acquisition check with retry logic ---
            max_retries = 10 # Try for 0.5 seconds (10 * 50ms)
            if self.latest_frame is None:
                if self.image_acquisition_retries < max_retries:
                    self.image_acquisition_retries += 1
                    self.get_logger().warn(
                        f"Frame not available yet. Retrying in 50ms... ({self.image_acquisition_retries}/{max_retries})"
                    )
                    # Create a short timer to try again
                    self.pre_detection_timer = self.create_timer(
                        0.05, # 50ms
                        self._process_pre_unparking_image_callback
                    )
                    return # Exit the function and wait for the retry timer
                else:
                    self.get_logger().error(
                        "Failed to acquire image after multiple retries. Aborting unparking."
                    )
                    self.state = State.FINISHED
                    return
            # --- END OF ADDED SECTION ---

            self.get_logger().info("PRE-UNPARKING DETECT (Step 1): Image acquired. Processing image...")
            
            # Reset retry counter for the next time
            self.image_acquisition_retries = 0
            
            dominant_color, max_red_area, max_green_area = self._find_and_save_dominant_blob(
                frame_rgb=self.latest_frame,
                turn_count=0,
                base_name="pre_unparking_detection"
            )
            
            close_obstacle_threshold = 3500.0
            # Check if EITHER red or green blob is larger than the threshold
            largest_blob_size = max(max_red_area, max_green_area)

            if largest_blob_size >= close_obstacle_threshold:
                self.has_obstacle_at_parking_exit = True
                self.get_logger().error( # Use ERROR level for high importance
                    f"!!! SPECIAL AVOIDANCE REQUIRED !!! Obstacle is very close (Max Blob: {largest_blob_size:.0f})."
                )
            else:
                self.has_obstacle_at_parking_exit = False
                self.get_logger().info(
                    f"Standard unparking procedure. Obstacle is at a safe distance (Max Blob: {largest_blob_size:.0f})."
                )

            # --- ADDED: Determine Unparking Strategy based on the 3 conditions ---
            strategy = UnparkingStrategy.UNDEFINED # Default to undefined

            # --- Pattern 1 Logic ---
            is_pattern1_case1 = self.direction == 'cw' and dominant_color == 'green'
            is_pattern1_case2 = self.direction == 'ccw' and dominant_color == 'red'
            is_pattern1_case3 = self.direction == 'ccw' and dominant_color == 'green' and not self.has_obstacle_at_parking_exit
            if is_pattern1_case1 or is_pattern1_case2 or is_pattern1_case3:
                strategy = UnparkingStrategy.STANDARD_EXIT_TO_OUTER_LANE

            # --- Pattern 2 Logic ---
            is_pattern2 = self.direction == 'cw' and dominant_color == 'red' and not self.has_obstacle_at_parking_exit
            if is_pattern2:
                strategy = UnparkingStrategy.STANDARD_EXIT_TO_INNER_LANE

            # --- Pattern 3 Logic ---
            is_pattern3 = self.direction == 'cw' and dominant_color == 'red' and self.has_obstacle_at_parking_exit
            if is_pattern3:
                strategy = UnparkingStrategy.AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CW

            # --- Pattern 4 Logic ---
            is_pattern4 = self.direction == 'ccw' and dominant_color == 'green' and self.has_obstacle_at_parking_exit
            if is_pattern4:
                strategy = UnparkingStrategy.AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CCW

            # Store the decided strategy in the class variable for later use
            self.unparking_strategy = strategy
            # --- END OF ADDED SECTION ---

            log_message = (
                f"--- PRE-UNPARKING DETECTION RESULT ---\n"
                f"      Direction: {self.direction.upper()}\n"
                f"      Dominant Color Detected: '{dominant_color}'\n"
                f"      Has Obstacle at Exit: {self.has_obstacle_at_parking_exit}\n"
                f"      >> Decided Strategy: {self.unparking_strategy.name} <<\n" # Log the decided strategy
                f"----------------------------------------"
            )
            self.get_logger().warn(log_message)

            # Reset step counter for next time this state is entered (good practice)
            self.pre_detection_step = 0

            # --- Transition to the next step of the unparking sequence ---
            self.get_logger().info("PRE-UNPARKING DETECT: Detection complete. Transitioning to INITIAL_TURN.")
            self.unparking_sub_state = UnparkingSubState.INITIAL_TURN

    def _camera_initialization_complete_callback(self):
        """
        Called by a timer after the camera has had time to move.
        Transitions the state machine to the color detection step and destroys the timer.
        """
        with self.state_lock:
            if self.camera_wait_timer:
                self.camera_wait_timer.destroy()
                self.camera_wait_timer = None

            if self.determine_course_sub_state == DetermineCourseSubState.INITIALIZING_CAMERA:
                self.get_logger().info("Camera initialization complete. Transitioning to DETECTING_OBSTACLE_COLOR.")
                # MODIFIED: Transition to the correct new state
                self.determine_course_sub_state = DetermineCourseSubState.DETECTING_OBSTACLE_COLOR

    # --- Core Driving/Action Functions ---
    def publish_twist_with_gain(self, linear_x, angular_z):
        """
        Calculates the final velocity command, applies gain, rate limiting, and publishes the Twist message.
        """
        # --- Throttling Logic (no changes here) ---
        current_time = self.get_clock().now()
        duration_since_last_pub = (current_time - self.last_cmd_pub_time).nanoseconds / 1e6

        if duration_since_last_pub < self.cmd_pub_interval_ms:
            return
        
        self.last_cmd_pub_time = current_time

        # --- Gain Application ---
        state_specific_gain = self._get_state_specific_gain()
        target_linear = linear_x * self.gain * state_specific_gain
        target_angular = angular_z * self.gain * state_specific_gain
        
        # --- NEW: Rate Limiter Logic ---
        # Calculate time delta (dt) based on the control loop rate (50Hz)
        dt = 1.0 / 50.0

        # Limit the change in linear velocity
        max_delta_linear = self.max_linear_acceleration * dt
        final_linear_x = np.clip(
            target_linear,
            self.last_published_linear - max_delta_linear,
            self.last_published_linear + max_delta_linear
        )

        # Limit the change in angular velocity
        max_delta_angular = self.max_angular_acceleration_rad * dt
        final_angular_z = np.clip(
            target_angular,
            self.last_published_angular - max_delta_angular,
            self.last_published_angular + max_delta_angular
        )
        # --- END OF NEW ---

        # --- Publish and Store Last Values ---
        # --- NEW: Publish only if the command has changed ---
        # Define a small tolerance to avoid floating point issues
        tolerance = 1e-4 
        is_linear_changed = abs(final_linear_x - self.last_published_linear) > tolerance
        is_angular_changed = abs(final_angular_z - self.last_published_angular) > tolerance
        
        # Publish if there's a change OR if the robot is supposed to stop (as a safety measure)
        is_stopping = abs(final_linear_x) < tolerance and abs(final_angular_z) < tolerance
        was_moving = abs(self.last_published_linear) > tolerance or abs(self.last_published_angular) > tolerance

        if is_linear_changed or is_angular_changed or (is_stopping and was_moving):
            twist_msg = Twist()
            twist_msg.linear.x = final_linear_x
            twist_msg.angular.z = final_angular_z
            self.publisher_.publish(twist_msg)
            
            self.last_published_linear = final_linear_x
            self.last_published_angular = final_angular_z

    def _execute_pid_alignment(self, msg: LaserScan, base_angle_deg: float, is_outer_wall: bool,
                            speed: float, override_target_dist: float = None,
                            disable_dist_control: bool = False):
        """
        A generic PID controller for aligning the robot parallel to a specified wall.
        Can now handle forward/backward movement and an overridden target distance.
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

        # --- Target Distance Calculation ---
        if override_target_dist is not None:
            target_dist = override_target_dist
            log_prefix += "_OVERRIDE"
        elif is_outer_wall:
            target_dist = self.align_target_outer_dist_start_area_m if self.wall_segment_index == 0 else self.align_target_outer_dist_m
        else: # Inner wall
            target_dist = self.align_target_inner_dist_m

        wall_angle = self._angle_normalize(base_angle_deg + wall_offset_deg)
        wall_dist = self.get_distance_at_world_angle(msg, wall_angle)
        
        angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg

        dist_steer = 0.0
        dist_error = 0.0
        log_mode = "NORMAL"

        if not math.isnan(wall_dist):
            dist_error = target_dist - wall_dist
        
        # --- NEW: Special logic for inner wall avoidance alignment ---
        if not is_outer_wall and self.is_in_avoidance_alignment and (math.isnan(wall_dist) or wall_dist > 1.0):
            # If the inner wall is lost during avoidance, try to estimate from the outer wall.
            outer_wall_angle = self._angle_normalize(base_angle_deg - wall_offset_deg) # Opposite side
            outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
            
            if not math.isnan(outer_wall_dist):
                effective_inner_dist = 1.0 - outer_wall_dist
                dist_error = target_dist - effective_inner_dist
                log_mode = f"ESTIMATED(O:{outer_wall_dist:.2f})"
            else:
                dist_error = 0.0 # No walls visible, rely on IMU only for heading.
                log_mode = "IMU_ONLY"
        
        
        # If the corner detection counter has started, prioritize heading control over distance control
        # to prevent sudden steering when the inner wall disappears.
        if not disable_dist_control:
            if self.inner_wall_far_counter > 0:
                dist_steer = 0.0
                log_mode += "_CORNER_APPROACH"
                self.get_logger().debug("Corner imminent, suppressing distance steering.", throttle_duration_sec=1.0)
            elif abs(dist_error) > self.align_dist_tolerance_m:
                dist_steer = self.align_kp_dist * dist_error * dist_steer_multiplier
        else:
            dist_steer = 0.0
            log_mode += "_IMU_ONLY"
        
        angular_z = angle_steer + dist_steer
        final_steer = max(min(angular_z, self.max_steer), -self.max_steer)
        
        self.get_logger().debug(
            f"{log_prefix}({log_mode}) | WallD: {wall_dist:.2f} | "
            f"YawErr: {angle_error_deg:.1f} | DistSteer: {dist_steer:.2f} | "
            f"FinalSteer: {final_steer:.2f}",
            throttle_duration_sec=0.2
        )

        self.get_logger().debug(
            f"{log_prefix}({log_mode}) | WallD:{wall_dist:.2f} TgtD:{target_dist:.2f} | " 
            f"YawErr:{angle_error_deg:.1f} | DistSteer:{dist_steer:.2f} | "
            f"FinalSteer:{final_steer:.2f}",
            throttle_duration_sec=0.2
        )

        self.publish_twist_with_gain(speed, final_steer)
        return final_steer

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

    def _update_dynamic_tilt(self, target_world_angle_deg: float):
        """
        Calculates and sends the tilt position to keep the camera aimed at a
        specific absolute world angle, throttled to a specific update rate.

        Args:
            target_world_angle_deg: The absolute angle in the world frame that the camera should point to.
        """
        # --- Throttle the update rate ---
        current_time = self.get_clock().now()
        duration_since_last_update = (current_time - self.last_tilt_update_time).nanoseconds / 1e6

        if duration_since_last_update < self.tilt_update_interval_ms:
            return

        self.last_tilt_update_time = current_time

        # --- Calculation Logic (integrated from the old _calculate function) ---

        # Step 1: Calculate the angle the camera needs to point, relative to the CURRENT robot body.
        camera_relative_angle_deg = self._angle_diff(target_world_angle_deg, self.current_yaw_deg)
        
        # Step 2: Convert the relative camera angle to a servo command value (offset from center).
        tilt_offset = camera_relative_angle_deg * self.servo_units_per_degree
        
        # Adjust the offset based on servo orientation and driving direction.
        # This assumes a positive offset means turning the camera left.
        # This might need to be inverted depending on the physical servo setup.
        target_tilt = self.tilt_center_position + tilt_offset
        
        # Step 3: Clamp the final value to be within the servo's physical limits.
        clamped_tilt = int(max(self.tilt_min_position, min(self.tilt_max_position, target_tilt)))
        
        # --- Servo Command Publishing ---
        
        # Only send a new command if the target position has changed.
        if clamped_tilt != self.last_sent_tilt_position:
            self.get_logger().debug(f"Updating dynamic tilt. Target World: {target_world_angle_deg:.1f}, Servo Pos: {clamped_tilt}", throttle_duration_sec=1.0)
            
            msg = SetPWMServoState()
            msg.duration = 0.1 # Short duration for smooth, continuous updates
            
            tilt_state = PWMServoState()
            tilt_state.id = [self.tilt_servo_id]
            tilt_state.position = [clamped_tilt]
            msg.state = [tilt_state]

            self.servo_pub.publish(msg)
            self.last_sent_tilt_position = clamped_tilt


    # --- Vision & Image Processing Helpers ---
    # --- Primary Vision Logic ---
    def _find_and_save_dominant_blob(self, frame_rgb, turn_count, base_name):
        """
        Processes a full image frame to find the dominant color blob (red or green),
        saves debug images, and returns the dominant color.

        Returns:
            A tuple (string, float, float):
            - Dominant color ('red', 'green', or 'none')
            - Max red blob area found
            - Max green blob area found
        """
        if frame_rgb is None:
            return 'none', 0.0, 0.0

        # --- 1. Process the image to get color masks ---
        detection_data = self._detect_obstacle_color_in_frame(frame_rgb)
        if not detection_data:
            return 'none', 0.0, 0.0

        # --- 2. Find the largest blob for each color ---
        red_mask = detection_data['masks']['RED']
        green_mask = detection_data['masks']['GREEN']
        max_red_area = self._find_largest_blob_area(red_mask)
        max_green_area = self._find_largest_blob_area(green_mask)

        # --- 3. Save annotated debug images ---
        if self.save_debug_images:
            # Since this is a full-frame detection, we don't need to draw an ROI.
            # We pass `rois=None`.
            self._save_annotated_image(
                base_name=base_name,
                turn_count=turn_count,
                frame_bgr=detection_data['frame_bgr'],
                masks={'RED': red_mask, 'GREEN': green_mask},
                rois=None, # No specific ROI for this full-frame detection
                sample_num=1
            )
            self.get_logger().info(f"Saved debug image for '{base_name}'")

        # --- 4. Determine the dominant color ---
        detection_threshold = 500  # This could be a parameter
        is_red_dominant = max_red_area > max_green_area and max_red_area > detection_threshold
        is_green_dominant = max_green_area > max_red_area and max_green_area > detection_threshold

        dominant_color = 'none'
        if is_red_dominant:
            dominant_color = 'red'
        elif is_green_dominant:
            dominant_color = 'green'
        
        return dominant_color, max_red_area, max_green_area

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
        
        # --- MODIFIED: Always calculate full_frame area ---
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

    def _find_largest_blob_area(self, mask):
        """
        Finds all contiguous blobs in a binary mask and returns the area of the largest one.
        Uses connectedComponentsWithStats for efficient blob analysis.
        """
        # Find connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)

        if num_labels <= 1: # Only background found
            return 0

        # The first label (0) is the background, so we ignore it by slicing from 1.
        areas = stats[1:, cv2.CC_STAT_AREA]
        
        if areas.size == 0:
            return 0
            
        return np.max(areas)

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

    def _save_annotated_image(self, base_name: str, turn_count: int, frame_bgr, masks, rois, sample_num: int = 1):
        """
        Saves annotated debug images for color detection, organizing planning
        images into corner-specific subfolders.
        """
        if not self.save_debug_images:
            return

        try:
            # --- Determine the save path dynamically ---
            save_path = self.debug_image_path
            # For planning images, create and use a corner-specific subfolder
            if base_name == 'planning_detection' and turn_count > 0:
                corner_folder_name = f"corner{turn_count}"
                save_path = os.path.join(self.debug_image_path, corner_folder_name)
                # Create the directory if it doesn't exist; exist_ok=True prevents errors
                os.makedirs(save_path, exist_ok=True)

            # Create a unique filename suffix
            filename_suffix = f"turn{turn_count}_sample{sample_num}"
            
            # Annotated ROI Image
            annotated_frame = frame_bgr.copy()
            if rois:
                for roi_name, roi_points in rois.items():
                    if roi_points: # Check if points list is not empty
                        cv2.polylines(annotated_frame, [np.array(roi_points, dtype=np.int32)], True, (255, 255, 0), 2)
            
            filename_annotated = os.path.join(save_path, f"{base_name}_{filename_suffix}_annotated.jpg")
            cv2.imwrite(filename_annotated, annotated_frame)

            # Combined Color Mask Image
            red_mask = masks.get('RED', np.zeros(frame_bgr.shape[:2], dtype=np.uint8))
            green_mask = masks.get('GREEN', np.zeros(frame_bgr.shape[:2], dtype=np.uint8))
            
            combined_mask_viz = np.zeros_like(frame_bgr)
            combined_mask_viz[np.where(green_mask == 255)] = (0, 255, 0) # BGR for Green
            combined_mask_viz[np.where(red_mask == 255)] = (0, 0, 255) # BGR for Red

            if rois:
                for roi_name, roi_points in rois.items():
                    cv2.polylines(combined_mask_viz, [np.array(roi_points, dtype=np.int32)], True, (255, 255, 0), 2)

            filename_mask = os.path.join(save_path, f"{base_name}_{filename_suffix}_masks.jpg")
            cv2.imwrite(filename_mask, combined_mask_viz)

            # Log only the first saved image to avoid flooding the console
            if sample_num == 1:
                self.get_logger().info(f"Saved debug images for {base_name} turn {turn_count} to: {save_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to save debug image: {e}")

    # --- Planning-Specific Vision Logic ---
    def _get_obstacle_pattern_from_areas(self, red_area, green_area, threshold):
        """
        Classifies the obstacle pattern based on the detected red and green pixel areas
        from a single frame.
        """
        is_red_present = red_area > threshold
        is_green_present = green_area > threshold

        if is_red_present and is_green_present:
            # If both are present, assume the one with the larger area is the primary
            return 'red_to_green' if red_area >= green_area else 'green_to_red'
        elif is_red_present:
            return 'red_only'
        elif is_green_present:
            return 'green_only'
        else:
            return 'none'

    def _scan_and_collect_data(self):
        """
        Performs a single instance of image scanning. It finds the largest
        contiguous blob of each color within a defined polygon ROI and updates
        the maximum area found so far.
        """

        # Increment frame counter first
        self.scan_frame_count += 1
        # Check if this frame should be processed
        if self.scan_frame_count % self.planning_scan_interval != 0:
            return # Skip processing for this frame

        # If we process this frame, use the total count as the sample number
        current_sample_num = self.scan_frame_count // self.planning_scan_interval

        if self.latest_frame is None:
            self.get_logger().warn("In scanning range but latest_frame is None.", throttle_duration_sec=1.0)
            return

        # 1. Get color masks
        detection_data = self._detect_obstacle_color_in_frame(self.latest_frame)
        if not detection_data:
            return
            
        red_mask = detection_data['masks']['RED']
        green_mask = detection_data['masks']['GREEN']

        # --- MODIFIED: Create a polygon ROI mask from the 4-point parameter ---
        # 2. Reshape the flat points and create the scanning ROI mask
        scan_roi_points = self._reshape_roi_points(self.planning_scan_roi_flat)
        scan_roi_mask = np.zeros(red_mask.shape[:2], dtype=np.uint8)
        
        if scan_roi_points: # Proceed only if the points are valid
            roi_corners = np.array([scan_roi_points], dtype=np.int32)
            cv2.fillPoly(scan_roi_mask, roi_corners, 255)
        
        # 3. Apply the ROI to the color masks
        red_mask_roi = cv2.bitwise_and(red_mask, scan_roi_mask)
        green_mask_roi = cv2.bitwise_and(green_mask, scan_roi_mask)

        # 4. Find the largest blob for each color
        max_red_area_in_frame = self._find_largest_blob_area(red_mask_roi)
        max_green_area_in_frame = self._find_largest_blob_area(green_mask_roi)

        # 5. Update the overall maximums and the sample number when it occurred
        if max_red_area_in_frame > self.max_red_blob_area:
            self.max_red_blob_area = max_red_area_in_frame
            self.max_red_blob_sample_num = current_sample_num # RECORD SAMPLE NUMBER
        
        if max_green_area_in_frame > self.max_green_blob_area:
            self.max_green_blob_area = max_green_area_in_frame
            self.max_green_blob_sample_num = current_sample_num # RECORD SAMPLE NUMBER

        # Debug logging
        self.get_logger().debug(
            f"Scan Frame: RedBlob={max_red_area_in_frame:.0f}, GreenBlob={max_green_area_in_frame:.0f} | "
            f"Max Encountered: R={self.max_red_blob_area:.0f}, G={self.max_green_blob_area:.0f}"
        )
        
        # Save debug images
        if self.save_debug_images:
            # --- MODIFIED: Use a single dictionary for ROIs to pass to the save function ---
            rois_to_visualize = {'scan_roi': scan_roi_points} if scan_roi_points else None

            self.detection_results.append(1) # Frame counter
            sample_num = len(self.detection_results)

            self._save_annotated_image(
                base_name="planning_detection",
                turn_count=self.turn_count + 1,
                frame_bgr=detection_data['frame_bgr'],
                masks={'RED': red_mask_roi, 'GREEN': green_mask_roi},
                rois=rois_to_visualize,
                sample_num=sample_num
            )

    # --- ROI Manipulation Helpers ---
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


    # --- General Helpers & Calculation Functions ---
    # --- State Logic Helpers ---
    def _check_for_corner(self, msg: LaserScan, base_angle_deg: float) -> tuple[bool, float]:
        """
        Checks for a corner. If the path plan is not yet complete, it transitions 
        to the planning state. Otherwise, it prepares for a normal turn.
        """
        if self.direction == 'ccw':
            inner_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)

        else: # cw
            inner_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)

        
        front_wall_dist = self.get_distance_at_world_angle(msg, base_angle_deg)
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
        outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)

        self.get_logger().debug(
            f"[CornerCheck] Turn:{self.turn_count} | "
            f"F_Dist:{front_wall_dist:.2f} (<1.2?) | "
            f"I_Dist:{inner_wall_dist:.2f} (>{self.inner_wall_disappear_threshold:.1f}?) | "
            f"Counter:{self.inner_wall_far_counter}/{self.inner_wall_disappear_count} | "
            f"CanTurn:{self.can_start_new_turn}",
            throttle_duration_sec=0.5
        )

        if self.turn_count > self.max_turns:
            # If we detect a corner but have already completed the required number of laps,
            # it means the primary finish condition was missed. Start parking immediately.
            self.get_logger().warn(
                f"BACKUP TRIGGER: Corner detected after max_turns ({self.turn_count}) reached. "
                "Forcing transition to PARKING state."
            )
            self.state = State.PARKING
            self.parking_sub_state = ParkingSubState.PREPARE_PARKING
            self.pre_parking_step = 0
            self.publish_twist_with_gain(0.0, 0.0)
            # Return True to signal that a state change has occurred and the caller should stop.
            return True, inner_wall_dist 

        # --- MODIFIED CORNER DETECTION LOGIC ---
        if not math.isnan(inner_wall_dist) and not math.isnan(outer_wall_dist) and not math.isnan(front_wall_dist):
            inner_wall_is_far = inner_wall_dist >= self.inner_wall_disappear_threshold
            front_is_close = front_wall_dist < 1.2

            if inner_wall_is_far and front_is_close:
                self.inner_wall_far_counter += 1
            else:
                # Reset counter only if the wall is confirmed to be close
                self.inner_wall_far_counter = 0
        # If inner_wall_dist is nan, do nothing and keep the counter's value.
        # This makes the detection robust against sporadic measurement failures.


        if self.inner_wall_far_counter >= self.inner_wall_disappear_count and self.can_start_new_turn:
            self.approach_base_yaw_deg = base_angle_deg
            is_planning_complete = '' not in self.avoidance_path_plan
            
            if not is_planning_complete:
                # Planning phase logic remains unchanged
                self.get_logger().info("Corner detected. Reversing to prepare for planning scan.")
                self.state = State.STRAIGHT
                self.straight_sub_state = StraightSubState.PRE_SCANNING_REVERSE
                self.pre_scanning_reverse_target_dist_m = 0.7 
            else:
                # Planning is complete, so start the new simplified turning sequence.
                # The sequence always starts with POSITIONING_REVERSE.
                self.get_logger().info("Corner detected. Starting simplified turning sequence.")
                self._prepare_for_turning(base_angle_deg) # This now just sets up variables
                self.state = State.TURNING
                self.turning_sub_state = TurningSubState.POSITIONING_REVERSE
            
            self.publish_twist_with_gain(0.0, 0.0)
            return True, inner_wall_dist
            
        elif self.inner_wall_far_counter >= self.inner_wall_disappear_count and not self.can_start_new_turn:
            self.get_logger().debug("Corner condition met, but waiting for turn permission.", throttle_duration_sec=1.0)
        
        return False, inner_wall_dist

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
            
            if (not math.isnan(front_dist) and front_dist < 1.75 and front_dist > 1.0 and
                not math.isnan(inner_wall_dist) and inner_wall_dist < 1.0 and 
                not math.isnan(sum_side_walls_dist) and sum_side_walls_dist < 1.1 and sum_side_walls_dist > 0.4 and
                is_oriented_correctly # Using the result of the new function
                ):
                self.get_logger().warn(
                    f"FINISH CONDITION MET: Turn count ({self.turn_count}) >= max_turns ({self.max_turns}) "
                    f"AND FrontDist ({front_dist:.2f}m) is within range "
                    f"AND InnerDist ({inner_wall_dist:.2f}m) < 1.0m."
                )
                self.get_logger().warn("FINISH CONDITION MET. TRANSITIONING TO PARKING STATE.")
                self.state = State.PARKING
                self.parking_sub_state = ParkingSubState.PREPARE_PARKING
                self.pre_parking_step = 0
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

    def _has_passed_obstacle(self, msg: LaserScan, base_angle_deg: float, is_checking_from_outer_wall: bool) -> bool:
        """
        Checks if the robot has finished passing an obstacle. This is a pure check function.

        Args:
            is_checking_from_outer_wall: True if we are on the outer wall, checking the inner side.

        Returns:
            True if the obstacle has been passed, False otherwise.
        """
        # Determine which wall to measure based on the current driving lane
        if is_checking_from_outer_wall:
            # We are on the OUTER wall, check the distance to the INNER wall.
            check_wall_offset_deg = 90.0 if self.direction == 'ccw' else -90.0
        else: # Checking from inner wall
            # We are on the INNER wall, check the distance to the OUTER wall.
            check_wall_offset_deg = -90.0 if self.direction == 'ccw' else 90.0
        
        check_wall_angle = self._angle_normalize(base_angle_deg + check_wall_offset_deg)
        check_wall_dist = self.get_distance_at_world_angle(msg, check_wall_angle)

        pass_thresh_m = self.avoid_inner_pass_thresh_m
        # Apply special threshold for the start area
        if self.wall_segment_index == 0 and is_checking_from_outer_wall:
            pass_thresh_m -= 0.2

        # --- Passing Detection Logic ---
        # First, check if we have started passing the obstacle yet.
        if not self.is_passing_obstacle:
            current_wall_dist = self.get_distance_at_world_angle(msg, self._angle_normalize(check_wall_angle + 180.0))
            is_oriented_correctly = self._check_yaw_alignment(base_angle_deg, 45.0)
            sum_side_dist = (check_wall_dist or 0) + (current_wall_dist or 0)
            
            if is_oriented_correctly and not math.isnan(sum_side_dist) and sum_side_dist < 0.7:
                self.get_logger().warn(">>> Passing obstacle now (during alignment)...")
                self.is_passing_obstacle = True
            return False # We have either just started passing or not started yet.

        # --- Completion Check Logic ---
        # If we are already in the "passing" state, check if we're done.
        if not math.isnan(check_wall_dist) and check_wall_dist > pass_thresh_m:
            return True # We are clear of the obstacle.
        
        return False # Still passing.

    def _is_safe_for_lane_change(self, msg: LaserScan, base_angle_deg: float) -> bool:
        """
        Checks if the robot is in a stable, straight section of the course,
        making it safe to initiate a lane change.

        This is determined by checking front and side wall distances and using a
        counter to ensure stability over time.

        Returns:
            True if a lane change is safe, False otherwise.
        """
        # --- 1. Get distances to front, inner, and outer walls ---
        front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)

        if self.direction == 'ccw':
            inner_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
        else: # cw
            inner_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)

        inner_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
        outer_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)

        # --- 2. Check if the stability conditions are met ---
        is_stable = False
        # Ensure all distances are valid before checking conditions
        if not math.isnan(front_dist) and not math.isnan(inner_dist) and not math.isnan(outer_dist):
            side_walls_sum = inner_dist + outer_dist
            
            # Condition: Side walls are within a normal course width, and front is clear.
            if (0.9 < side_walls_sum < 1.1) and (front_dist < 1.9):
                is_stable = True
        
        # --- 3. Update the counter based on stability ---
        if is_stable:
            self.lane_change_stability_counter += 1
            self.get_logger().debug(
                f"Lane change stability counter: {self.lane_change_stability_counter}",
                throttle_duration_sec=0.5
            )
        else:
            # If conditions are not met, reset the counter immediately.
            pass

        # --- 4. Return True if the counter reaches the threshold ---
        stability_threshold = 15
        if self.lane_change_stability_counter >= stability_threshold:
            self.get_logger().info(
                f"Lane change stability confirmed (counter reached {self.lane_change_stability_counter})."
            )
            # Reset the counter for the next time
            self.lane_change_stability_counter = 0
            return True
        
        return False

    def _update_turn_permission_counter(self, msg: LaserScan, base_angle_deg: float):
        """
        Updates a counter based on stable wall detection to determine when it's safe
        to detect a new corner.
        """
        # If a turn is already permitted, no need to count.
        if self.can_start_new_turn:
            return

        # Define inner and outer wall angles
        if self.direction == 'ccw':
            inner_wall_angle = self._angle_normalize(base_angle_deg + 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
        else: # cw
            inner_wall_angle = self._angle_normalize(base_angle_deg - 90.0)
            outer_wall_angle = self._angle_normalize(base_angle_deg + 90.0)

        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
        outer_wall_dist = self.get_distance_at_world_angle(msg, outer_wall_angle)
        
        # Condition: Both walls are visible and the path is not too wide (i.e., not in a corner).
        if not math.isnan(inner_wall_dist) and not math.isnan(outer_wall_dist) and \
        (inner_wall_dist + outer_wall_dist) < 1.2:
            self.stable_alignment_counter += 1
        else:
            pass

        # If the counter reaches a threshold, permit the next turn.
        # The threshold (e.g., 50) means about 1 second of stable alignment at 50Hz.
        if self.stable_alignment_counter > 50: 
            self.can_start_new_turn = True
            self.stable_alignment_counter = 0 # Reset after enabling
            self.get_logger().warn("Stable alignment detected. New turn detection is ENABLED.")

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

        if not math.isnan(left_dist) and left_dist < 3.0 and left_dist > self.course_detection_threshold_m:
            # MODIFIED: Set the node's direction parameter immediately
            self.direction = "ccw"
            if not self.initial_path_is_left or self.initial_position_is_near:
                self.last_avoidance_path_was_outer = True
            else:
                self.last_avoidance_path_was_outer = False
            
            self.get_logger().warn(f"Left side is open (dist: {left_dist:.2f}m). Course is CCW.")
            return True, "ccw"
        
        if not math.isnan(right_dist) and right_dist < 3.0 and right_dist > self.course_detection_threshold_m:
            # MODIFIED: Set the node's direction parameter immediately
            self.direction = "cw"
            if self.initial_path_is_left or self.initial_position_is_near:
                self.last_avoidance_path_was_outer = True
            else:
                self.last_avoidance_path_was_outer = False
            
            self.get_logger().warn(f"Right side is open (dist: {right_dist:.2f}m). Course is CW.")
            return True, "cw"
            
        return False, ""

    def _update_avoidance_plan_based_on_vision(self, final_pattern: str):
        """
        Determines the next avoidance path based on the final decided pattern
        from the majority vote.
        """
        # Map obstacle configuration to path type based on driving direction
        if self.direction == 'ccw':
            path_mapping = {
                'red_only': 'outer',
                'green_only': 'inner',
                'red_to_green': 'outer_to_inner',
                'green_to_red': 'inner_to_outer',
                'none': 'outer' # Failsafe: if no obstacle is detected, take the wider outer path
            }
        else:  # cw
            path_mapping = {
                'red_only': 'inner',
                'green_only': 'outer',
                'red_to_green': 'inner_to_outer',
                'green_to_red': 'outer_to_inner',
                'none': 'outer' # Failsafe: if no obstacle is detected, take the wider outer path
            }
        
        next_avoidance_path = path_mapping.get(final_pattern, 'outer')
        next_segment_index = (self.wall_segment_index + 1) % len(self.avoidance_path_plan)
        
        # Update the path plan
        self.avoidance_path_plan[next_segment_index] = next_avoidance_path
        self.get_logger().info(f"Path plan for segment {next_segment_index} set to: '{next_avoidance_path}'")

        # --- This part correctly handles the "lane change" requirement ---
        if next_avoidance_path in ['outer_to_inner', 'inner_to_outer']:
            if not self.entrance_obstacle_plan[next_segment_index]:
                self.get_logger().warn(
                    f"Path is a lane change ('{next_avoidance_path}'). "
                    f"Forcing entrance_obstacle_plan to True for a safer turn."
                )
                self.entrance_obstacle_plan[next_segment_index] = True

    def _prepare_for_turning(self, base_angle_deg):
        """
        Calculates and sets up the necessary variables for a turn,
        but does not transition the state itself.
        """
        self.approach_base_yaw_deg = base_angle_deg
        
        # This logic is no longer needed as the turn is simpler

        self.get_logger().info("--- Preparing for simplified turn ---")
        self.get_logger().info(f"  Base Angle for Turn: {base_angle_deg:.2f} deg")
        self.get_logger().info("-------------------------------------")

        self.inner_wall_far_counter = 0
        self.can_start_new_turn = False
        self.stable_alignment_counter = 0
        self.state = State.TURNING
        self.turning_sub_state = TurningSubState.POSITIONING_REVERSE
        self.publish_twist_with_gain(0.0, 0.0)

    def _complete_avoidance_phase(self, next_sub_state: StraightSubState, path_was_outer: bool):
        """
        A generic helper function to finalize an avoidance phase (e.g., pass_through)
        and transition to the next state. It centralizes the resetting of state variables.

        Args:
            next_sub_state: The StraightSubState to transition into.
            path_was_outer: A boolean indicating if the completed path was on the outer side.
        """
        self.is_passing_obstacle = False
        self.last_avoidance_path_was_outer = path_was_outer
        self.straight_sub_state = next_sub_state
        
        self.publish_twist_with_gain(0.0, 0.0)

    def _get_turn_strategy(self):
        """
        Determines the appropriate turn distance, angle, and speeds based on the
        transition from the current lane to the next planned lane.
        Includes special handling for the final turn before parking.
        """
        next_segment_index = (self.wall_segment_index + 1) % len(self.avoidance_path_plan)
        is_entering_final_segment = next_segment_index == self.max_turns % len(self.avoidance_path_plan) # Assuming parking segment is 

        # --- 1. Determine the nature of the next path (outer or inner) ---
        is_next_path_outer = False # Default to inner for safety
        if is_entering_final_segment and self.final_approach_lane_is_outer is not None:
            # --- Special Case: Final turn before parking ---
            # Use the pre-determined flag for the most reliable decision.
            self.get_logger().info("Turn strategy: Using pre-determined flag for final segment approach.", throttle_duration_sec=1.0)
            is_next_path_outer = self.final_approach_lane_is_outer
        else:
            # --- Standard Case: Regular turn during laps ---
            # Use the vision-based avoidance plan.
            next_path_type = self._get_path_type_for_segment(next_segment_index, default_path='outer')
            is_next_path_outer = next_path_type in ['outer', 'outer_to_inner']
        
        # --- 2. Check if the entrance to the next segment is obstructed ---
        is_next_entrance_obstructed = self.entrance_obstacle_plan[next_segment_index]

        # --- 3. Select strategy based on current lane, next lane, and entrance state ---
        if self.last_avoidance_path_was_outer:
            # =============================
            # CURRENT LANE: OUTER
            # =============================
            if is_next_path_outer:
                # --- Strategy: OUTER -> OUTER ---
                if is_next_entrance_obstructed:
                    # Case 1: Entrance might be obstructed
                    dist, angle, app_speed, turn_speed = (
                        self.turn_outer_to_outer_dist_m,
                        self.turn_outer_to_outer_angle_deg,
                        self.turn_outer_to_outer_approach_speed,
                        self.turn_outer_to_outer_turn_speed
                    )
                else:
                    # Case 2: Entrance is CLEAR
                    dist, angle, app_speed, turn_speed = (
                        self.turn_outer_to_outer_clear_dist_m,
                        self.turn_outer_to_outer_clear_angle_deg,
                        self.turn_outer_to_outer_clear_approach_speed,
                        self.turn_outer_to_outer_clear_turn_speed
                    )
            else:
                # --- Strategy: OUTER -> INNER ---
                if is_next_entrance_obstructed:
                    # Case 3: Entrance might be obstructed
                    dist, angle, app_speed, turn_speed = (
                        self.turn_outer_to_inner_dist_m,
                        self.turn_outer_to_inner_angle_deg,
                        self.turn_outer_to_inner_approach_speed,
                        self.turn_outer_to_inner_turn_speed
                    )
                else:
                    # Case 4: Entrance is CLEAR
                    dist, angle, app_speed, turn_speed = (
                        self.turn_outer_to_inner_clear_dist_m,
                        self.turn_outer_to_inner_clear_angle_deg,
                        self.turn_outer_to_inner_clear_approach_speed,
                        self.turn_outer_to_inner_clear_turn_speed
                    )
        else:
            # =============================
            # CURRENT LANE: INNER
            # =============================
            if is_next_path_outer:
                # --- Strategy: INNER -> OUTER ---
                if is_next_entrance_obstructed:
                    # Case 5: Entrance might be obstructed
                    dist, angle, app_speed, turn_speed = (
                        self.turn_inner_to_outer_dist_m,
                        self.turn_inner_to_outer_angle_deg,
                        self.turn_inner_to_outer_approach_speed,
                        self.turn_inner_to_outer_turn_speed
                    )
                else:
                    # Case 6: Entrance is CLEAR
                    dist, angle, app_speed, turn_speed = (
                        self.turn_inner_to_outer_clear_dist_m,
                        self.turn_inner_to_outer_clear_angle_deg,
                        self.turn_inner_to_outer_clear_approach_speed,
                        self.turn_inner_to_outer_clear_turn_speed
                    )
            else:
                # --- Strategy: INNER -> INNER ---
                if is_next_entrance_obstructed:
                    # Case 7: Entrance might be obstructed
                    dist, angle, app_speed, turn_speed = (
                        self.turn_inner_to_inner_dist_m,
                        self.turn_inner_to_inner_angle_deg,
                        self.turn_inner_to_inner_approach_speed,
                        self.turn_inner_to_inner_turn_speed
                    )
                else:
                    # Case 8: Entrance is CLEAR
                    dist, angle, app_speed, turn_speed = (
                        self.turn_inner_to_inner_clear_dist_m,
                        self.turn_inner_to_inner_clear_angle_deg,
                        self.turn_inner_to_inner_clear_approach_speed,
                        self.turn_inner_to_inner_clear_turn_speed
                    )
        
        # Apply special offset for the first corner approach
        if is_next_path_outer and next_segment_index == 0:
            dist += self.turn_start_area_dist_offset_m

        return dist, angle, app_speed, turn_speed

    # --- Sensor Data Helpers ---
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

    # --- Geometry & Math Helpers ---
    def _calculate_base_angle(self):
        """Calculates the ideal angle of the current wall segment based on direction."""
        if self.direction == 'ccw':
            return self.wall_segment_index * 90.0
        else: # cw
            return self.wall_segment_index * -90.0

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

    # --- Misc Helpers ---
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
            if self.straight_sub_state == StraightSubState.ALIGN_WITH_INNER_WALL:
                state_specific_gain = self.gain_straight_align_inner_wall
            elif  self.straight_sub_state == StraightSubState.ALIGN_WITH_OUTER_WALL:
                state_specific_gain = self.gain_straight_align_outer_wall
            elif self.straight_sub_state == StraightSubState.AVOID_OUTER_TURN_IN:
                state_specific_gain = self.gain_straight_outer_turn_in

        # Add more conditions here if needed in the future
        # elif self.state == State.STRAIGHT and self.straight_sub_state == ... :
        #     state_specific_gain = self.gain_some_other_state
        
        return state_specific_gain

    def _get_effective_detection_threshold(self, scan_msg: LaserScan):
        """Calculates the dynamic detection threshold based on the distance to the obstacle."""
        base_detection_distance_m = 1.2
        
        if self.last_avoidance_path_was_outer:
            course_width_m = 2.0
            if self.direction == 'ccw':
                outer_wall_angle = self._angle_normalize(self.approach_base_yaw_deg - 90.0)
            else: # cw
                outer_wall_angle = self._angle_normalize(self.approach_base_yaw_deg + 90.0)
            
            outer_wall_dist = self.get_distance_at_world_angle(scan_msg, outer_wall_angle)

            if not math.isnan(outer_wall_dist) and 0.1 < outer_wall_dist < course_width_m:
                effective_distance_outer = max(0.1, course_width_m - outer_wall_dist)
                distance_ratio_sq = (base_detection_distance_m / effective_distance_outer) ** 2
                threshold = self.planning_detection_threshold * distance_ratio_sq
                threshold = max(200, min(threshold, self.planning_detection_threshold * 5.0))
                self.get_logger().debug(f"Dynamic threshold calculated: {threshold:.1f}")
                return threshold
            else:
                self.get_logger().warn("Could not get valid outer wall distance. Using fallback multiplier.")
                return self.planning_detection_threshold * self.planning_detection_threshold_outer_path_multiplier
        else:
            self.get_logger().info(f"Last path was INNER. Using standard threshold: {self.planning_detection_threshold:.1f}")
            return self.planning_detection_threshold

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