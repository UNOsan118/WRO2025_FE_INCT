import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from enum import Enum, auto
from std_srvs.srv import Trigger
import threading

class State(Enum):
    PREPARATION = auto()
    STRAIGHT = auto()
    TURNING = auto()
    FINISHED = auto()

class PreparationSubState(Enum):
    WAITING_FOR_CONTROLLER = auto()
    DETERMINE_DIRECTION = auto()

class TurningSubState(Enum):
    POSITIONING_REVERSE = auto()
    APPROACH_CORNER = auto()
    EXECUTE_PIVOT_TURN = auto()
    FINALIZE_TURN = auto()

class OpenNavigatorNode(Node):
    # --- Initialization & Lifecycle ---
    def __init__(self):
        super().__init__('open_navigator_node')

        # 1. CORE NODE SETUP
        # =================
        self.start_time = self.get_clock().now()
        self.cmd_pub_interval_ms = 33

        self.declare_parameter('log_level_str', 'DEBUG')
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
        self.state = State.PREPARATION
        self.preparation_sub_state = PreparationSubState.WAITING_FOR_CONTROLLER
        self.turning_sub_state = None 
        
        # 3. TUNING PARAMETERS (Simplified for Open Course)
        # =================================================
        # --- General & Debug ---
        self.correct_mirrored_scan = True
        self.max_valid_range_m = 3.0
        self.max_turns = 4 # 12
        
        # --- Driving & Speed Control ---
        self.forward_speed = 0.2
        self.max_steer = 1.2
        self.declare_parameter('gain', 2.0)
        self.direction = None
        
        # --- Rate Limiter (Smoother) ---
        self.max_linear_acceleration = 3000.0
        self.max_angular_acceleration_rad = 7000.0

        # --- Alignment (PID) ---
        self.align_kp_angle = 0.02
        self.align_kp_dist = 7.5
        self.align_target_outer_dist_m = 0.2
        self.align_dist_tolerance_m = 0.005

        # --- Turning ---
        self.turn_approach_dist_m = 0.55
        self.turn_angle_deg = 40.0
        self.turn_approach_speed = 0.20
        self.turn_speed = 0.20
        self.turn_positioning_reverse_speed = -0.14 # Corrected to be negative
        
        # --- Corner Detection ---
        self.inner_wall_disappear_threshold = 1.3
        self.inner_wall_disappear_count = 3
        
        # --- Finish Condition ---
        self.finish_stability_threshold = 50

        # 4. INTERNAL STATE VARIABLES
        # ===========================
        self.current_yaw_deg = 0.0
        self.latest_scan_msg = None
        
        # Rate Limiter state
        self.last_published_linear = 0.0
        self.last_published_angular = 0.0
        self.last_cmd_pub_time = self.get_clock().now()

        # --- Stuck Detection and Recovery ---
        self.stuck_detector_enabled = True
        self.stuck_check_interval_sec = 0.25
        self.stuck_duration_threshold_sec = 0.5
        self.stuck_wall_collision_dist_m = 0.05
        self.recovery_speed_boost_increment = 0.5
        self.recovery_speed_boost_max = 3.0
        self.recovery_angular_reduction_start_level = 3
        self.recovery_angular_reduction_step = 0.25
        self.recovery_max_level = 7
        self.recovery_reverse_gain = -0.8
        self.recovery_final_reverse_dist_m = 0.20
        self.stuck_motion_yaw_threshold_deg = 0.5
        self.stuck_motion_dist_threshold_m = 0.02
        self.last_stuck_check_time = self.get_clock().now()
        self.motion_command_start_time = None
        self.last_yaw_at_check = self.current_yaw_deg
        self.last_front_dist_at_check = -1.0
        self.recovery_gain = 1.0
        self.recovery_angular_gain = 1.0
        self.stuck_level = 0
        self.is_in_final_recovery_reverse = False
        self.stuck_position_for_reverse = None
        self.final_reverse_start_dists = None

        # General state flags and counters
        self.turn_count = 0
        self.wall_segment_index = 0
        self.can_start_new_turn = True
        self.inner_wall_far_counter = 0
        self.stable_alignment_counter = 0
        self.finish_stability_counter = 0
        self.last_state = self.state 

        # State-specific variables
        self.approach_base_yaw_deg = 0.0
        self.direction_detection_patience_counter = 0

        # 5. CORE COMPONENTS
        # ==================
        self.state_lock = threading.RLock()

        # 6. SYSTEM INITIALIZATION CALLS
        # ==============================
        self._load_parameters()
        self._setup_ros_communications()
        self.controller_ready_client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        self.yaw_publisher_ready_client = self.create_client(Trigger, '/yaw_publisher_node/init_finish')
        
        # 7. MAIN LOOP TIMER
        # ==================
        control_loop_rate = 50.0
        self.control_loop_timer = self.create_timer(
            1.0 / control_loop_rate,
            self.control_loop_callback
        )

        self.get_logger().info(f'Open Navigator Node started. Initial state: {self.state.name}')

    def _load_parameters(self):
        self.gain = self.get_parameter('gain').get_parameter_value().double_value

    def _setup_ros_communications(self):
        qos_profile_cmd_vel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(
            Twist, '/controller/cmd_vel', qos_profile=qos_profile_cmd_vel
        )
        
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
        with self.state_lock:
            if self.state == State.FINISHED:
                self._handle_state_finished()
                return
            
            if self.latest_scan_msg is None:
                self.get_logger().info("Waiting for first LaserScan message...", throttle_duration_sec=2)
                return

            msg = self.latest_scan_msg

            # State Dispatcher
            if self.state == State.PREPARATION:
                self._handle_state_preparation(msg)
            elif self.state == State.STRAIGHT:
                self._handle_state_straight(msg)
            elif self.state == State.TURNING:
                self._handle_state_turning(msg)

    def scan_callback(self, msg):
        with self.state_lock:
            if not msg.ranges: return
            ranges = list(msg.ranges)
            for i in range(len(ranges)):
                if ranges[i] > self.max_valid_range_m:
                    ranges[i] = math.inf
            msg.ranges = ranges

            if self.correct_mirrored_scan:
                self.latest_scan_msg = self._correct_mirrored_scan(msg)
            else:
                self.latest_scan_msg = msg

    def yaw_callback(self, msg):
        with self.state_lock:
            self.current_yaw_deg = msg.data

    # --- Main State Handlers ---
    def _handle_state_preparation(self, msg: LaserScan):
        if self.preparation_sub_state == PreparationSubState.WAITING_FOR_CONTROLLER:
            self._handle_preparation_sub_waiting_for_controller()
        elif self.preparation_sub_state == PreparationSubState.DETERMINE_DIRECTION:
            self._handle_preparation_sub_determine_direction(msg)

    def _handle_state_straight(self, msg):
        self._handle_straight_sub_align_with_outer_wall(msg)

    def _handle_state_turning(self, msg):
        if self.turning_sub_state == TurningSubState.POSITIONING_REVERSE:
            self._handle_turning_sub_positioning_reverse(msg)
        elif self.turning_sub_state == TurningSubState.APPROACH_CORNER:
            self._handle_turning_sub_approach_corner(msg)
        elif self.turning_sub_state == TurningSubState.EXECUTE_PIVOT_TURN:
            self._handle_turning_sub_execute_pivot_turn(msg)
        elif self.turning_sub_state == TurningSubState.FINALIZE_TURN:
            self._handle_turning_sub_finalize_turn()

    def _handle_state_finished(self):
        self.publish_twist_with_gain(0.0, 0.0)
        if self.start_time is not None:
            end_time = self.get_clock().now()
            duration_total_seconds = (end_time - self.start_time).nanoseconds / 1e9
            minutes = int(duration_total_seconds // 60)
            seconds = duration_total_seconds % 60
            self.get_logger().info("=============================================")
            self.get_logger().info("               R U N   F I N I S H E D               ")
            self.get_logger().info(f"    Elapsed Time: {minutes} minutes, {seconds:.1f} seconds")
            self.get_logger().info("=============================================")
            self.start_time = None

    # --- Preparation Sub-States ---
    def _handle_preparation_sub_waiting_for_controller(self):
        controller_ready = self.controller_ready_client.service_is_ready()
        yaw_publisher_ready = self.yaw_publisher_ready_client.service_is_ready()
        if not controller_ready or not yaw_publisher_ready:
            self.get_logger().info('Waiting for dependency services...', throttle_duration_sec=1.0)
            return
        self.get_logger().info("All dependency services are ready. Determining direction.")
        self.preparation_sub_state = PreparationSubState.DETERMINE_DIRECTION

    def _handle_preparation_sub_determine_direction(self, msg: LaserScan):
        """
        Determines course direction, then continues to move forward using IMU only
        until the first corner is detected.
        """
        # --- Direction Determination Logic (runs only once) ---
        if self.direction is None:
            left_dist = self.get_distance_at_world_angle(msg, self.current_yaw_deg + 90.0)
            right_dist = self.get_distance_at_world_angle(msg, self.current_yaw_deg - 90.0)
            open_threshold = 1.5

            if not math.isnan(left_dist) and left_dist > open_threshold:
                self.direction = "ccw"
            elif not math.isnan(right_dist) and right_dist > open_threshold:
                self.direction = "cw"
            
            if self.direction is not None:
                self.get_logger().warn(f"Direction determined: {self.direction.upper()}. Proceeding to first corner with IMU only.")
        
        # --- Check for the first corner ---
        # Once direction is set, _calculate_base_angle works, so we can check for a corner.
        if self.direction is not None:
            is_turning, _ = self._check_for_corner(msg, base_angle_deg=0.0)
            if is_turning:
                # If a corner is found, the state transition is handled by _check_for_corner.
                return

        # --- Continuous Action: Move forward with IMU ONLY ---
        self.get_logger().debug("In PREPARATION, moving forward with IMU only.", throttle_duration_sec=1.0)
        self._execute_pid_alignment(
            msg=msg,
            base_angle_deg=0.0, # Target is to go straight
            speed=0.17,
            disable_dist_control=True # IMU_ONLY
        )

    # --- Straight Sub-State ---
    def _handle_straight_sub_align_with_outer_wall(self, msg: LaserScan):
        if self._check_for_finish_condition(msg):
            return
            
        base_angle_deg = self._calculate_base_angle()
        self._update_turn_permission_counter(msg, base_angle_deg)
        
        is_turning, _ = self._check_for_corner(msg, base_angle_deg)
        if is_turning:
            return

        # Speed is now always the standard forward speed.
        self._execute_pid_alignment(
            msg=msg, 
            base_angle_deg=base_angle_deg, 
            speed=self.forward_speed
        )

    # --- Turning Sub-States ---
    def _handle_turning_sub_positioning_reverse(self, msg: LaserScan):
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)
        approach_trigger_dist, _, _, _ = self._get_turn_strategy()
        reverse_target_dist = approach_trigger_dist + 0.05

        if not math.isnan(front_dist) and front_dist >= reverse_target_dist:
            self.get_logger().info(f"Positioning reverse complete. Transitioning to APPROACH_CORNER.")
            self.publish_twist_with_gain(0.0, 0.0)
            self.turning_sub_state = TurningSubState.APPROACH_CORNER
        else:
            self.get_logger().debug(f"Positioning Reverse... (Front: {front_dist:.2f}m)", throttle_duration_sec=0.2)
            self._execute_pid_alignment(
                msg=msg,
                base_angle_deg=self.approach_base_yaw_deg,
                speed=-self.turn_positioning_reverse_speed,
                disable_dist_control=True
            )

    def _handle_turning_sub_approach_corner(self, msg: LaserScan):
        front_dist = self.get_distance_at_world_angle(msg, self.approach_base_yaw_deg)
        if math.isnan(front_dist):
            self.get_logger().warn("Cannot get front distance for turn approach, stopping.", throttle_duration_sec=1.0)
            self.publish_twist_with_gain(0.0, 0.0)
            return

        trigger_dist, _, approach_speed, _ = self._get_turn_strategy()

        if front_dist <= trigger_dist:
            self.get_logger().info(f"Front wall is close ({front_dist:.2f}m). Transitioning to EXECUTE_PIVOT_TURN.")
            self.turning_sub_state = TurningSubState.EXECUTE_PIVOT_TURN
            self.publish_twist_with_gain(0.0, 0.0)
            return

        # Determine if IMU_ONLY mode should be forced
        is_first_turn = self.turn_count == 0
        
        self.get_logger().debug(f"Approaching corner for turn {self.turn_count + 1}. IMU_ONLY: {is_first_turn}", throttle_duration_sec=0.5)

        self._execute_pid_alignment(
            msg=msg, 
            base_angle_deg=self.approach_base_yaw_deg, 
            speed=approach_speed,
            disable_dist_control=is_first_turn # Force IMU_ONLY for the first turn
        )

    def _handle_turning_sub_execute_pivot_turn(self, msg: LaserScan):
        _, turn_amount_deg, _, base_turn_speed = self._get_turn_strategy()
        
        angle_turned_deg = abs(self._angle_diff(self.current_yaw_deg, self.approach_base_yaw_deg))
        remaining_angle_deg = turn_amount_deg - angle_turned_deg

        if remaining_angle_deg <= 2.0:
            self.get_logger().info("Pivot turn complete. Finalizing turn.")
            self.turning_sub_state = TurningSubState.FINALIZE_TURN
            self.publish_twist_with_gain(0.0, 0.0)
            return
        
        turn_kp = 0.02
        proportional_steer = turn_kp * remaining_angle_deg
        steer_direction = 1.0 if self.direction == 'ccw' else -1.0
        final_steer = proportional_steer * steer_direction

        min_turn_speed = base_turn_speed * 0.7
        speed_reduction_factor = max(0, 1.0 - (angle_turned_deg / turn_amount_deg))
        final_speed = min_turn_speed + (base_turn_speed - min_turn_speed) * speed_reduction_factor

        dynamic_max = self._get_dynamic_max_steer(final_speed)
        final_steer = np.clip(final_steer, -dynamic_max, dynamic_max)

        self.publish_twist_with_gain(final_speed, final_steer)

    def _handle_turning_sub_finalize_turn(self):
        self.turn_count += 1
        self.wall_segment_index = (self.wall_segment_index + 1) % 4
        self.get_logger().info(f"Turn {self.turn_count} complete. Entering new segment: {self.wall_segment_index}")
        
        self.state = State.STRAIGHT
        
        self.inner_wall_far_counter = 0 
        self.turning_sub_state = None
        self.can_start_new_turn = False
        self.stable_alignment_counter = 0
        self.publish_twist_with_gain(0.0, 0.0)

    # --- Core Driving/Action Functions ---
    def publish_twist_with_gain(self, linear_x, angular_z):
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_pub_time).nanoseconds / 1e6 < self.cmd_pub_interval_ms:
            return
        self.last_cmd_pub_time = current_time

        target_linear = linear_x * self.gain * self.recovery_gain
        target_angular = angular_z * self.gain * self.recovery_angular_gain
        
        dt = 1.0 / 50.0
        max_delta_linear = self.max_linear_acceleration * dt
        final_linear_x = np.clip(
            target_linear,
            self.last_published_linear - max_delta_linear,
            self.last_published_linear + max_delta_linear
        )
        max_delta_angular = self.max_angular_acceleration_rad * dt
        final_angular_z = np.clip(
            target_angular,
            self.last_published_angular - max_delta_angular,
            self.last_published_angular + max_delta_angular
        )

        tolerance = 1e-4 
        is_linear_changed = abs(final_linear_x - self.last_published_linear) > tolerance
        is_angular_changed = abs(final_angular_z - self.last_published_angular) > tolerance
        is_stopping = abs(final_linear_x) < tolerance and abs(final_angular_z) < tolerance
        was_moving = abs(self.last_published_linear) > tolerance or abs(self.last_published_angular) > tolerance

        if is_linear_changed or is_angular_changed or (is_stopping and was_moving):
            twist_msg = Twist()
            twist_msg.linear.x = final_linear_x
            twist_msg.angular.z = final_angular_z
            self.publisher_.publish(twist_msg)
            
            self.last_published_linear = final_linear_x
            self.last_published_angular = final_angular_z

    def _execute_pid_alignment(self, msg: LaserScan, base_angle_deg: float,
                            speed: float, override_target_dist: float = None,
                            disable_dist_control: bool = False):
        # Safety check: if direction is not yet known, force IMU_ONLY mode.
        if self.direction is None:
            disable_dist_control = True

        wall_offset_deg = -90.0 if self.direction == 'ccw' else 90.0
        
        target_dist = self.align_target_outer_dist_m
        if override_target_dist is not None:
            target_dist = override_target_dist

        wall_angle = self._angle_normalize(base_angle_deg + wall_offset_deg)
        wall_dist = self.get_distance_at_world_angle(msg, wall_angle)
        
        angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg

        dist_steer = 0.0
        log_mode = "NORMAL"

        if not disable_dist_control:
            # The 'inner_wall_far_counter' check is now removed.
            # Distance control will remain active even when approaching a corner.
            if not math.isnan(wall_dist) and abs(target_dist - wall_dist) > self.align_dist_tolerance_m:
                dist_error = target_dist - wall_dist
                dist_steer_multiplier = 1.0 if self.direction == 'ccw' else -1.0
                dist_steer = self.align_kp_dist * dist_error * dist_steer_multiplier
        else:
            log_mode += "_IMU_ONLY"
        
        angular_z = angle_steer + dist_steer
        dynamic_max = self._get_dynamic_max_steer(speed)
        final_steer = np.clip(angular_z, -dynamic_max, dynamic_max)
        
        self.get_logger().debug(
            f"ALIGN_OUTER({log_mode}) | WallD:{wall_dist:.2f} TgtD:{target_dist:.2f} | " 
            f"YawErr:{angle_error_deg:.1f} | DistSteer:{dist_steer:.2f} | "
            f"FinalSteer:{final_steer:.2f}",
            throttle_duration_sec=0.2
        )
        self.publish_twist_with_gain(speed, final_steer)

    # --- State Logic Helpers ---
    def _get_turn_strategy(self):
        dist = self.turn_approach_dist_m
        return dist, self.turn_angle_deg, self.turn_approach_speed, self.turn_speed

    def _check_for_corner(self, msg: LaserScan, base_angle_deg: float) -> tuple[bool, float]:
        inner_wall_offset = 90.0 if self.direction == 'ccw' else -90.0
        inner_wall_angle = self._angle_normalize(base_angle_deg + inner_wall_offset)
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
        
        self.get_logger().debug(f"[CornerCheck] Turn:{self.turn_count} | I_Dist:{inner_wall_dist:.2f} | Counter:{self.inner_wall_far_counter}/{self.inner_wall_disappear_count} | CanTurn:{self.can_start_new_turn}", throttle_duration_sec=0.5)

        if self.turn_count > self.max_turns:
            self.get_logger().warn(f"BACKUP TRIGGER: Exceeded max_turns ({self.max_turns}). Forcing finish.")
            self.state = State.FINISHED
            self.publish_twist_with_gain(0.0, 0.0)
            return True, inner_wall_dist 

        if not math.isnan(inner_wall_dist) and inner_wall_dist >= self.inner_wall_disappear_threshold:
            self.inner_wall_far_counter += 1
        else:
            self.inner_wall_far_counter = 0

        if self.inner_wall_far_counter >= self.inner_wall_disappear_count and self.can_start_new_turn:
            self.get_logger().info("Corner detected. Starting turning sequence.")
            self.approach_base_yaw_deg = base_angle_deg
            self.state = State.TURNING
            self.turning_sub_state = TurningSubState.POSITIONING_REVERSE
            self.publish_twist_with_gain(0.0, 0.0)
            return True, inner_wall_dist
            
        return False, inner_wall_dist

    def _check_for_finish_condition(self, msg: LaserScan) -> bool:
        """
        Checks if the robot has completed the required laps and is in a stable
        straight section before finishing.
        """
        # Only check for finish condition on the final lap.
        if self.turn_count < self.max_turns:
            return False

        base_angle_deg = self._calculate_base_angle()
        
        # --- Get sensor data for condition checking ---
        front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)
        inner_offset = 90.0 if self.direction == 'ccw' else -90.0
        outer_offset = -inner_offset
        inner_dist = self.get_distance_at_world_angle(msg, self._angle_normalize(base_angle_deg + inner_offset))
        outer_dist = self.get_distance_at_world_angle(msg, self._angle_normalize(base_angle_deg + outer_offset))
        
        # --- Check if stability conditions for finishing are met ---
        is_stable_for_finish = False
        if not math.isnan(inner_dist) and not math.isnan(outer_dist) and not math.isnan(front_dist):
            course_width = inner_dist + outer_dist
            # Condition: Course width is normal AND front is reasonably close.
            if (0.9 < course_width < 1.1) and (front_dist < 1.75):
                is_stable_for_finish = True

        # --- Update the counter ---
        if is_stable_for_finish:
            self.finish_stability_counter += 1
        else:
            # If conditions are ever not met on the final lap, reset the counter.
            self.finish_stability_counter = 0

        self.get_logger().debug(
            f"[FinishCheck] Counter: {self.finish_stability_counter}/{self.finish_stability_threshold} "
            f"Stable: {is_stable_for_finish}",
            throttle_duration_sec=0.5
        )

        # --- Check for final completion ---
        if self.finish_stability_counter >= self.finish_stability_threshold:
            self.get_logger().warn(
                f"FINISH: Stable final straight detected after {self.turn_count} turns. "
                f"Front dist: {front_dist:.2f}m. Halting."
            )
            self.state = State.FINISHED
            self.publish_twist_with_gain(0.0, 0.0) # Send immediate stop command
            return True
            
        return False

    def _update_turn_permission_counter(self, msg: LaserScan, base_angle_deg: float):
        if self.can_start_new_turn: return

        inner_offset = 90.0 if self.direction == 'ccw' else -90.0
        outer_offset = -inner_offset
        inner_dist = self.get_distance_at_world_angle(msg, self._angle_normalize(base_angle_deg + inner_offset))
        outer_dist = self.get_distance_at_world_angle(msg, self._angle_normalize(base_angle_deg + outer_offset))
        
        if not math.isnan(inner_dist) and not math.isnan(outer_dist) and (inner_dist + outer_dist) < 1.2:
            self.stable_alignment_counter += 1
        
        if self.stable_alignment_counter > 50: 
            self.can_start_new_turn = True
            self.stable_alignment_counter = 0
            self.get_logger().warn("Stable alignment detected. New turn detection is ENABLED.")

    # --- Sensor Data & Geometry Helpers ---
    def get_distance_at_world_angle(self, scan_data, world_angle_deg):
        robot_local_angle_deg = world_angle_deg - self.current_yaw_deg
        target_angle_rad = math.radians(robot_local_angle_deg)
        while target_angle_rad < scan_data.angle_min: target_angle_rad += 2.0 * math.pi
        while target_angle_rad >= scan_data.angle_min + 2.0 * math.pi: target_angle_rad -= 2.0 * math.pi

        if scan_data.angle_increment <= 0.0: return float("nan")
        index = int((target_angle_rad - scan_data.angle_min) / scan_data.angle_increment)

        num_ranges = len(scan_data.ranges)
        if not (0 <= index < num_ranges): return float("nan")

        indices_to_check = [index, (index + 1) % num_ranges, (index - 1 + num_ranges) % num_ranges]

        for i in indices_to_check:
            distance = scan_data.ranges[i]
            if not math.isnan(distance) and not math.isinf(distance):
                return distance
        return float("nan")

    def _correct_mirrored_scan(self, msg: LaserScan) -> LaserScan:
        corrected_msg = LaserScan()
        corrected_msg.header = msg.header
        corrected_msg.angle_min, corrected_msg.angle_max = msg.angle_min, msg.angle_max
        corrected_msg.angle_increment, corrected_msg.time_increment = msg.angle_increment, msg.time_increment
        corrected_msg.scan_time, corrected_msg.range_min, corrected_msg.range_max = msg.scan_time, msg.range_min, msg.range_max
        corrected_msg.ranges = list(reversed(msg.ranges))
        if msg.intensities:
            corrected_msg.intensities = list(reversed(msg.intensities))
        return corrected_msg

    def _calculate_base_angle(self):
        return self.wall_segment_index * (90.0 if self.direction == 'ccw' else -90.0)

    def _angle_normalize(self, angle_deg):
        while angle_deg >= 180.0: angle_deg -= 360.0
        while angle_deg < -180.0: angle_deg += 360.0
        return angle_deg

    def _angle_diff(self, target_deg, current_deg):
        diff = target_deg - current_deg
        while diff <= -180.0: diff += 360.0
        while diff > 180.0: diff -= 360.0
        return diff

    def _get_dynamic_max_steer(self, current_speed: float) -> float:
        base_speed = self.forward_speed
        if base_speed < 0.01: base_speed = 0.01
        speed_ratio = abs(current_speed) / base_speed
        dynamic_max_steer = self.max_steer * speed_ratio
        min_allowable_steer = 0.3
        return np.clip(dynamic_max_steer, min_allowable_steer, self.max_steer)

def main(args=None):
    rclpy.init(args=args)
    node = OpenNavigatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
