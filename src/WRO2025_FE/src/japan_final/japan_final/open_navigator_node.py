import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
from enum import Enum, auto
from std_srvs.srv import Trigger
import threading

# --- Simplified State Machine Enums ---
class State(Enum):
    DETERMINE_COURSE = auto()
    STRAIGHT = auto()
    FINISHED = auto()


class DetermineCourseSubState(Enum):
    SEARCHING = auto()
    REVERSING = auto()


class StraightSubState(Enum):
    ALIGNING = auto()
    TURNING = auto()
    REVERSING_FOR_TURN = auto()


class OpenNavigatorNode(Node):
    def __init__(self):
        super().__init__("open_navigator_node")

        # --- State Machine ---
        self.state = State.DETERMINE_COURSE
        self.determine_course_sub_state = DetermineCourseSubState.SEARCHING
        self.straight_sub_state = StraightSubState.ALIGNING

        # --- Core Driving Parameters ---
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("max_steer", 0.8)
        self.declare_parameter("gain", 3.0)
        self.declare_parameter("direction", "ccw")
        self.declare_parameter("correct_mirrored_scan", True)

        # --- Course Logic Parameters ---
        self.declare_parameter("max_turns", 12)
        self.declare_parameter("finish_front_dist_m", 1.5)

        # --- Corner Detection Parameters ---
        self.declare_parameter("corner_inner_dist_threshold_m", 1.5)
        self.declare_parameter("corner_detection_count", 3)
        self.declare_parameter("post_turn_stabilization_sec", 2.0)

        # --- Wall Following (ALIGNING) Parameters ---
        self.declare_parameter("align_kp_angle", 0.03)
        self.declare_parameter("align_kp_dist", 4.0)
        self.declare_parameter("align_target_outer_dist_m", 0.3)
        self.declare_parameter("align_dist_tolerance_m", 0.03)

        # --- Cornering (TURNING) Parameters ---
        self.declare_parameter("turn_in_kp_angle", 0.04)
        self.declare_parameter("turn_in_angle_deg", 45.0)
        self.declare_parameter("turn_in_target_dist_m", 0.43)
        self.declare_parameter("pre_turn_reverse_dist_m", 0.58)

        # --- Course Detection Parameters ---
        self.declare_parameter("course_detection_threshold_m", 1.5)
        self.declare_parameter("course_detection_speed", 0.15)
        self.declare_parameter("course_detection_reverse_dist_m", 0.55)

        # --- Logging Level ---
        self.declare_parameter("log_level", "INFO")

        self._load_parameters()

        # --- Internal State Variables ---
        self.state_lock = threading.RLock()
        self.turn_count = 0
        self.inner_wall_far_counter = 0
        self.wall_segment_index = 0
        self.current_yaw_deg = 0.0
        self.latest_scan_msg = None
        self.imu_ready = False
        self.is_stabilizing = False
        self.inner_wall_approached = False

        self.max_valid_range_m = 3.0

        # --- System Setup ---
        self._setup_ros_communications()
        self.controller_ready_client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        self.wait_for_controller_ready()

        self.cmd_pub_interval_ms = 33 
        self.last_cmd_pub_time = self.get_clock().now()

        # --- MODIFICATION: High-frequency control loop ---
        control_loop_rate = 50.0 # Hz
        self.control_loop_timer = self.create_timer(
            1.0 / control_loop_rate,
            self.control_loop_callback
        )

        # --- Data Logging Timer ---
        self.data_log_timer = self.create_timer(0.5, self._log_sensor_data)

        self.get_logger().info(f"Open Navigator Node started. Initial state: {self.state.name}")

    def _load_parameters(self):
        """Loads all ROS2 parameters into class variables."""
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.max_steer = self.get_parameter('max_steer').get_parameter_value().double_value
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().string_value
        self.correct_mirrored_scan = self.get_parameter('correct_mirrored_scan').get_parameter_value().bool_value
        self.max_turns = self.get_parameter('max_turns').get_parameter_value().integer_value
        self.finish_front_dist_m = self.get_parameter('finish_front_dist_m').get_parameter_value().double_value
        self.post_turn_stabilization_sec = self.get_parameter('post_turn_stabilization_sec').get_parameter_value().double_value
        self.corner_inner_dist_threshold_m = self.get_parameter('corner_inner_dist_threshold_m').get_parameter_value().double_value
        self.corner_detection_count = self.get_parameter('corner_detection_count').get_parameter_value().integer_value
        self.align_kp_angle = self.get_parameter('align_kp_angle').get_parameter_value().double_value
        self.align_kp_dist = self.get_parameter('align_kp_dist').get_parameter_value().double_value
        self.align_target_outer_dist_m = self.get_parameter('align_target_outer_dist_m').get_parameter_value().double_value
        self.align_dist_tolerance_m = self.get_parameter('align_dist_tolerance_m').get_parameter_value().double_value
        self.turn_in_kp_angle = self.get_parameter('turn_in_kp_angle').get_parameter_value().double_value
        self.turn_in_angle_deg = self.get_parameter('turn_in_angle_deg').get_parameter_value().double_value
        self.turn_in_target_dist_m = self.get_parameter('turn_in_target_dist_m').get_parameter_value().double_value
        self.pre_turn_reverse_dist_m = self.get_parameter('pre_turn_reverse_dist_m').get_parameter_value().double_value
        self.course_detection_threshold_m = self.get_parameter('course_detection_threshold_m').get_parameter_value().double_value
        self.course_detection_speed = self.get_parameter('course_detection_speed').get_parameter_value().double_value
        self.course_detection_reverse_dist_m = self.get_parameter('course_detection_reverse_dist_m').get_parameter_value().double_value
        
        log_level_str = self.get_parameter('log_level').get_parameter_value().string_value
        log_level_map = {'DEBUG': rclpy.logging.LoggingSeverity.DEBUG, 'INFO': rclpy.logging.LoggingSeverity.INFO}
        log_level = log_level_map.get(log_level_str.upper(), rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().set_level(log_level)

    def _setup_ros_communications(self):
        """Initializes all publishers and subscribers."""
        self.publisher_ = self.create_publisher(Twist, "/controller/cmd_vel", 10)

        qos_profile_lidar = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_subscriber = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_lidar
        )
        self.yaw_subscriber = self.create_subscription(
            Float64, "/imu/yaw", self.yaw_callback, 10
        )

        rclpy.get_default_context().on_shutdown(lambda: self.publisher_.publish(Twist()))

    def wait_for_controller_ready(self):
        """
        Waits until the ros_robot_controller node reports that it is ready.
        This is a blocking call intended to be used in __init__.
        """
        self.get_logger().info("Waiting for ros_robot_controller to be ready...")
        while not self.controller_ready_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Controller service not available, waiting again...')
        
        request = Trigger.Request()
        future = self.controller_ready_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info("ros_robot_controller is ready. Starting navigation.")
        else:
            self.get_logger().error("Failed to confirm controller readiness. The node might not function correctly.")

    def control_loop_callback(self):
        """
        Main logic loop, running at a high frequency (e.g., 50Hz).
        It uses the latest available sensor data to make decisions.
        """
        with self.state_lock:
            # Wait until both IMU and LiDAR are ready
            if not self.imu_ready or self.latest_scan_msg is None:
                if not self.imu_ready:
                    self.get_logger().info("Waiting for IMU to be ready...", throttle_duration_sec=2)
                if self.latest_scan_msg is None:
                    self.get_logger().info("Waiting for first LaserScan message...", throttle_duration_sec=2)
                return
            
            # Use the stored message for all logic in this loop iteration
            msg = self.latest_scan_msg

            # Dispatch to the appropriate state handler
            if self.state == State.DETERMINE_COURSE:
                self._handle_state_determine_course(msg)
            elif self.state == State.STRAIGHT:
                self._handle_state_straight(msg)
            elif self.state == State.FINISHED:
                self.publisher_.publish(Twist())

    def scan_callback(self, msg):
        with self.state_lock:
            """Callback for LaserScan data. Filters out long ranges and saves the message."""
            if not msg.ranges:
                return

            # --- NEW: Filter out ranges greater than max_valid_range_m ---
            ranges = list(msg.ranges)
            for i in range(len(ranges)):
                if ranges[i] > self.max_valid_range_m:
                    ranges[i] = math.inf
            msg.ranges = ranges
            # --- END OF NEW ---

            if self.correct_mirrored_scan:
                self.latest_scan_msg = self._correct_mirrored_scan(msg)
            else:
                self.latest_scan_msg = msg

    def yaw_callback(self, msg):
        """Stores the latest yaw angle from the IMU and sets the ready flag."""
        with self.state_lock:
            self.current_yaw_deg = msg.data
            if not self.imu_ready:
                self.imu_ready = True
                self.get_logger().info("IMU is ready. Navigator will start moving.")
    
    def _handle_state_determine_course(self, msg):
        """Dispatches to the correct sub-state handler for course determination."""
        if self.determine_course_sub_state == DetermineCourseSubState.SEARCHING:
            self._handle_determine_sub_searching(msg)
        elif self.determine_course_sub_state == DetermineCourseSubState.REVERSING:
            self._handle_determine_sub_reversing(msg)

    def _handle_state_straight(self, msg):
        """Dispatches to the correct handler based on the straight_sub_state."""
        if self.straight_sub_state == StraightSubState.ALIGNING:
            self._handle_straight_sub_aligning(msg)
        elif self.straight_sub_state == StraightSubState.TURNING:
            self._handle_straight_sub_turning(msg)
        elif self.straight_sub_state == StraightSubState.REVERSING_FOR_TURN:
            self._handle_straight_sub_reversing_for_turn(msg)

    def _handle_determine_sub_searching(self, msg):
        """
        Determines course direction while following the closer wall.
        If a direction is found, transitions to the REVERSING sub-state.
        """
        course_found, _ = self._check_course_and_get_direction(msg)
        if course_found:
            self.get_logger().info(
                f"Course direction set to: {self.direction.upper()}. Reversing to create space."
            )
            self.determine_course_sub_state = DetermineCourseSubState.REVERSING
            self.publisher_.publish(Twist())  # Stop briefly before reversing
            return

        # PID wall following to maintain a set distance while searching
        left_dist = self.get_distance_at_world_angle(msg, 90.0)
        right_dist = self.get_distance_at_world_angle(msg, -90.0)

        use_left_wall = not math.isnan(left_dist) and (
            math.isnan(right_dist) or left_dist < right_dist
        )

        target_yaw_deg = 0.0
        angle_error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg

        dist_steer = 0.0
        target_dist = 0.3
        wall_dist_to_log = 0.0
        if use_left_wall:
            dist_error = target_dist - left_dist
            dist_steer = self.align_kp_dist * dist_error * -1.0
            wall_dist_to_log = left_dist
        else:
            dist_error = target_dist - right_dist
            dist_steer = self.align_kp_dist * dist_error * 1.0
            wall_dist_to_log = right_dist

        angular_z = angle_steer + dist_steer
        final_steer = np.clip(angular_z, -self.max_steer, self.max_steer)

        self.publish_twist(self.course_detection_speed, final_steer)
        self.get_logger().debug(
            f"Searching... WallDist: {wall_dist_to_log:.2f}, Steer: {final_steer:.2f}",
            throttle_duration_sec=0.2,
        )

    def _handle_determine_sub_reversing(self, msg):
        """Reverses straight back until a target distance from the front wall is reached."""
        front_dist = self.get_distance_at_world_angle(msg, 0.0)

        if math.isnan(front_dist):
            self.get_logger().warn(
                "Cannot get front distance for reversing. Reversing slowly.",
                throttle_duration_sec=1.0,
            )
            self.publish_twist(-self.course_detection_speed * 0.5, 0.0)
            return

        # Check for completion of the reverse maneuver
        if front_dist >= self.course_detection_reverse_dist_m:
            self.get_logger().info(
                f"Reverse complete (FrontDist: {front_dist:.2f}m). Starting first turn."
            )
            self.state = State.STRAIGHT
            self.straight_sub_state = StraightSubState.TURNING
            self.publisher_.publish(Twist())
        else:
            # Reverse straight back (no steering)
            self.get_logger().debug(
                f"Reversing... Target: > {self.course_detection_reverse_dist_m:.2f}m, "
                f"Current: {front_dist:.2f}m",
                throttle_duration_sec=0.2,
            )
            self.publish_twist(-self.course_detection_speed, 0.0)

    def _handle_straight_sub_aligning(self, msg):
        """Follows the outer wall, manages the inner wall approach flag, and checks for corners."""
        if self._check_for_finish_condition(msg):
            return

        base_angle_deg = self._calculate_base_angle()

        # Get sensor readings for logic below
        front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)
        inner_wall_angle_offset = 90.0 if self.direction == "ccw" else -90.0
        inner_wall_angle = self._angle_normalize(base_angle_deg + inner_wall_angle_offset)
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)
        angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)

        # 1. Arm corner detection only when the robot is stable and near the inner wall.
        if not self.inner_wall_approached and not self.is_stabilizing:
            is_inner_wall_close = not math.isnan(inner_wall_dist) and inner_wall_dist < 1.0
            is_angle_stable = abs(angle_error_deg) < 30.0

            if is_inner_wall_close and is_angle_stable:
                self.get_logger().info(
                    f"Inner wall approached (Dist: {inner_wall_dist:.2f}m, YawErr: {angle_error_deg:.1f}deg). "
                    "Corner detection is now armed."
                )
                self.inner_wall_approached = True

        # 2. Check for corners, but only if not in post-turn stabilization and armed.
        if not self.is_stabilizing and self.inner_wall_approached:
            # --- Expanded Corner Detection Logic ---
            # Condition 1: Inner wall has disappeared
            is_inner_wall_gone = self._check_for_corner(msg, base_angle_deg)
            # Condition 2: Front wall is extremely close
            is_front_wall_critical = not math.isnan(front_dist) and front_dist < 0.5

            if is_inner_wall_gone or is_front_wall_critical:
                log_reason = (
                    "Inner wall disappeared"
                    if is_inner_wall_gone
                    else f"Front wall critical ({front_dist:.2f}m)"
                )
                self.get_logger().info(f"Corner detected ({log_reason}). Reversing to create space.")
                self.straight_sub_state = StraightSubState.REVERSING_FOR_TURN
                self.publisher_.publish(Twist())
                return
                
        elif self.is_stabilizing:
            self.get_logger().debug(
                "Stabilizing after turn... Corner detection paused.",
                throttle_duration_sec=0.5,
            )
        elif not self.inner_wall_approached:
            log_dist = f"{inner_wall_dist:.2f}m" if not math.isnan(inner_wall_dist) else "N/A"
            self.get_logger().debug(
                f"Waiting for inner wall approach (Dist: {log_dist}, YawErr: {angle_error_deg:.1f}deg)... "
                "Corner detection unarmed.",
                throttle_duration_sec=0.5,
            )

        # 3. If no corner is detected, continue wall following.
        self._execute_pid_alignment(msg, base_angle_deg)

    def _handle_straight_sub_reversing_for_turn(self, msg):
        """Reverses until a safe distance is reached, then starts the turn."""
        base_angle_deg = self._calculate_base_angle()

        # Condition 1: Front wall distance
        front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)
        is_front_dist_safe = (
            not math.isnan(front_dist) and front_dist >= self.pre_turn_reverse_dist_m
        )

        # Condition 2: Sum of side walls distance (fallback)
        left_angle = self._angle_normalize(base_angle_deg + 90.0)
        right_angle = self._angle_normalize(base_angle_deg - 90.0)
        left_dist = self.get_distance_at_world_angle(msg, left_angle)
        right_dist = self.get_distance_at_world_angle(msg, right_angle)

        is_side_dist_safe = False
        side_dist_sum = float("nan")
        if not math.isnan(left_dist) and not math.isnan(right_dist):
            side_dist_sum = left_dist + right_dist
            if side_dist_sum < 1.3:
                is_side_dist_safe = True

        # Completion Check (OR condition)
        if is_front_dist_safe or is_side_dist_safe:
            log_reason = (
                f"FrontDist: {front_dist:.2f}m"
                if is_front_dist_safe
                else f"SideSum: {side_dist_sum:.2f}m"
            )
            self.get_logger().info(f"Reverse complete ({log_reason}). Starting turn.")
            self.straight_sub_state = StraightSubState.TURNING
            self.publisher_.publish(Twist())
        else:
            self.get_logger().debug(
                f"Reversing for turn... Front: {front_dist:.2f}m, SideSum: {side_dist_sum:.2f}m",
                throttle_duration_sec=0.2,
            )
            self.publish_twist(-self.course_detection_speed, 0.0)

    def _handle_straight_sub_turning(self, msg):
        """Turns in towards the next outer wall until a target distance is met."""
        # Calculate angles relative to the NEXT wall segment
        next_wall_segment_index = (self.wall_segment_index + 1) % 4
        next_base_angle_deg = next_wall_segment_index * (
            90.0 if self.direction == "ccw" else -90.0
        )
        direction_multiplier = 1.0 if self.direction == "ccw" else -1.0

        # Target yaw is an aggressive angle relative to the NEXT wall to cut the corner
        target_yaw_deg = self._angle_normalize(
            next_base_angle_deg - (self.turn_in_angle_deg * direction_multiplier)
        )

        # Completion is checked against the NEXT outer wall
        next_outer_wall_angle_offset = -90.0 if self.direction == "ccw" else 90.0
        next_outer_wall_angle = self._angle_normalize(
            next_base_angle_deg + next_outer_wall_angle_offset
        )
        wall_dist = self.get_distance_at_world_angle(msg, next_outer_wall_angle)

        # Completion Logic: If we are close enough to the next wall
        if not math.isnan(wall_dist) and wall_dist <= self.turn_in_target_dist_m:
            self.turn_count += 1
            self.wall_segment_index = next_wall_segment_index
            self.get_logger().info(
                f"Turn {self.turn_count} complete (Dist to next wall: {wall_dist:.2f}m). Resuming wall following."
            )
            self.straight_sub_state = StraightSubState.ALIGNING
            self.publisher_.publish(Twist())

            # Start stabilization timer to prevent immediate re-triggering of corner detection
            self.is_stabilizing = True
            self.create_timer(self.post_turn_stabilization_sec, self._finish_stabilization)

            # Reset the inner wall approach flag for the next segment
            self.inner_wall_approached = False
            return

        # P-Control Steering Logic
        error_deg = self._angle_diff(target_yaw_deg, self.current_yaw_deg)
        angular_z = self.turn_in_kp_angle * error_deg
        final_steer = np.clip(angular_z, -self.max_steer, self.max_steer)

        self.get_logger().debug(
            f"Turning... TargetYaw: {target_yaw_deg:.1f}, NextWallDist: {wall_dist:.2f}, Steer: {final_steer:.2f}",
            throttle_duration_sec=0.2,
        )
        self.publish_twist(self.forward_speed * 0.7, final_steer)
    
    def _check_for_finish_condition(self, msg: LaserScan) -> bool:
        """Checks if the robot has completed the required number of turns and is near a wall."""
        if self.turn_count >= self.max_turns:
            base_angle_deg = self._calculate_base_angle()
            front_dist = self.get_distance_at_world_angle(msg, base_angle_deg)

            if not math.isnan(front_dist) and front_dist < self.finish_front_dist_m:
                self.get_logger().info(
                    f"FINISH CONDITION MET: {self.turn_count} turns and front dist {front_dist:.2f}m."
                )
                self.state = State.FINISHED
                self.publisher_.publish(Twist())
                return True
        return False

    def _check_for_corner(self, msg: LaserScan, base_angle_deg: float) -> bool:
        """Checks if the inner wall has disappeared, indicating a corner."""
        inner_wall_angle_offset = 90.0 if self.direction == "ccw" else -90.0
        inner_wall_angle = self._angle_normalize(base_angle_deg + inner_wall_angle_offset)
        inner_wall_dist = self.get_distance_at_world_angle(msg, inner_wall_angle)

        if (
            not math.isnan(inner_wall_dist)
            and inner_wall_dist >= self.corner_inner_dist_threshold_m
        ):
            self.inner_wall_far_counter += 1
        else:
            self.inner_wall_far_counter = 0

        return self.inner_wall_far_counter >= self.corner_detection_count

    def _execute_pid_alignment(self, msg: LaserScan, base_angle_deg: float):
        """A PID controller for aligning the robot parallel to the outer wall."""
        outer_wall_angle_offset = -90.0 if self.direction == "ccw" else 90.0
        wall_angle = self._angle_normalize(base_angle_deg + outer_wall_angle_offset)
        wall_dist = self.get_distance_at_world_angle(msg, wall_angle)

        angle_error_deg = self._angle_diff(base_angle_deg, self.current_yaw_deg)
        angle_steer = self.align_kp_angle * angle_error_deg

        dist_steer = 0.0
        if not math.isnan(wall_dist):
            dist_error = self.align_target_outer_dist_m - wall_dist

            # If the angle error is too large, prioritize correcting the angle
            if abs(angle_error_deg) > 30.0:
                self.get_logger().debug(
                    f"Angle error {angle_error_deg:.1f} is large, ignoring dist_steer."
                )
                dist_error = 0.0

            if abs(dist_error) > self.align_dist_tolerance_m:
                # CCW: Outer wall is on the RIGHT. To move away, steer LEFT (+).
                # CW:  Outer wall is on the LEFT.  To move away, steer RIGHT (-).
                dist_steer_multiplier = 1.0 if self.direction == "ccw" else -1.0
                dist_steer = self.align_kp_dist * dist_error * dist_steer_multiplier

        angular_z = angle_steer + dist_steer
        final_steer = np.clip(angular_z, -self.max_steer, self.max_steer)

        self.get_logger().debug(
            f"Aligning... WallD: {wall_dist:.2f}, YawErr: {angle_error_deg:.1f}, "
            f"AngleSteer: {angle_steer:.2f}, DistSteer: {dist_steer:.2f}, Steer: {final_steer:.2f}",
            throttle_duration_sec=0.2,
        )
        self.publish_twist(self.forward_speed, final_steer)

    def _check_course_and_get_direction(self, msg: LaserScan) -> tuple[bool, str]:
        """Checks left and right wall distances to determine course direction."""
        left_dist = self.get_distance_at_world_angle(msg, 90.0)
        right_dist = self.get_distance_at_world_angle(msg, -90.0)

        if not math.isnan(left_dist) and left_dist > self.course_detection_threshold_m:
            self.direction = "ccw"
            return True, "ccw"
        if not math.isnan(right_dist) and right_dist > self.course_detection_threshold_m:
            self.direction = "cw"
            return True, "cw"
        return False, ""

    def publish_twist(self, linear_x, angular_z):
        """
        Applies gain and publishes the Twist message, with throttling to prevent
        flooding the serial port of the controller.
        """
        current_time = self.get_clock().now()
        duration_since_last_pub = (current_time - self.last_cmd_pub_time).nanoseconds / 1e6

        if duration_since_last_pub < self.cmd_pub_interval_ms:
            return # Skip publishing

        self.last_cmd_pub_time = current_time
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_x * self.gain
        twist_msg.angular.z = angular_z * self.gain
        self.publisher_.publish(twist_msg)
    
    def _correct_mirrored_scan(self, msg: LaserScan) -> LaserScan:
        """Corrects a mirrored (clockwise) LaserScan message."""
        corrected_msg = LaserScan()
        corrected_msg.header = msg.header
        (
            corrected_msg.angle_min,
            corrected_msg.angle_max,
            corrected_msg.angle_increment,
            corrected_msg.time_increment,
            corrected_msg.scan_time,
            corrected_msg.range_min,
            corrected_msg.range_max,
        ) = (
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment,
            msg.time_increment,
            msg.scan_time,
            msg.range_min,
            msg.range_max,
        )
        corrected_msg.ranges = list(reversed(msg.ranges))
        if msg.intensities:
            corrected_msg.intensities = list(reversed(msg.intensities))
        return corrected_msg

    def get_distance_at_world_angle(self, scan_data, world_angle_deg):
        """Gets the LiDAR distance at a specific world angle, with fallback to neighbors."""
        robot_local_angle_deg = world_angle_deg - self.current_yaw_deg
        target_angle_rad = math.radians(robot_local_angle_deg)

        while target_angle_rad < scan_data.angle_min:
            target_angle_rad += 2.0 * math.pi
        while target_angle_rad >= scan_data.angle_min + 2.0 * math.pi:
            target_angle_rad -= 2.0 * math.pi

        if scan_data.angle_increment <= 0.0:
            return float("nan")
        index = int((target_angle_rad - scan_data.angle_min) / scan_data.angle_increment)

        num_ranges = len(scan_data.ranges)
        if not (0 <= index < num_ranges):
            return float("nan")

        indices_to_check = [
            index,
            (index + 1) % num_ranges,
            (index - 1 + num_ranges) % num_ranges,
        ]

        for i in indices_to_check:
            distance = scan_data.ranges[i]
            if not math.isnan(distance) and not math.isinf(distance):
                return distance
        return float("nan")

    def _calculate_base_angle(self):
        """Calculates the ideal angle of the current wall segment."""
        return self.wall_segment_index * (90.0 if self.direction == "ccw" else -90.0)

    def _angle_normalize(self, angle_deg):
        """Normalize an angle to the range [-180, 180)."""
        while angle_deg >= 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def _angle_diff(self, target_deg, current_deg):
        """Calculates the shortest angle difference between two angles."""
        diff = target_deg - current_deg
        while diff <= -180.0:
            diff += 360.0
        while diff > 180.0:
            diff -= 360.0
        return diff
    
    def _log_sensor_data(self):
        """Periodically logs key sensor data for analysis if the logger level is DEBUG."""
        with self.state_lock:
            if self.get_logger().get_effective_level() > rclpy.logging.LoggingSeverity.DEBUG:
                return
            if self.latest_scan_msg is None:
                return

            base_angle = self._calculate_base_angle()
            angle_front = base_angle
            angle_left = self._angle_normalize(base_angle + 90.0)
            angle_right = self._angle_normalize(base_angle - 90.0)

            dist_front = self.get_distance_at_world_angle(self.latest_scan_msg, angle_front)
            dist_left = self.get_distance_at_world_angle(self.latest_scan_msg, angle_left)
            dist_right = self.get_distance_at_world_angle(self.latest_scan_msg, angle_right)

            f_str = f"{dist_front:6.3f}" if not math.isnan(dist_front) else "  ----"
            l_str = f"{dist_left:6.3f}" if not math.isnan(dist_left) else "  ----"
            r_str = f"{dist_right:6.3f}" if not math.isnan(dist_right) else "  ----"

            timestamp_sec = self.get_clock().now().nanoseconds / 1e9
            state_str = "UNKNOWN"
            if self.state == State.STRAIGHT:
                state_str = f"{self.state.name}:{self.straight_sub_state.name}"
            elif self.state == State.DETERMINE_COURSE:
                state_str = f"{self.state.name}:{self.determine_course_sub_state.name}"
            else:
                state_str = f"{self.state.name}"

            self.get_logger().debug(
                f"[DATA_LOG] {timestamp_sec:13.3f} | "
                f"State: {state_str:<35} | "
                f"Yaw: {self.current_yaw_deg:6.1f} | "
                f"Left: {l_str} | Front: {f_str} | Right: {r_str}"
            )

    def _finish_stabilization(self):
        """Callback function to finish the post-turn stabilization period."""
        with self.state_lock:
            self.get_logger().debug("Stabilization complete. Corner detection re-enabled.")
            self.is_stabilizing = False

def main(args=None):
    rclpy.init(args=args)
    node = OpenNavigatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()