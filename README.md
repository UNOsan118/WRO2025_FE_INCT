# WRO 2025 Future Engineers: Team Ishikawa KOSEN <img src="https://upload.wikimedia.org/wikipedia/en/9/9e/Flag_of_Japan.svg" alt="Flag of Japan" width="30"/>

<!-- A captivating main image of your robot -->
![Main Robot Image](./v-photos/main_view.jpg)

<!-- Social media badges (optional but nice) -->
[![YouTube](https://img.shields.io/badge/YouTube-%23FF0000.svg?style=for-the-badge&logo=youtube&logoColor=white)](https://youtube.com/your-channel-link)
[![Instagram](https://img.shields.io/badge/Instagram-%23E4405F.svg?style=for-the-badge&logo=instagram&logoColor=white)](https://instagram.com/your-profile-link)

This is the official repository for **Team Ishikawa KOSEN**, representing Japan in the World Robot Olympiad (WRO) 2025 Future Engineers category. This document provides a comprehensive overview of our robot's design, strategy, and implementation, developed by Shinichi Uno and Kosei Takano.

---

## 1. Team Members & Roles

<!-- A formal picture of the team -->
![Team Photo](t-photos/team_formal.jpg)

*   **[Member 1 Name]:** Lead Programmer & Strategist
*   **[Member 2 Name]:** Mechanical Designer & Systems Integrator
*   ... (add more members and their primary roles)

> [!NOTE]
> For more team pictures, including our journey, please visit the [`t-photos`](./t-photos) directory.

---

## 2. Repository Structure

A brief guide to navigating our project repository.

*   `src/`: Contains the core ROS 2 source code, including our main `obstacle_navigator_node`.
*   `models/`: 3D CAD files for all custom-designed and 3D-printed parts.
*   `schemes/`: Circuit diagrams, system architecture diagrams, and state-machine flowcharts.
*   `v-photos/`: A gallery of our robot from all angles.
*   `t-photos/`: Photos of our team.
*   `video/`: Performance videos demonstrating the robot's capabilities.
*   `README.md`: This file, detailing our entire engineering process.

---

## 3. Bill of Materials (BOM)

A complete list of all electrical and mechanical components used in our robot.

| Component | Image | Quantity | Link/Datasheet |
| :--- | :---: | :---: | :--- |
| **Control Unit** | | | |
| Hiwonder MentorPi A1 | `(image here)` | 1 | [Product Page](link) |
| Raspberry Pi 4B | `(image here)` | 1 | [Product Page](link) |
| **Sensors** | | | |
| YDLIDAR X2L | `(image here)` | 1 | [Datasheet](link) |
| RGB Camera | `(image here)` | 1 | [Product Page](link) |
| **Actuators** | | | |
| DC Motors (w/ Encoders) | `(image here)` | 2 | [Product Page](link) |
| Steering Servo | `(image here)` | 1 | [Product Page](link) |
| Pan/Tilt Servos | `(image here)` | 2 | [Product Page](link) |
| ... (add all other components) | | | |

---

## 4. Vehicle Overview & Photos

A high-level introduction to our robot's design philosophy and key features.

| Front View | Right View | Rear View |
| :---: | :---: | :---: |
| `(image)` | `(image)` | `(image)` |
| **Left View** | **Top View** | **Bottom View** |
| `(image)` | `(image)` | `(image)` |

> [!NOTE]
> For more detailed photos, including close-ups of specific mechanisms, please see the [`v-photos`](./v-photos) directory.

---

## 5. Mobility Management

This section covers our robot's mechanical design and movement capabilities.

### 5.1. Chassis Design
*   (Explanation of why the MentorPi A1 was chosen, any modifications made, and the rationale behind the overall structure.)
*   (Include 3D model renders from the `models` directory.)

### 5.2. Drive System
*   (Details about the motors, wheels, and power transmission. Explain the selection process based on torque, speed, and efficiency requirements.)

### 5.3. Steering System
*   (Description of the steering mechanism, servo selection, and any custom parts designed to improve precision and durability.)

---

## 6. Power and Sense Management

This section details the electrical systems and the sensors that allow our robot to perceive its environment.

### 6.1. Circuit Diagram
*   (Embed a clear and professional circuit diagram from the `schemes` directory.)
*   (Provide a brief explanation of the power distribution, connections between the Raspberry Pi, motor controller, and sensors.)

### 6.2. Sensor Suite
*   **YDLIDAR X2L:** Explain its role in wall detection, corner detection, and localization.
*   **RGB Camera:** Describe how it's used for obstacle color detection and strategic planning.
*   **IMU & Encoders:** Detail their contribution to precise odometry and heading control.

---

## 7. Obstacle Management Strategy

This is the core of our autonomous navigation logic, implemented in the `obstacle_navigator_node.py`.

### 7.1. Overall Strategy: A Hierarchical State Machine

To navigate the complex and dynamic environment of the WRO course, our robot employs a **Hierarchical State Machine (HSM)**. This software architecture allows us to systematically manage the robot's behavior by breaking down the overall task into a series of distinct, manageable states. The robot always knows its current state and transitions to the next state based on clear, predefined conditions derived from sensor data.

This approach ensures robust, predictable, and debuggable behavior. The system is built around six main states, each responsible for a specific phase of the mission.

**Main States Overview:**

*   **`PREPARATION`**: Initializes all systems, including the camera and sensors, and determines the overall course direction (CW/CCW) at the start. (Parking Start Only)
*   **`UNPARKING`**: Executes a dynamic, pre-planned strategy to safely exit the parking area and enter the main course. (Parking Start Only)
*   **`STRAIGHT`**: The primary driving state. The robot follows the wall using PID control, detects upcoming corners, and plans its avoidance strategy for the next segment.
*   **`TURNING`**: Executes a precise, multi-step pivot turn to navigate the 90-degree corners of the course.
*   **`PARKING`**: Upon completing the required number of laps, this state manages the final maneuver to park the robot safely in the designated area.
*   **`FINISHED`**: The final state where all motors are stopped, concluding the run.

The core of our robot's operation is the cyclical transition between the `STRAIGHT` and `TURNING` states, which allows it to repeatedly navigate the course segments and corners.

**High-Level State Transition Flowchart:**

![State Machine Flowchart](schemes\state_flowchart.png)

### 7.2. The ROS 2 Ecosystem: How Our Nodes Collaborate

Our autonomous navigation system is built entirely on the **Robot Operating System 2 (ROS 2)**, an advanced, open-source framework for robotics. ROS 2 allows us to structure our software in a modular and robust way, where independent programs called **Nodes** communicate with each other in real-time.

The core of this communication is the **Publish/Subscribe (Pub/Sub)** model. Sensor nodes "publish" data onto named channels called **Topics**, and our main logic node "subscribes" to these topics to receive the data. Similarly, our logic node publishes control commands, which the motor controller node subscribes to. This decoupled architecture makes the system highly flexible, scalable, and easy to debug.

Our system consists of three primary nodes that work in concert:

1.  **Sensor Drivers Node(s):** Responsible for interfacing with the hardware (LiDAR, IMU, Camera) and publishing their raw data to ROS 2 topics.
2.  **`ObstacleNavigatorNode` (The Brain):** This is the central logic node that we developed. It subscribes to all sensor data, processes it through the hierarchical state machine, makes decisions, and publishes velocity and servo commands.
3.  **Motor Controller Node (`ros_robot_controller`):** This node subscribes to the commands from our "Brain" and translates them into low-level electrical signals to control the motors and servos.

The following diagram illustrates the flow of data between these nodes.

**Node and Topic Communication Diagram:**

<p align="center">
    <img src="schemes\ros2_node_diagram.png" alt="Unparking Strategies" width="500">
</p>

### 7.3. Dynamic Strategy Selection

A key feature of our robot is its ability to dynamically select the optimal strategy in real-time, rather than following a single, hard-coded path. This adaptability is crucial for handling the various challenges on the course. We have implemented two primary dynamic selection systems: one for unparking and one for cornering.

#### 7.3.1. Unparking Strategy (4 Patterns)

**The Challenge:** Our initial approach was to exit the parking area first and then identify the obstacle color, but this resulted in significant time loss and inefficient movements. We realized that since the obstacle is visible from the starting position, a pre-emptive scan could optimize the entire sequence. The problem then became how to reliably select an exit strategy before moving, using only a single, potentially noisy, initial observation.

**Our Solution:** To address this, our robot executes a pre-unparking check to gather data and select one of four specialized exit strategies.

1.  **Direction Detection:** The robot first uses its LiDAR to determine the course direction by measuring the distance to the side walls.
2.  **Obstacle Proximity Scan:** It then aims its camera towards the exit and analyzes the size of the detected obstacle blob to determine if it is "close" or "far".
3.  **Strategy Selection:** Based on these two pieces of information, the robot selects the most appropriate strategy from the following decision matrix:

| Course Direction | Obstacle Presence | Selected Strategy | Description |
| :--- | :--- | :--- | :--- |
| **Clockwise** | Far | `STANDARD_EXIT_TO_OUTER_LANE` | A standard turn onto the wider outer lane. |
| **Clockwise** | Far (Red Obstacle) | `STANDARD_EXIT_TO_INNER_LANE` | A standard turn onto the tighter inner lane. |
| **Clockwise** | Close (Red Obstacle) | `AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CW` | A complex maneuver involving a forward push and turn to clear the obstacle. |
| **Counter-Clockwise**| Far | `STANDARD_EXIT_TO_OUTER_LANE` | A standard turn onto the wider outer lane. |
| **Counter-Clockwise**| Close | `AVOID_EXIT_OBSTACLE_TO_INNER_LANE_CCW`| A reverse maneuver to create space before turning onto the inner lane. |

<p align="center">
    <img src="schemes\Unparking_Strategies.jpg" alt="Unparking Strategies" width="500">
</p>

This intelligent selection process ensures a safe and efficient exit from the parking area, regardless of the initial conditions.

#### 7.3.2. Obstacle Planning Strategy: Proactive Corner Scanning

**The Challenge:**
Simply reacting to obstacles as they appear during high-speed driving is inefficient and risky. It takes too long to detect and process obstacles in real-time, leading to abrupt stops and suboptimal paths.

**Our Solution:**
Instead of reacting, we adopted a **proactive planning strategy**. Before entering a straight segment, the robot stops at the corner, scans the entire upcoming segment for obstacles, and decides on a complete path plan in advance. This "scan-then-drive" approach has a powerful advantage: the obstacle layout is determined only on the first lap, and this plan is reused on subsequent laps, allowing for much faster, smoother, and more confident driving.

This planning phase consists of three key steps:

1.  **Pre-Scan Positioning:**
    On the first lap, upon detecting a corner, the robot doesn't immediately turn. Instead, it precisely positions itself at a safe, optimal vantage point. Using its IMU for orientation correction, it aims its camera directly down the upcoming straight segment, ensuring a clear and stable view for the scan.

2.  **"Move-and-Scan" with Max-Blob Detection:**
    After positioning, the robot creeps forward very slowly while its camera continuously scans the segment. For each frame, it detects red and green obstacle "blobs" and calculates their area. To guard against sensor noise or fleeting changes in lighting, we don't rely on a single image. Instead, the final decision is based on the **largest blob area detected across the entire scan period**. This max-blob approach makes our detection system extremely robust. During this phase, the robot also specifically determines if an obstacle is blocking the entrance to the segment.

<p align="center">
  <img src="schemes/planning_scan_camera_view.png" alt="Camera view during planning scan" width="400">
  <img src="schemes/planning_scan_detection_result.png" alt="Blob detection result" width="400">
  <br>
  <em>Left: The raw camera view during a proactive scan. Right: The processed result, where only the detected red and green blobs are isolated for area calculation.</em>
</p>

The data from this scan is then analyzed to make a final decision. The image below shows the actual log output from our robot's analysis process. Over a scan of 36 frames, it found the largest green blob (area: 2122.0) to be significantly larger than the largest red blob and the detection threshold, leading to the final decision of a `'green_to_red'` obstacle pattern.

<p align="center">
  <img src="schemes\planning_analysis_log.png" alt="Log output of planning analysis" width="400">
  <br>
  <em>The actual log output, showing the data-driven decision process.</em>
</p>

3.  **Path Plan Generation (4 Patterns):**
    The result of the scan (e.g., "red only," "green-to-red") is combined with the current driving direction (CW/CCW) to select one of four possible avoidance plans for the next segment: `outer-only`, `inner-only`, `outer-to-inner`, or `inner-to-outer`. This decision is stored in our `avoidance_path_plan` array. This array acts as a complete "blueprint" for the course, enabling the robot to navigate smoothly on the second and third laps by simply referencing the plan without needing to scan again.

#### 7.3.3. Straight Strategy: PID Control and Obstacle Avoidance

**The Challenge:** The "straight" segments of the course are the most critical sections, as this is where obstacles are physically present. The robot must perform two core tasks in tandem: maintaining a precise, stable trajectory along a wall, while simultaneously executing smooth maneuvers to bypass these obstacles.

**Our Solution:** The `STRAIGHT` state is our answer to this challenge. Its logic is composed of three primary actions that work together to achieve fast and safe navigation.

1.  **Wall Approach (The "Turn-In" Maneuver):**
    This is the first and most crucial phase of obstacle avoidance. Instead of attempting to swerve around an obstacle, the robot executes a deliberate **"Turn-In"** maneuver (`AVOID_*_TURN_IN` sub-state). It purposefully steers towards the opposite wall at a sharp, predefined angle. This action proactively creates the necessary space to bypass the upcoming obstacle safely and predictably.

2.  **PID Wall-Following (Align and Pass):**
    This is the core of our stable driving logic. Whether driving normally or passing an obstacle, the robot uses a robust **PID (Proportional-Integral-Derivative) controller**. This system continuously fuses data from the **IMU** (for heading) and **LiDAR** (for distance) to make constant, minute steering corrections. This allows the robot to maintain a precise distance from its reference wall, ensuring a perfectly straight and stable trajectory. During an avoidance maneuver, this same logic is used to align with the *opposite* wall and glide past the obstacle.

<p align="center">
  <img src="schemes/pid_wall_following_principle.jpg" alt="PID Wall-Following Principle" width="600">
  <br>
  <em><b>For the case shown in this image,</b> the PID controller calculates two opposing commands. The <b>LiDAR</b>, detecting the robot is too far, generates a <em>Distance Error</em> commanding a <b>turn to the left</b>. Simultaneously, the <b>IMU</b>, detecting the inward angle, generates a <em>Heading Error</em> commanding a <b>turn to the right</b>. The final steering output is a weighted sum of these values, resulting in a smooth correction back to the target path.</em>
</p>

3.  **Lane Change for Complex Scenarios:**
    For the most challenging course layouts, our robot employs advanced lane change logic. **If both a red and a green obstacle exist within the same straight segment, the robot will execute a full lane change maneuver after clearing the first obstacle to position itself for the second.** This corresponds to our pre-planned strategies like `outer_to_inner` and ensures the robot can fluidly navigate even the most complex obstacle combinations without stopping.


#### 7.3.4. Turning Strategy (8 Patterns)

**The Challenge:** Simply making a 90-degree turn at every corner is inefficient. The optimal turning maneuver (e.g., approach distance, turn angle) depends on the robot's current path, its planned path for the *next* segment, and any obstacles present at the corner's entrance.

**Our Solution:** Our `_get_turn_strategy()` function acts as a dynamic decision-maker before every turn. It analyzes three key factors to select the best set of turning parameters from 8 possible combinations:

1.  **Current Lane:** Is the robot currently on the `Outer` or `Inner` path?
2.  **Next Lane Plan:** Is the plan for the next segment `Outer`, `Inner`, `Outer-to-Inner`, or `Inner-to-Outer`?
3.  **Entrance Obstacle:** Is there an obstacle detected at the entrance of the corner that requires a wider turn?

Based on these factors, the robot adjusts its behavior, allowing for tighter, faster turns when the path is clear, and safer, wider turns when navigating complex transitions or obstacles. This logic is the core of our robot's lap-time performance and reliability.

<p align="center">
    <img src="schemes\Turning_Strategies.jpg" alt="Unparking Strategies" width="500">
</p>
<p align="center">
    <img src="schemes\if_entrance_is_clear.jpg" alt="Unparking Strategies" width="600">
</p>

### 7.4. Key Algorithms & Engineering Decisions

Our robot's reliability is not the result of a single piece of code, but the evolution of several key algorithms born from trial and error. This section details the engineering decisions behind our most critical navigation functions.

#### 7.4.1. Corner Detection: From Fragile Vision to Robust LiDAR Logic

**Initial Idea & Failure:** Our first attempt at corner detection used the camera to find colored lines on the course floor. However, this proved unreliable. The lines were often obscured by obstacles, and varying lighting conditions caused frequent misdetections.

**Evolution to LiDAR & The "Double-Detection" Problem:** We then pivoted to a LiDAR-based approach: a corner is detected when the inner wall is no longer visible. While this worked reasonably well, it introduced a new, critical bug: when turning a corner from the outer lane, the robot would sometimes detect the *same corner twice*, causing it to fail.

**Final Solution:** To solve this, we implemented a counter mechanism. A corner is only confirmed when the inner wall has been out of sight for a *sustained number of consecutive scans*. This simple addition eliminated the double-detection bug and made our cornering logic exceptionally robust.

Here is the simplified logic in pseudo code:

```python
def check_for_corner(scan_data, state):
    """Detects a corner using a counter for robustness."""
    
    # Get distances to the inner wall (where corners appear) and the front wall.
    inner_dist = get_distance_to_inner_wall(scan_data)
    front_dist = get_distance_to_front_wall(scan_data)

    # A corner might be ahead if the inner wall vanishes AND we are approaching a front wall.
    is_corner_imminent = (inner_dist > INNER_WALL_DISAPPEAR_THRESHOLD and
                          front_dist < FRONT_WALL_APPROACH_THRESHOLD)

    if is_corner_imminent:
        # If the condition is met, increment the confirmation counter.
        state.corner_detection_counter += 1
    else:
        # Otherwise, reset it immediately.
        state.corner_detection_counter = 0

    # Only confirm the corner if the condition has been true for a sustained period.
    # This prevents false positives from sensor noise or temporary occlusions.
    if state.corner_detection_counter >= CORNER_CONFIRMATION_COUNT:
        return "CORNER_DETECTED"
    
    return "NO_CORNER"
```

#### 7.4.2. Lane Change Safety: From Reactive to Predictive

**Initial Idea & Failure:** Originally, a lane change was triggered *after* the robot thought it had passed an obstacle. We attempted to detect this by monitoring the side LiDAR data for a `NORMAL -> NARROW -> NORMAL` distance pattern. However, this reactive approach was highly inaccurate, especially with small 5cm obstacles. The robot often missed the transition or reacted too late.

**Final Solution:** The unreliable reactive method was completely replaced by our current `_is_safe_for_lane_change()` function. Instead of trying to detect the *moment* of passing, we now predictively check if the upcoming space is **stable and wide enough for a sustained period**. This shift from a reactive to a predictive model drastically improved the safety and reliability of our lane change maneuvers.

```python
# --- Pseudo Code for Safe Lane Change ---
def is_safe_for_lane_change(scan_data, state):
    """Predictively checks if the upcoming path is stable enough for a lane change."""
    
    # Get distances to all relevant walls.
    inner_dist = get_distance_to_inner_wall(scan_data)
    outer_dist = get_distance_to_outer_wall(scan_data)
    front_dist = get_distance_to_front_wall(scan_data)
    
    # Check for two conditions:
    # 1. Is the course width normal (not in a corner or wide-open area)?
    is_width_normal = (NORMAL_COURSE_WIDTH_MIN < inner_dist + outer_dist < NORMAL_COURSE_WIDTH_MAX)
    # 2. Is the path ahead clear of any immediate obstacles?
    is_front_clear = front_dist > MIN_FRONT_CLEARANCE_FOR_LANE_CHANGE
    
    if is_width_normal and is_front_clear:
        # If the path is stable and clear, increment the stability counter.
        state.lane_change_stability_counter += 1
    else:
        # If conditions are not met, the path is unstable. Reset the counter.
        state.lane_change_stability_counter = 0
        
    # A lane change is only permitted if the path has been confirmed as stable
    # for a certain number of consecutive checks.
    if state.lane_change_stability_counter >= STABILITY_CONFIRMATION_COUNT:
        return True # It's safe to change lanes.
        
    return False # It's not safe yet.
```

#### 7.4.3. Sensor Fusion: Overcoming the "Black Wall Problem" with IMU

**The Challenge:** Our biggest initial hurdle was frequent loss of LiDAR data. A simple approach of reading distances at fixed angles (e.g., -90°, 0°, 90°) failed constantly. This was due to two compounding factors: the black walls of the course absorb light, and LiDAR signals weaken significantly when not hitting a surface perpendicularly.

**The "Aha!" Moment & Solution:** Through experimentation, we discovered that even the black walls provide reliable readings if, and only if, the laser hits them at a **perfect 90-degree angle**. This led to the development of our core sensor fusion function, `get_distance_at_world_angle()`.

This function uses the robot's real-time orientation from the **IMU** to continuously calculate the *exact LiDAR angle needed to hit the wall perpendicularly*, regardless of the robot's own orientation. By dynamically selecting the perfect laser beam at every moment, we transformed a "fatal flaw" of the LiDAR into a reliable navigation tool.

<p align="center">
    <img src="schemes\IMU-Corrected_LiDAR_Sensing.jpg" alt="Unparking Strategies" width="500">
</p>

```python
# --- Pseudo Code for IMU + LiDAR Sensor Fusion ---
def get_distance_at_world_angle(world_angle_deg, current_yaw_deg, scan_data):
    """Gets a reliable wall distance by correcting for the robot's orientation."""

    # --- This is the core of the fusion logic ---
    # To hit a wall at a specific "world" angle (e.g., 90 degrees),
    # calculate the angle the LiDAR must look at relative to the robot's own front.
    robot_local_angle = world_angle_deg - current_yaw_deg
    
    # Find the specific laser beam in the 360-degree scan that matches this local angle.
    lidar_index = find_lidar_index_for_angle(robot_local_angle, scan_data)
    
    # Get the primary distance measurement from that single, perfect beam.
    distance = scan_data.ranges[lidar_index]

    # As a fallback, if the primary beam failed (e.g., hit a seam in the wall),
    # check its immediate neighbors to get a valid reading.
    if not is_valid(distance):
        distance = check_neighboring_beams(lidar_index, scan_data)

    return distance
```



### 7.5. Source Code: A Modular ROS 2 Ecosystem

Our software is not a single script, but a complete, modular robotics application built on the **ROS 2 framework**. The entire system is organized into a collection of specialized ROS 2 packages, each with a distinct responsibility. This "separation of concerns" is a key principle of modern software engineering, making our system robust, maintainable, and easy to extend.

The directory tree below shows the high-level structure of our ROS 2 workspace, which is composed of our core logic packages and the necessary hardware drivers.

```
src
├─ driver/ # Low-level hardware drivers for the motor controller
├─ imu/ # Node for the external IMU sensor
├─ international_final/ # Core logic for the main obstacle challenge
├─ japan_final/ # Logic adapted for the Japan Final (if different)
└─ peripherals/ # Launch files and configurations for camera, etc.
```

#### 7.5.1. The Brain: Core Navigation Logic

The central intelligence of our robot resides in the `international_final` package. This is where sensor data is processed, decisions are made via our state machine, and commands are issued. We have developed two primary navigator nodes for the different challenges.

*   **`obstacle_navigator_node.py` (Obstacle Challenge)**
    This is the heart of our system. It contains the full implementation of our Hierarchical State Machine, dynamic strategy selection, PID control, and all the algorithms described in the previous sections. The code is extensively commented to explain the purpose of each function and the logic behind our engineering decisions.
    > **[View Source Code: obstacle_navigator_node.py](src\WRO2025_FE\src\international_final\international_final\obstacle_navigator_node.py)**

*   **`open_navigator_node.py` (Open Challenge)**
    This is a simplified version of the navigator for the challenge without obstacles. It focuses purely on high-speed, stable wall-following and cornering.
    > **[View Source Code: open_navigator_node.py](src\WRO2025_FE\src\international_final\international_final\open_navigator_node.py)**

#### 7.5.2. Supporting Packages: Drivers and Interfaces

The "Brain" cannot function alone. It relies on a suite of supporting packages that act as the bridge to the physical hardware.

| Package Name | Role & Purpose |
| :--- | :--- |
| `ros_robot_controller` | This driver package interfaces directly with the Hiwonder motor controller board (STM32). It subscribes to `/controller/cmd_vel` and other command topics, translating high-level ROS commands into low-level motor and servo signals. |
| `ros_robot_controller_msgs`| Defines the **custom message and service types** (e.g., `SetPWMServoState`) that we created for communication between our logic node and the controller. This is crucial for controlling peripherals like the camera's pan-tilt servos. |
| `imu` | This package contains the node responsible for reading data from our external BNO055 IMU sensor and publishing the critical yaw (heading) data to the `/imu/yaw` topic. |
| `peripherals` | A utility package that contains the main **launch files** for starting the entire system, as well as configurations for hardware like the camera. |

#### 7.5.3. How to Run: Launch Files

To meet the goal of reproducibility, the entire system can be started with a single command using ROS 2 launch files. These files are the designated entry points that correctly start all the necessary nodes (drivers, logic, etc.) with the correct parameters.

Our main launch file, which brings the entire robot to life, can be found here:
> **[View Main Launch File: startup_manager.launch.py](src\WRO2025_FE\src\international_final\launch\startup_manager.launch.py)**

This modular and well-documented structure ensures that our work can be understood, verified, and reproduced by the judges.

### 7.6. Future Improvements

While we are proud of our current system, the engineering process is one of continuous improvement. Based on our testing and development experience, we have identified several key areas for future enhancement. This demonstrates our commitment to pushing the boundaries of what our robot can achieve.

| Area for Improvement | Current Status & Limitations | Proposed Future Enhancement |
| :--- | :--- | :--- |
| **State Machine Robustness** | Our state machine is currently implemented with a series of `if/elif` statements. While effective, it can become complex to manage as more states are added. | Transition to a formal state machine library (like `SMACH` for ROS 2). This would provide greater robustness, clearer state transition definitions, and better visualization tools. |
| **Localization Accuracy** | We currently rely on a turn counter and corner detection for localization. This can be prone to errors if a corner is missed or misidentified. | Implement a more advanced localization algorithm, such as an **Extended Kalman Filter (EKF)**, to fuse data from the IMU, wheel encoders, and LiDAR. This would provide a much more accurate and reliable estimate of the robot's position on the course at all times. |
| **Parameter Tuning** | Our many control parameters (e.g., PID gains, turning speeds) are currently tuned manually through extensive trial and error. This process is time-consuming and may not yield the absolute optimal values. | Develop a simulation environment (e.g., in Gazebo) to automate the tuning process. Using optimization algorithms or machine learning (Reinforcement Learning), we could automatically discover the best set of parameters for maximum speed and stability. |

These planned improvements show that our current design is not an endpoint, but a solid foundation upon which even more advanced and intelligent behaviors can be built.

---

## 8. 3D Models & Custom Parts

*   (Showcase some of the most important custom-designed parts, such as sensor mounts or protective bumpers.)
*   (Include renders and a brief description of their purpose.)
*   (Provide a link to the `models` directory for access to all `.stl` or `.step` files.)

---

## 9. Performance Videos

*   (Embed or link to YouTube videos demonstrating the robot successfully completing the course.)
*   (It's recommended to have separate videos for different challenges, e.g., with and without obstacles, parking start, etc.)

---

## 10. References

*   [WRO 2025 Future Engineers Rules](link-to-rules)
*   [ROS 2 Documentation](https://docs.ros.org/)
*   (Any other significant libraries, tutorials, or resources used.)