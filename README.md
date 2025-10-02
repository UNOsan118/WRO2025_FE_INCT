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

### 7.1. Overall Strategy: Hierarchical State Machine
*   (Introduce the concept of the state machine.)
*   (Embed a high-level flowchart showing the main state transitions: `PREPARATION` -> `UNPARKING` -> `STRAIGHT` -> `TURNING` -> `PARKING`.)

### 7.2. Dynamic Strategy Selection
*   **Unparking Strategy (4 Patterns):** Explain how LiDAR and camera data at the start determine the optimal unparking sequence. (Include a simple decision tree diagram.)
*   **Turning Strategy (8 Patterns):** Describe the logic in `_get_turn_strategy()`, explaining how the robot dynamically adjusts its cornering based on the current lane, next lane plan, and obstacle presence.

### 7.3. Core Logic (Pseudo Code & Source Code)
*   (Present simplified pseudo-code for key functions like `_check_for_corner` and `_execute_pid_alignment` to make the logic understandable.)
*   (Provide a direct link to the fully commented `obstacle_navigator_node.py` source file.)

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