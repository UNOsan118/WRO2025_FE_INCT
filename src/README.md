# ROS 2 Workspace and Package Overview

This document provides an overview of the ROS 2 workspace structure and the roles of each package within the `src` directory.

## Workspace Structure

The parent directory, `WRO2025_FE`, serves as the **root of our ROS 2 workspace**. A ROS 2 workspace is the top-level folder that contains the source code for our packages, as well as the build artifacts and installed executables.

## Build Process

All packages within this workspace are built using the `colcon` build tool.

### 1. Building the Workspace

To build all the packages, navigate to the workspace root (`WRO2025_FE`) and run the following command:

```bash
colcon build
```

This command will automatically create the following directories in the workspace root:

*   **`build/`**: Contains intermediate files generated during the build process.
*   **`install/`**: Contains the final output of the build, including executables, libraries, and the crucial `setup` files needed to run our nodes.
*   **`log/`**: Contains log files from the build process, which are useful for debugging build errors.

### 2. Sourcing the Workspace

After a successful build, you must **source** the workspace's setup file. This command makes the ROS 2 system aware of our custom packages and nodes, allowing them to be launched. This must be done in every new terminal you open.

```bash
source install/setup.bash
```

## Packages in `src`

This `src` directory contains all the source code for our project, organized into the following ROS 2 packages:

*   **`driver`**: Contains the low-level hardware driver packages (`ros_robot_controller`, etc.) responsible for interfacing directly with the Hiwonder motor controller board (STM32).

*   **`imu`**: A dedicated package that contains the node for communicating with our external BNO055 IMU sensor and publishing its data.

*   **`international_final`**: The core package for the international competition. It contains the main navigation logic (`obstacle_navigator_node.py`), launch files, and configurations for the obstacle challenge.

*   **`japan_final`**: A version of the navigation logic that was specifically adapted for the Japan Final.
inations of nodes from different packages.

*   **`peripherals`**: A utility package for managing peripheral devices. It contains configurations and launch files for components like the camera.

## Standard Structure of a ROS 2 Python Package

Each package in our workspace follows the standard directory structure for ROS 2 Python packages. This convention helps to systematically organize code, configuration files, and launch scripts. The roles of the key directories and files are explained below.

*   **`[package_name]/` directory (e.g., `international_final/`):**
    *   **Role:** This directory, named after the package, contains the actual Python source code that becomes the executable node. Scripts like `obstacle_navigator_node.py` are placed here.

*   **`launch/` directory:**
    *   **Role:** Stores the "launch scripts" (`.launch.py` files) used to start ROS 2 nodes. A single launch file can start multiple nodes simultaneously with specific parameters, which is essential for launching our complex system with a single command.

*   **`resource/` directory:**
    *   **Role:** Contains a "marker file" required for ROS 2 to correctly identify the package. Usually, an empty file with the same name as the package (e.g., `international_final`) is placed here.

*   **`test/` directory:**
    *   **Role:** Holds automated test code to verify that the package's code functions correctly. This is used to ensure software quality.

*   **`package.xml` file:**
    *   **Role:** Acts as the "business card" of the package. It contains metadata such as the package name, version, author, and, most importantly, **the other ROS 2 packages it depends on**. The `colcon build` tool uses this file to determine the correct build order.

*   **`setup.py` file:**
    *   **Role:** Serves as the "instruction manual" for the `colcon` build system. This file specifies which Python scripts should be treated as executable nodes (`entry_points`) and where to install data files like the `launch` directory.

*   **`setup.cfg` file:**
    *   **Role:** A configuration file that assists `setup.py`. It typically contains basic information to help the build system locate the settings defined in `setup.py`.