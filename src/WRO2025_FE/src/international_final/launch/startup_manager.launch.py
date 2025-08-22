import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    This is the main launch file for system startup.
    It starts the hardware controller, the LIDAR, and the run manager immediately.
    After a 10-second delay, it plays a beep to indicate that all systems are ready.
    """

    # These lines find the install paths of other ROS2 packages
    imu_test_pkg_dir = get_package_share_directory('imu')
    peripherals_pkg_dir = get_package_share_directory('peripherals')
    controller_pkg_dir = get_package_share_directory('controller')

    # 1. Launch the hardware controller
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg_dir, 'launch', 'controller.launch.py')
        )
    )

    # 2. IMU 
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_test_pkg_dir, 'launch', 'yaw.launch.py')
        )
    )

    # 3. LiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg_dir, 'launch', 'lidar_with_filter.launch.py')
        )
    )

    # 4. Camera
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg_dir, 'launch', 'usb_cam.launch.py')
        )
    )

    # 5. Launch our run manager node that waits for the button press
    run_manager_node = Node(
        package='international_final',
        executable='run_manager',
        name='run_manager_node',
        output='screen'
    )
    
    # --- Action to be executed after a delay ---

    # 6. Define the action to play a "system ready" beep
    system_ready_beep_node = Node(
        package='startup_test',
        executable='simple_beep',
        name='system_ready_beep_node',
    )

    # Use a TimerAction to execute the beep after a 10-second delay
    delayed_beep_action = TimerAction(
        period=10.0,
        actions=[system_ready_beep_node]
    )

    # --- Assemble the final LaunchDescription ---
    # The order in this list does not strictly control execution order,
    # but grouping them logically improves readability.
    return LaunchDescription([
        # Start these immediately
        controller_launch,
        imu_launch,
        lidar_launch,
        cam_launch,
        run_manager_node,
        
        # Start this after the delay
        delayed_beep_action
    ])