import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Get Package Directories ---
    # These lines find the install paths of other ROS2 packages
    imu_test_pkg_dir = get_package_share_directory('imu')
    peripherals_pkg_dir = get_package_share_directory('peripherals')
    controller_pkg_dir = get_package_share_directory('controller')

    # --- Declare Launch Arguments ---
    # This allows you to change values from the command line
    # e.g., ros2 launch fusion_test state_machine_wall_follower.launch.py direction:=cw
    direction_arg = DeclareLaunchArgument(
        'direction', default_value='ccw',
        description="Wall following direction ('cw' or 'ccw')"
    )
    gain_arg = DeclareLaunchArgument(
        'gain', default_value='2.0', # Adjusted default for potentially smoother start
        description="Gain for both linear and angular speeds"
    )
    speed_arg = DeclareLaunchArgument(
        'speed', default_value='0.2', # Adjusted default
        description="Base forward speed in m/s"
    )

    # --- Include other Launch Files ---
    # This part launches the necessary drivers and controllers
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_test_pkg_dir, 'launch', 'yaw.launch.py')
        )
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg_dir, 'launch', 'lidar_with_filter.launch.py')
        )
    )
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg_dir, 'launch', 'usb_cam.launch.py')
        )
    )
    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg_dir, 'launch', 'controller.launch.py')
        )
    )

    # --- Define the State Machine Wall Follower Node ---
    # This is the main change from the reference file
    obstacle_navigator_node = Node(
        package='international_final',
        executable='run_obstacle_navigator',  # Use the executable name from setup.py
        name='obstacle_navigator_node',   # Define a unique name for this node
        output='screen',
        emulate_tty=True, # Often useful for seeing colored logs
        parameters=[{
            'forward_speed': LaunchConfiguration('speed'),
            'max_steer': 0.8, # This can be made an argument too if needed
            'gain': LaunchConfiguration('gain'),
            'direction': LaunchConfiguration('direction'),
        }]
    )

    # --- Assemble the Launch Description ---
    # The order here doesn't usually matter, but grouping is good practice
    return LaunchDescription([
        # Arguments must be declared before they are used
        direction_arg,
        gain_arg,
        speed_arg,
        
        # Include the other robot components
        imu_launch,
        lidar_launch,
        cam_launch,
        motor_controller_launch,
        
        # Add our new node to the launch list
        obstacle_navigator_node
    ])