import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Get Package Directories ---
    peripherals_pkg_dir = get_package_share_directory('peripherals')
    controller_pkg_dir = get_package_share_directory('controller')

    # --- Declare Launch Arguments ---
    gain_arg = DeclareLaunchArgument(
        'gain', default_value='2.0',
        description="Gain for both linear and angular speeds"
    )

    # --- Include other Launch Files ---
    # This part launches the necessary drivers and controllers
    """
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_test_pkg_dir, 'launch', 'yaw.launch.py')
        )
    )
    """

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg_dir, 'launch', 'lidar_with_filter.launch.py')
        )
    )
    """
    cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg_dir, 'launch', 'usb_cam.launch.py')
        )
    )
    """
    motor_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg_dir, 'launch', 'controller.launch.py')
        )
    )

    # --- Define the Open Navigator Node ---
    open_navigator_node = Node(
        package='international_final',
        executable='run_open_navigator',
        name='open_navigator_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gain': LaunchConfiguration('gain'),
            # Other parameters will use their default values defined in the node.
        }]
    )

    # --- Assemble the Launch Description ---
    return LaunchDescription([
        gain_arg,
        
        # imu_launch,
        lidar_launch,
        # cam_launch, # Removed
        motor_controller_launch,
        
        open_navigator_node
    ])