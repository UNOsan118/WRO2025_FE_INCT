import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_pkg_dir = get_package_share_directory('imu')
    calibration_file_path = os.path.join(imu_pkg_dir, 'config', 'bno055_full_calibration.json')

    yaw_publisher_node = Node(
        package='imu',
        executable='yaw_publisher',
        name='yaw_publisher_node',
        output='screen',
        parameters=[
            {
                'calibration_file': calibration_file_path,
                'frame_id': 'base_link', # Or your robot's base frame
                'publish_rate': 30.0,
                'zero_on_start': True
            }
        ]
    )
    
    return LaunchDescription([
        yaw_publisher_node
    ])