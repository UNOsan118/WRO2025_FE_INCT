# ~/test_wro0620/src/imu_test/launch/bno055.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    imu_test_pkg_dir = get_package_share_directory('imu_test')
    
    calibration_file_path = os.path.join(imu_test_pkg_dir, 'config', 'bno055_imu_calibration.json')

    bno055_node = Node(
        package='imu_test',
        executable='bno055_publisher',
        name='bno055_publisher_node',
        output='screen',
        parameters=[
            {
                'calibration_file': calibration_file_path,
                'frame_id': 'bno055_imu_link',
                'publish_rate': 50.0
            }
        ]
    )
    
    return LaunchDescription([
        bno055_node
    ])