# ~/test_wro0620/src/peripherals/launch/lidar_with_filter.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    peripherals_pkg_dir = get_package_share_directory('peripherals')
    filter_config_path = os.path.join(peripherals_pkg_dir, 'config', 'ld19_filter.yaml')

    # --- Launch Configurations ---
    # The final, filtered topic name that other nodes will use
    scan_topic = LaunchConfiguration('scan_topic')
    # The raw, unfiltered topic name from the driver
    scan_raw_topic = LaunchConfiguration('scan_raw_topic')
    # The TF frame for the lidar
    lidar_frame = LaunchConfiguration('lidar_frame')

    # --- Node Definitions ---
    # 1. Lidar Driver Node
    # This node reads from the hardware and publishes to the 'scan_raw_topic'
    lidar_driver_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld19_driver',
        output='screen',
        parameters=[
            {
                'product_name': 'LDLiDAR_LD19',
                'topic_name': scan_raw_topic, # Publish to the raw topic
                'port_baudrate': 230400,
                'port_name': '/dev/ldlidar',
                'frame_id': lidar_frame,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0
            }
        ]
    )

    # 2. Laser Filter Node
    # This node subscribes to 'scan_raw_topic' and publishes the filtered data to 'scan_topic'
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter', # This name must match the top key in the YAML file
        parameters=[filter_config_path],
        remappings=[
            # Remap the filter's input from 'scan' to our raw topic
            ('scan', scan_raw_topic),
            # Remap the filter's output from 'scan_filtered' to our final topic
            ('scan_filtered', scan_topic)
        ]
    )

    return LaunchDescription([
        # --- Declare Arguments with Default Values ---
        DeclareLaunchArgument(
            'scan_topic',
            default_value='scan',
            description='Topic name for the filtered laser scan data'
        ),
        DeclareLaunchArgument(
            'scan_raw_topic',
            default_value='scan_raw',
            description='Topic name for the raw laser scan data'
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='lidar_frame',
            description='TF frame ID for the lidar'
        ),
        
        # --- Add nodes to the launch description ---
        lidar_driver_node,
        laser_filter_node,
    ])