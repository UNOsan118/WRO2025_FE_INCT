from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts ONLY the fuzzy logic driving node.
    The controller and LIDAR are expected to be running already.
    """
    obstacle_navigator_node = Node(
        package='japan_final',
        executable='run_obstacle_navigator',  # Use the executable name from setup.py
        name='obstacle_navigator_node',   # Define a unique name for this node
        output='screen',
        emulate_tty=True, # Often useful for seeing colored logs
    )

    return LaunchDescription([
        obstacle_navigator_node
    ])