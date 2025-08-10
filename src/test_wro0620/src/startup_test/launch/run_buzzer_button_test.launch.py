import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    This launch file starts two main components:
    1. The core hardware controller launch file.
    2. The buzzer and button test node, delayed by 10 seconds.
    """

    # 1. Define the action to launch the controller
    controller_pkg_path = get_package_share_directory('controller')
    
    controller_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg_path, 'launch', 'controller.launch.py')
        )
    )

    # 2. Define the node for our buzzer and button test
    buzzer_button_test_node = Node(
        package='startup_test',
        executable='buzzer_button_test',
        name='buzzer_button_test_node',
        output='screen'
    )

    # 3. Create a TimerAction to delay the launch of the test node
    # This will wait for 10 seconds after the launch file starts,
    # and then execute the 'actions' list.
    delayed_buzzer_button_test = TimerAction(
        period=10.0,  # The delay in seconds
        actions=[buzzer_button_test_node] # The action(s) to execute after the delay
    )

    # 4. Create a LaunchDescription and add the actions.
    # The controller will start immediately, and the test node will start after the delay.
    return LaunchDescription([
        controller_launch_file,
        delayed_buzzer_button_test
    ])