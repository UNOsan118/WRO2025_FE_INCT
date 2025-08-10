# setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'imu_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='BNO055 publisher for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055_publisher = imu_test.bno055_publisher_node:main',
            'yaw_publisher = imu_test.yaw_publisher_node:main',
            'yaw_subscriber = imu_test.yaw_subscriber_node:main',
        ],
    },
    scripts=['scripts/calibrate_imu.py'],
)