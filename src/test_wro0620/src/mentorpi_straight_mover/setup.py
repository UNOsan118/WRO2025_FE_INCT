from setuptools import find_packages, setup

package_name = 'mentorpi_straight_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='11306260+liangfuyuan@user.noreply.gitee.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_movement_tester = mentorpi_straight_mover.straight_mover_node:main',
            'stop_robot = mentorpi_straight_mover.stop_robot_node:main',
            'imu_subscriber = mentorpi_straight_mover.imu_subscriber_node:main',
            'yaw_controller = mentorpi_straight_mover.yaw_controller_node:main',
        ],
    },
)
