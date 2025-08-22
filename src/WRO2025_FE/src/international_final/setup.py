import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'international_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
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
            'run_obstacle_navigator = international_final.obstacle_navigator_node:main',
            'run_open_navigator = international_final.open_navigator_node:main',
            'run_manager = international_final.run_manager:main',
        ],
    },
)
