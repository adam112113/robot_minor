from setuptools import setup
import os
from glob import glob

package_name = 'robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    #     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     ('share/' + package_name + '/launch', ['launch/robot_slam.launch.py']),
    #     ('share/' + package_name + '/config', ['config/rplidar_a1.yaml', 'config/slam_toolbox.yaml']),
    # ],
        # Include package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include launch and config directories
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Mecanum robot driver, odometry, and SLAM integration using RPLiDAR A1.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'motion_controller_node = robot_slam.motion_controller_node:main',
            'serial_driver = robot_slam.serial_driver:main',
            'odometry = robot_slam.odometry:main',
        ],
    },
)


