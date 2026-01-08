import os
from glob import glob
from setuptools import setup

package_name = 'mecanum_joystick'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install the YAML mapping file so launch files can pass it as a parameter
        (os.path.join('share', package_name), ['mecanum_joystick/mappings.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Joystick to Arduino serial bridge for mecanum robot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_driver = mecanum_joystick.serial_driver:main',
            'joy_to_twist = mecanum_joystick.joy_to_twist:main',
        ],
    },
)

