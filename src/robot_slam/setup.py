from setuptools import setup

package_name = 'mecanum_joystick'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick_serial.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Joystick to Arduino serial bridge for mecanum robot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = robot_slam.'
            'serial_driver = mecanum_joystick.serial_driver:main',
        ],
    },
)

