from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # joy node (reads joystick device)
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                # device param can be set here if needed,
                # or leave default to /dev/input/js0
                # 'dev': '/dev/input/js0'
            }]
        ),

        # teleop_twist_joy: converts joystick -> cmd_vel
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy",
            output="screen",
            parameters=[{
                "axis_linear.x": 1,
                "axis_linear.y": 0,
                "axis_angular.yaw": 3,
                "scale_linear.x": 0.1,
                "scale_linear.y": 0.1,
                "scale_angular.yaw": 0.1,
                # set buttons/enable as you like
                "enable_button": 6,
                "enable_turbo_button": 7,
                "require_enable_button": True,
            }],
        ),

        # our serial driver
        Node(
            package="mecanum_joystick",
            executable="serial_driver",
            name="serial_driver",
            output="screen",
            parameters=[{
                "port": "/dev/ttyACM0",
                "baudrate": 115200,
                "wheel_radius": 0.03,
                "wheel_separation_x": 0.375,
                "wheel_separation_y": 0.290,
                "max_rpm": 10,
                "send_rate_hz": 20.0
            }]
        )
    ])
