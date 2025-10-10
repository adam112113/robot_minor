import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generates the launch description for the robot
    """

    # ROS system share path (Jazzy)
    ros_dir = '/opt/ros/jazzy/share/'

    # Joystick Reader
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            # nothing I guess
        }]
    )

    # Joystick to Twist converter
    #joy2twist_share = get_package_share_directory('mecanum_joystick')
    #mappings = os.path.join(joy2twist_share, 'mappings.yaml')
    joy2twist = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        output="screen",
        parameters=[{
            "axis_linear.x": 1,
            "axis_linear.y": 0,
            "axis_angular.yaw": 3,
            "scale_linear.x": 1.0,
            "scale_linear.y": 1.0,
            "scale_angular.yaw": 1.0,
            # set buttons/enable as you like
            "enable_button": 6,
            "enable_turbo_button": 7,
            "require_enable_button": True,
        }]
        # package='mecanum_joystick',
        # executable='joy_to_twist',
        # name='joy_to_twist',
        # output='screen',
        # parameters=[mappings]
    )

    # Odometry node (executable name must match your setup.py entry point)
    odometry = Node(
        package='robot_slam',
        executable='odometry',   # <-- ensure 'odometry' is in setup.py entry_points
        name='odometry',
        output='screen',
    )

    # Serial connection (serial_driver executable must also be in setup.py entry points)
    serial_driver = Node(
        package="robot_slam",
        executable="serial_driver",  # <-- ensure this matches setup.py
        name="serial_driver",
        output="screen",
        parameters=[{
            "port": "/dev/ttyACM0",
            "baudrate": 115200,
            # "wheel_radius": 0.03,
            # "wheel_separation_x": 0.375,
            # "wheel_separation_y": 0.290,
            # "max_rpm": 10,
            # "send_rate_hz": 20.0
        }]
    )

    # RPLIDAR config file (ensure file exists at this path)
    # rplidar_config = os.path.join(
    #     get_package_share_directory('robot_slam'),
    #     'config',
    #     'rplidar_a1.yaml'
    # )

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True
        }],
    )

    # Static transform base_link -> laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_laser',
        output='screen',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']
    )

    # nav2 (bringup)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_dir, 'nav2_bringup', 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'False'}.items()
    )

    # SLAM toolbox params (make sure this exists)
    slam_toolbox_share = get_package_share_directory('robot_slam')  # <-- fixed
    slam_params_file = os.path.join(slam_toolbox_share, 'config', 'slam_toolbox.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_dir, 'slam_toolbox', 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'slam_params_file': slam_params_file,
            'queue_size': 10,
        }.items()
    )

    # RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(ros_dir, 'nav2_bringup', 'rviz', 'nav2_default_view.rviz')],
        output='screen'
    )

    return LaunchDescription([
        joy,
        joy2twist,
        odometry,
        serial_driver,
        rplidar,
        static_tf_base_to_laser,
        nav2,
        slam_toolbox,
        rviz2,
    ])


# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node


# def generate_launch_description():
#     """
#     Generates the launch description for the robot
#     """

#     # ROS Directory
#     ros_dir = '/opt/ros/jazzy/share/'

#     # Joystick Reader
#     joy = Node(
#         package='joy',
#         executable='joy_node',
#         name='joy_node',
#         output='screen',
#         parameters=[{
#             # 'dev': '/dev/input/js0',
#         }]
#     )

#     # Joystick to Twist converter
#     joy2twist_share = get_package_share_directory('ros2_joy_twist')
#     mappings = os.path.join(joy2twist_share, 'mappings.yaml')
#     joy2twist = Node(
#         package='ros2_joy_twist',
#         executable='joy_to_twist',
#         name='joy_to_twist',
#         output='screen',
#         parameters=[mappings],
#     )

#     # Odometry
#     odometry = Node(
#         package='robot_slam',
#         executable='odometry',
#         name='odometry',
#         output='screen',
#     )
#     # odometry = Node(
#     #     package='robot_slam',
#     #     executable='odometry',
#     #     name='mecanum_odometry',
#     #     output='screen',
#     #     parameters=[{
#     #         'wheel_radius': 0.33,
#     #         'wheel_seperation_x': 0.375,
#     #         'wheel_seperation_y': 0.290
#     #     }]
#     # )

#     #serial connection
#     serial_driver = Node(
#         package="robot_slam",
#         executable="serial_driver",
#         name="serial_driver",
#         output="screen",
#         parameters={
#             "port": "/dev/ttyACM0",
#             "baudrate": 115200,
#             "wheel_radius": 0.03,
#             "wheel_separation_x": 0.375,
#             "wheel_separation_y": 0.290,
#             "max_rpm": 10,
#             "send_rate_hz": 20.0
#         }    
#     )

#     rplidar_config = os.path.join(
#         get_package_share_directory('robot_slam'),
#         'config',
#         'rplidar_a1.yaml'
#     )

#     rplidar = Node(
#         package='rplidar_ros',
#         executable='rplidar_composition',
#         name='rplidar_composition',
#         output='screen',
#         parameters=[rplidar_config],
#     )

#     # # Lidar node
#     # rplidar = Node(
#     #     name='rplidar_composition',
#     #     package='rplidar_ros',
#     #     executable='rplidar_composition',
#     #     output='screen',
#     #     parameters=[{
#     #         'serial_port': '/dev/ttyUSB0',
#     #         'serial_baudrate': 115200,  # A1 / A2
#     #         # 'serial_baudrate': 256000, # A3
#     #         'frame_id': 'laser',
#     #         'inverted': False,
#     #         'angle_compensate': True
#     #     }],
#     # )

#     # Static transform base_link -> laser (adjust translation/rotation as needed)
#     static_tf_base_to_laser = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_base_to_laser',
#         output='screen',
#         arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']
#     )

#     # nav2
#     nav2 = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(ros_dir, 'nav2_bringup', 'launch', 'navigation_launch.py')
#         ),
#         launch_arguments={'use_sim_time': 'False'}.items()
#     )

#     # Slam_toolbox with explicit params
#     slam_toolbox_share = get_package_share_directory('robot')
#     slam_params_file = os.path.join(slam_toolbox_share, 'config', 'slam_toolbox.yaml')
#     slam_toolbox = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(ros_dir, 'slam_toolbox', 'launch', 'online_async_launch.py')
#         ),
#         launch_arguments={
#             'use_sim_time': 'False',
#             'slam_params_file': slam_params_file
#         }.items()
#     )

#     # Visualization
#     rviz2 = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         arguments=['-d', os.path.join(ros_dir, 'nav2_bringup', 'rviz', 'nav2_default_view.rviz')],
#         output='screen'
#     )

#     return LaunchDescription([
#         joy,
#         joy2twist,
#         odometry,
#         # motor_controller,
#         serial_driver,
#         rplidar,
#         static_tf_base_to_laser,
#         nav2,
#         slam_toolbox,
#         rviz2,
#     ])