import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Nav2 navigation with pre-saved map
    Use this AFTER you have created a map with robot_slam.launch.py
    """

    # Paths
    pkg_robot_slam = get_package_share_directory('robot_slam')
    nav2_bringup_dir = '/opt/ros/jazzy/share/nav2_bringup'
    
    # Launch arguments
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_robot_slam, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_robot_slam, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for Nav2'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )

    # Joystick Reader
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{}]
    )

    # Joystick to Twist converter
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
            "enable_button": 6,
            "enable_turbo_button": 7,
            "require_enable_button": False,
        }]
    )

    # Odometry node
    odometry = Node(
        package='robot_slam',
        executable='odometry',
        name='odometry',
        output='screen',
    )

    # Serial connection
    serial_driver = Node(
        package="robot_slam",
        executable="serial_driver",
        name="serial_driver",
        output="screen",
        parameters=[{
            "port": "/dev/ttyACM0",
            "baudrate": 115200,
        }],
    )

    # RPlidar A1
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': True,
            'angle_compensate': True,
        }],
        respawn=True,
        respawn_delay=2.0
    )

    # Static TF: base_link -> laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_laser',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.15',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ],
    )

    # Nav2 Bringup (localization + navigation)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'True',
            'use_respawn': 'False'
        }.items()
    )

    # Click-to-navigate node (your nav2_pose.py)
    click_to_nav = Node(
        package='robot_slam',
        executable='navigate_to_pose_client',
        name='navigate_to_pose_client',
        # executable='nav2_pose',
        # name='click_to_nav_goal',
        output='screen',
    )

    # NOTE: RViz runs on HOST machine, not on Pi5 (Ubuntu Server has no GUI)
    # Use launch_nav_rviz.sh on your host machine to start RViz

    return LaunchDescription([
        # Declare launch arguments
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        
        # Hardware and sensors
        joy,
        joy2twist,
        odometry,
        serial_driver,
        rplidar,
        static_tf_base_to_laser,
        
        # Nav2 stack
        nav2_bringup,
        
        # Helper nodes
        click_to_nav,
    ])
