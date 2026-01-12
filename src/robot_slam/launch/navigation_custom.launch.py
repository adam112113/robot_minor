import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Custom Nav2 launch - bypasses route_server dependency
    """

    # Paths
    pkg_robot_slam = get_package_share_directory('robot_slam')
    
    # Launch arguments
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_robot_slam, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_robot_slam, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Joystick Reader
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    # Joystick to Twist converter
    joy2twist = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        output='screen',
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
        output='screen',
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

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': map_yaml_file}
        ]
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    # Planner server  
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file]
    )

    # Collision Monitor
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file]
    )

    # Lifecycle manager for localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # Lifecycle manager for navigation (NO route_server!)
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': autostart},
                    {'node_names': ['controller_server',
                                    'planner_server',
                                    'behavior_server',
                                    'bt_navigator',
                                    'velocity_smoother',
                                    'collision_monitor']}]
    )

    # Click-to-navigate node (optional - can be launched separately)
    click_to_nav = Node(
        package='robot_slam',
        executable='nav2_pose',
        name='click_to_nav_goal',
        output='screen',
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        
        # Hardware and sensors
        joy,
        joy2twist,
        odometry,
        serial_driver,
        rplidar,
        static_tf_base_to_laser,
        
        # Nav2 Localization
        map_server,
        amcl,
        lifecycle_manager_localization,
        
        # Nav2 Navigation
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_navigation,
        
        # Helper nodes (commented out to reduce DDS participants)
        click_to_nav,
    ])
