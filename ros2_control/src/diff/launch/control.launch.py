from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_ekf = LaunchConfiguration('use_ekf', default='false')
    world_name = LaunchConfiguration('world_name', default='depot.sdf')

    pkg_megabot = FindPackageShare('diff')
    
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_megabot, 'urdf', 'mws_control.urdf']),
        ' use_sim_time:=', use_sim_time
    ])
    
    robot_description_param = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': use_sim_time
    }

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param],
        output='screen'
    )

    ros2_control_hw_node = Node(
        condition=UnlessCondition(use_sim_time),
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_param, PathJoinSubstitution([pkg_megabot, 'config', 'controllers.yaml'])],
        remappings=[
            ('/diffbot_base_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/diffbot_base_controller/odom', '/odom_unfiltered'), 
        ],
        output='screen'
    )

    gz_ros2_control_node = Node(
        condition=IfCondition(use_sim_time),
        package='gz_ros2_control',
        executable='gz_ros2_control',
        parameters=[robot_description_param, PathJoinSubstitution([pkg_megabot, 'config', 'controllers.yaml'])],
        output='screen'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        condition=IfCondition(use_sim_time),
        launch_arguments={'gz_args': world_name}.items()
    )

    joint_state_spawner = TimerAction(period=3.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen'
    )])

    diff_drive_spawner = TimerAction(period=4.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['diffbot_base_controller', '--controller-manager', '/controller_manager'], output='screen'
    )])

    imu_spawner = TimerAction(period=5.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['imu_broadcaster', '--controller-manager', '/controller_manager'], output='screen'
    )])

    # MADGWICK FILTER NODE

    # madgwick_node = Node(
    #     condition=UnlessCondition(use_sim_time),
    #     package='imu_filter_madgwick',
    #     executable='imu_filter_madgwick_node',  # ← changed
    #     name='imu_filter_madgwick',
    #     parameters=[
    #         PathJoinSubstitution([pkg_megabot, 'config', 'madgwick.yaml']),
    #         {'use_sim_time': use_sim_time}
    #     ],
    #     remappings=[
    #         ('/imu/data_raw', '/imu_broadcaster/imu'),
    #         ('/imu/data', '/imu/data'),
    #     ],
    #     output='screen'
    # )

    # EKF NODE
    ekf_node = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            PathJoinSubstitution([pkg_megabot, 'config', 'ekf.yaml']),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/imu/data', '/imu_broadcaster/imu'),   
        ],
        output='screen'
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_megabot, 'rviz', 'mws.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}], output='screen'
    )

    spawn_robot_node = TimerAction(period=2.0, actions=[Node(
        condition=IfCondition(use_sim_time), package='ros_gz_sim', executable='create',
        arguments=['-name', 'diff_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.2', '-Y', '0.0'],
        output='screen'
    )])

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_ekf', default_value='false', description='Enable EKF sensor fusion'),
        DeclareLaunchArgument('world_name', default_value='empty.sdf', description='Gazebo world file'),

        rsp_node, ros2_control_hw_node, gz_ros2_control_node,
        gazebo_launch, spawn_robot_node,
        joint_state_spawner, diff_drive_spawner, imu_spawner,
        # madgwick_node, 
        ekf_node, rviz_node,
    ])

    return ld