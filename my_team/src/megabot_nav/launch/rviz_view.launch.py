import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    open_rviz = LaunchConfiguration('open_rviz', default='true')
    rviz_config_dir = os.path.join(
        get_package_share_directory('megabot_nav'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'open_rviz',
            default_value='true',
            description='Launch Rviz?'),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_to_odom_broadcaster',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'odom'],
        #     output='screen'
        # ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_footprint_to_map_broadcaster',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
        #     output='screen'
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
        )
    ])
