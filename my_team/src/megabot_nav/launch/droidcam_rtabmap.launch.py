# Launches RTAB-Map RGB-D SLAM with planar 2D mapping, depth-based obstacle/grid generation, IMU/odometry fusion, loop closure, visualization, and point-cloud obstacle detection, with SLAM/localization modes.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'imu_frame_id': 'imu',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'use_action_for_goal': True,
        'wait_imu_to_init': False,

        # planar SLAM, g2o backend
        'Reg/Force3DoF': 'true',
        'Optimizer/Slam2D': 'true',
        'Optimizer/Robust': 'true',
        'Optimizer/Strategy': '1',
        'g2o/Optimizer': '0',
        'g2o/Solver': '0',

        # 2D occupancy grid from depth, not point cloud
        'Grid/3D': 'false',
        'Grid/Sensor': '1',
        'Grid/FromDepth': 'true',
        'Grid/RayTracing': 'true',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.20',
        'Grid/RangeMax': '12.0',
        'Grid/NoiseFilteringRadius': '0.03',
        'Grid/NoiseFilteringMinNeighbors': '3',

        # ORB features, used for both tracking and loop closure
        'Kp/MinDistance': '3',
        'Kp/MaxFeatures': '1500',
        'Kp/SubPix': 'true',
        'Kp/DetectorStrategy': '3',
        'Vis/FeatureType': '3',
        'Vis/MinInliers': '10',
        'Vis/CorType': '0',
        'RGBD/LoopClosureReextractFeatures': 'true',
        'RGBD/LoopThr': '0.3',
        'RGBD/OptimizeMaxError': '10.0',
        'RGBD/DepthMin': '0.20',
        'RGBD/DepthMax': '12.0',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '5',
        'RGBD/ProximityPathFilteringRadius': '0.5',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.1',

        'Rtabmap/DetectionRate': '0',
        'Rtabmap/StartNewMapOnLoopClosure': 'false',

        'approx_sync': True,  
        'approx_sync_max_interval': 0.05,
        'sync_queue_size': 30,
        'Mem/DepthCompressionFormat': '.png',
    }

    # obstacles_detection only wants Grid/* params, passing the rest triggers ROS 2 warnings
    grid_parameters = {
        k: v for k, v in parameters.items()
        if k.startswith('Grid/') or k in ('frame_id', 'use_sim_time')
    }

    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('imu', '/imu_broadcaster/imu'),
        ('odom', '/odom'),
        ('grid_map', '/map'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),

        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d'],
        ),

        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters, {
                'Mem/IncrementalMemory': False,
                'Mem/InitWMWithAllNodes': True,
            }],
            remappings=remappings,
        ),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters],
            remappings=remappings,
        ),

        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{
                'decimation': 2,
                'max_depth': 3.0,
                'voxel_size': 0.02,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('depth/image', '/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('cloud', '/camera/cloud'),
                ('imu', '/imu_broadcaster/imu'),
                ('odom', '/odom'),
            ],
        ),

        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[grid_parameters],
            remappings=[
                ('cloud', '/camera/cloud'),
                ('obstacles', '/camera/obstacles'),
                ('ground', '/camera/ground'),
            ],
        ),
    ])