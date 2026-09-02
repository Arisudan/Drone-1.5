import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    min_obstacle_height_arg = DeclareLaunchArgument(
        'min_obstacle_height',
        default_value='0.2',
        description='Minimum obstacle height for 2D grid map (meters)'
    )
    max_obstacle_height_arg = DeclareLaunchArgument(
        'max_obstacle_height',
        default_value='1.8',
        description='Maximum obstacle height for 2D grid map (meters)'
    )
    cell_size_arg = DeclareLaunchArgument(
        'cell_size',
        default_value='0.05',
        description='Occupancy grid cell resolution (meters)'
    )
    range_min_arg = DeclareLaunchArgument(
        'range_min',
        default_value='0.4',
        description='Minimum depth range sensor cutoff (meters)'
    )
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='3.3',
        description='Maximum depth range sensor cutoff (meters to eliminate D435i far noise)'
    )

    return LaunchDescription([
        min_obstacle_height_arg,
        max_obstacle_height_arg,
        cell_size_arg,
        range_min_arg,
        range_max_arg,
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_camera_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'subscribe_stereo': True,
                'subscribe_odom_info': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.02,
                'sync_queue_size': 30,

                # High-Quality 2D Occupancy Grid Parameters (Filtered & Sparsified for Drone SLAM)
                'Grid/FromDepth': 'false',          # Generate 3D point cloud grid from stereo
                'Grid/RayTracing': 'true',          # Ray tracing to mark free space cleanly
                'Grid/3D': 'true',                  # OctoMap 3D filtering -> 2D projection
                'Grid/CellSize': LaunchConfiguration('cell_size'),
                'Grid/MinObstacleHeight': LaunchConfiguration('min_obstacle_height'),
                'Grid/MaxObstacleHeight': LaunchConfiguration('max_obstacle_height'),
                'Grid/RangeMin': LaunchConfiguration('range_min'),
                'Grid/RangeMax': LaunchConfiguration('range_max'),

                # Map Sparsification & Point Cloud Voxelization Parameters
                'Grid/VoxelSize': '0.08',                  # 8 cm 3D voxel downsampling for sparse uniform grid
                'Grid/DepthDecimation': '2',               # 2x decimation to sparsify stereo input cloud
                'Grid/Scan2dUnknownSpaceFilled': 'false',   # Do NOT fill unknown space; keep map sparse & clean
                'Grid/MapCleanup': 'true',                 # Clean up outdated / unobserved grid nodes

                # Strict Noise Filtering & Surface Normal Segmentation
                'Grid/NoiseFilteringRadius': '0.15',       # 15 cm radius sphere for noise check
                'Grid/NoiseFilteringMinNeighbors': '8',    # Require 8 points inside sphere to keep
                'Grid/ClusterRadius': '0.1',                # Cluster points to reject single noise specks
                'Grid/MaxGroundAngle': '30.0',             # Surfaces steeper than 30 deg are obstacles
                'Grid/NormalsSegmentation': 'true',        # Separate ground/floor plane from vertical walls
                'Grid/FlatObstacleDetected': 'false',      # Do not treat flat floors/rugs as obstacles

                'Reg/Force3DoF': 'false',           # Full 6-DOF drone motion model
                'Reg/Strategy': '0',                # Visual registration
                'Rtabmap/DetectionRate': '2.0',
                'Mem/IncrementalMemory': 'true',
            }],
            arguments=['--delete_db_on_start'],
            remappings=[
                ('left/image_rect', '/camera/infra1/image_rect_raw'),
                ('left/camera_info', '/camera/infra1/camera_info'),
                ('right/image_rect', '/camera/infra2/image_rect_raw'),
                ('right/camera_info', '/camera/infra2/camera_info'),
                ('odom', '/odom'),
                ('map', '/map'),
            ]
        )
    ])
