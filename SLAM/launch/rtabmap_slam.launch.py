import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rtabmap_node = Node(
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

            # Core 2D Occupancy Grid Parameters (Ultra-Fine 2.5 cm Grid)
            'Grid/FromDepth': 'false',          # Generate grid from stereo point cloud
            'Grid/RayTracing': 'true',          # Mark free space using ray tracing
            'Grid/3D': 'true',                  # Build 3D OctoMap / 2D projection
            'Grid/CellSize': '0.025',           # 2.5 cm ultra-fine cell resolution
            'Grid/MinObstacleHeight': '0.30',    # Filters out low floor furniture/baseboards (<30cm)
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/NormalsSegmentation': 'true', # Filter out ground noise (vertical walls only)
            'Grid/DepthDecimation': '2',        # Depth decimation factor 2 (retains sharp wall corners)
            'Grid/FlatObstacleHandledAsGround': 'true',
            'GridGlobal/OccupancyThr': '0.65',  # High confidence threshold (>65%)

            # Drone Self-Footprint Clearing
            'Grid/FootprintLength': '0.4',
            'Grid/FootprintWidth': '0.4',

            # Sensor Range Filtering (D435i accuracy optimization)
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '3.5',

            # Surface & Normal Filtering
            'Grid/MaxGroundAngle': '30.0',
            'Grid/NormalK': '20',

            # Tight Noise & Cluster Filtering
            'Grid/NoiseFilteringRadius': '0.05',
            'Grid/NoiseFilteringMinNeighbors': '8',
            'Grid/ClusterRadius': '0.1',
            'Grid/MinClusterSize': '10',

            # Bayesian Occupancy Probability Tuning
            'Grid/ProbHit': '0.75',
            'Grid/ProbMiss': '0.35',
            'Grid/ProbClampingMax': '0.99',
            'Grid/Scan2dUnknownSpaceFilled': 'true',

            # Global Loop Closure Map Correction
            'Grid/GlobalFullUpdate': 'true',
            'Optimizer/Strategy': '1',

            # SLAM Registration Tuning
            'Reg/Force3DoF': 'false',
            'Reg/Strategy': '0',
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

    # ROS2 Map Thinning Node (Generates 1-pixel wide /map_thin topic)
    thinning_node = Node(
        package='rtabmap_drone_pkg',
        executable='map_thinning_node.py',
        name='map_thinning_node',
        output='screen',
        parameters=[{
            'occupancy_threshold': 60,
            'min_wall_cluster_size': 20,         # Discards all scattered noise specks outside room bounds
        }]
    )

    return LaunchDescription([
        rtabmap_node,
        thinning_node,
    ])
