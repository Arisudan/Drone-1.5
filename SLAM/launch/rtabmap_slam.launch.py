import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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

                # SLAM & 2D Occupancy Grid Parameters for Indoor Drone
                'Grid/FromDepth': 'false',          # Generate grid from stereo point cloud
                'Grid/RayTracing': 'true',          # Mark free space using ray tracing
                'Grid/3D': 'true',                  # Build 3D OctoMap / 2D projection
                'Grid/CellSize': '0.05',
                'Grid/MinObstacleHeight': '0.1',
                'Grid/MaxObstacleHeight': '2.0',
                'Grid/NoiseFilteringRadius': '0.1',
                'Grid/NoiseFilteringMinNeighbors': '5',
                'Reg/Force3DoF': 'false',           # 3D motion model for drone
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
