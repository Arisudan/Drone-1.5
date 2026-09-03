import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    range_max_arg = DeclareLaunchArgument(
        'range_max',
        default_value='3.5',
        description='Max range for grid ray tracing (meters)'
    )
    min_obstacle_height_arg = DeclareLaunchArgument(
        'min_obstacle_height',
        default_value='0.1',
        description='Minimum obstacle height for 2D grid map (meters)'
    )
    max_obstacle_height_arg = DeclareLaunchArgument(
        'max_obstacle_height',
        default_value='2.0',
        description='Maximum obstacle height for 2D grid map (meters)'
    )
    cell_size_arg = DeclareLaunchArgument(
        'cell_size',
        default_value='0.05',
        description='Occupancy grid cell resolution (meters)'
    )

    return LaunchDescription([
        range_max_arg,
        min_obstacle_height_arg,
        max_obstacle_height_arg,
        cell_size_arg,
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
                'Grid/CellSize': LaunchConfiguration('cell_size'),
                'Grid/RangeMax': LaunchConfiguration('range_max'),
                'Grid/MinObstacleHeight': LaunchConfiguration('min_obstacle_height'),
                'Grid/MaxObstacleHeight': LaunchConfiguration('max_obstacle_height'),
                'Grid/NoiseFilteringRadius': '0.05',
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
