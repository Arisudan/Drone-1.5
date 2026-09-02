import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_camera_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'subscribe_stereo': True,
                'subscribe_imu': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.02,
                'wait_imu_to_init': True,
                'queue_size': 30,
                'Odom/Strategy': '0',        # 0=Frame-to-Map
                'Odom/FeatureType': '8',     # 8=ORB / 6=GFTT
                'Odom/FillInfoData': 'true',
                'Vis/MaxDepth': '8.0',
                'Vis/MinInliers': '10',
            }],
            remappings=[
                ('left/image_rect', '/camera/infra1/image_rect_raw'),
                ('left/camera_info', '/camera/infra1/camera_info'),
                ('right/image_rect', '/camera/infra2/image_rect_raw'),
                ('right/camera_info', '/camera/infra2/camera_info'),
                ('imu', '/camera/imu'),
                ('odom', '/odom'),
            ]
        )
    ])
