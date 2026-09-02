import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='',
            parameters=[{
                'initial_reset': True,
                'tf_prefix': '',
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': False,
                'enable_depth': False,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 2,  # 2 = linear_interpolation
                'depth_module.emitter_enabled': 0,  # 0 = OFF (no dot projector)
                'depth_module.infra_profile': '640x480x30',
                'gyro_fps': 200,
                'accel_fps': 200,
                'publish_tf': True,
                'base_frame_id': 'camera_link',
            }],
            output='screen'
        )
    ])
