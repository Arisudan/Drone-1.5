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
            respawn=True,           # Auto-restarts node if it exits or crashes
            respawn_delay=5.0,      # 5-second non-blocking delay (prevents USB kernel lockup)
            parameters=[{
                # True (was False): forces a full hardware reset of the D435i on every
                # node startup. Confirmed live this session: the camera degrades to
                # near-zero stereo matches after hours of continuous operation and heavy
                # USB handling nearby, recoverable only via a MANUAL physical unplug/
                # replug - this makes that reset happen automatically every launch
                # instead, matching the upstream reference (github.com/Arisudan/
                # Drone-1.5/SLAM), which never disabled it.
                'initial_reset': True,
                'reconnect_timeout': 6.0,
                'wait_for_device_timeout': 10.0,
                'tf_prefix': '',
                'enable_sync': True,
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

                # RealSense Option 2 Filters: Disparity & Hole Filling for Glass/Smooth Surfaces
                'disparity_filter.enable': True,
                'spatial_filter.enable': True,
                'temporal_filter.enable': True,
                'hole_filling_filter.enable': True,
            }],
            output='screen'
        )
    ])
