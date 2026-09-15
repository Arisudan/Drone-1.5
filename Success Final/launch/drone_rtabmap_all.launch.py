import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rtabmap_drone_pkg')
    rviz_config_path = os.path.join(pkg_share, 'config', 'rtabmap_drone.rviz')

    # Launch arguments
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
        default_value='0.025',
        description='Occupancy grid cell resolution (meters)'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Whether to launch RViz2 (default false for headless flying Radxa; laptop renders locally)'
    )
    enable_tcp_map_streamer_arg = DeclareLaunchArgument(
        'enable_tcp_map_streamer',
        default_value='true',
        description='Whether to run TCP map streaming fallback bridge on port 5765 for Wi-Fi GCS'
    )
    enable_video_streamer_arg = DeclareLaunchArgument(
        'enable_video_streamer',
        default_value='true',
        description='Whether to run the D435i JPEG/MJPEG FPV video streamer on port 8080 for Wi-Fi GCS'
    )
    enable_px4_bridge_arg = DeclareLaunchArgument(
        'enable_px4_bridge',
        default_value='true',
        description='Whether to stream external vision pose to Pixhawk MAVLink'
    )
    pixhawk_device_arg = DeclareLaunchArgument(
        'pixhawk_device',
        default_value='tcp:127.0.0.1:5760',
        description='Serial port device path or mavlink-router TCP endpoint connected to Pixhawk MAVLink'
    )
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='921600',
        description='Serial port baudrate for MAVLink connection'
    )

    # Step 1: Launch D435i Camera Node
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'd435i_stereo_imu.launch.py')
        )
    )

    # Step 1b: Launch D435i RGB Video Streamer (delay 2 seconds for camera color stream init)
    # Independent of the odom/SLAM timing chain below - FPV video has no dependency on VIO.
    video_streamer_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rtabmap_drone_pkg',
                executable='d435i_video_streamer.py',
                name='d435i_video_streamer',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                parameters=[{
                    'port': 8080,
                    'jpeg_quality': 80,
                    'topic': '/camera/color/image_raw',
                }],
                condition=IfCondition(LaunchConfiguration('enable_video_streamer'))
            )
        ]
    )

    # Step 2: Launch Stereo Odometry Node (delay 5 seconds for camera stream & TF init)
    odom_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'stereo_inertial_odom.launch.py')
                )
            )
        ]
    )

    # Step 3: Launch PX4 Vision Bridge Node (delay 6 seconds for odom stream init)
    px4_bridge_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rtabmap_drone_pkg',
                executable='px4_vision_bridge.py',
                name='px4_vision_bridge',
                output='screen',
                # respawn: this node holds a live TCP connection to mavlink-router.
                # Restarting that service (needed after e.g. a USB re-enumeration -
                # see README's Development Session Log) drops the connection out from
                # under it. Without respawn, ros2 launch's default behaviour if this
                # node ever exits unexpectedly is to tear down the ENTIRE pipeline -
                # confirmed live: a mavlink-router restart took camera/SLAM/everything
                # down with it, not just this bridge. respawn keeps the blast radius
                # to just this one node.
                respawn=True,
                respawn_delay=2.0,
                parameters=[{
                    'device': LaunchConfiguration('pixhawk_device'),
                    'baud': LaunchConfiguration('baud'),
                    'odom_topic': '/odom',
                    'use_ned_conversion': True
                }],
                condition=IfCondition(LaunchConfiguration('enable_px4_bridge'))
            )
        ]
    )

    # Step 4: Launch RTAB-Map SLAM Node (delay 8 seconds for odom init)
    slam_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'rtabmap_slam.launch.py')
                )
            )
        ]
    )

    # Step 5: Launch Map Thinning & Noise Purging Node (delay 9 seconds)
    map_thinning_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='rtabmap_drone_pkg',
                executable='map_thinning_node.py',
                name='map_thinning_node',
                output='screen',
                parameters=[{
                    'occupancy_threshold': 60,
                    'min_wall_area_pixels': 20,  # Discards all scattered noise specks outside room bounds
                }]
            )
        ]
    )

    # Step 6: Launch Wall Boundary Extractor Node (delay 9.5 seconds)
    wall_boundary_node = TimerAction(
        period=9.5,
        actions=[
            Node(
                package='rtabmap_drone_pkg',
                executable='wall_boundary_node.py',
                name='wall_boundary_node',
                output='screen'
            )
        ]
    )

    # Step 7: Launch TCP Map Streamer Node for Wi-Fi GCS Fallback (delay 9.8 seconds)
    tcp_map_streamer_node = TimerAction(
        period=9.8,
        actions=[
            Node(
                package='rtabmap_drone_pkg',
                executable='tcp_map_streamer_node.py',
                name='tcp_map_streamer_node',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=IfCondition(LaunchConfiguration('enable_tcp_map_streamer'))
            )
        ]
    )

    # Step 8: Launch RViz2 (delay 10 seconds, disabled by default on drone)
    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen',
                condition=IfCondition(LaunchConfiguration('launch_rviz'))
            )
        ]
    )

    return LaunchDescription([
        min_obstacle_height_arg,
        max_obstacle_height_arg,
        cell_size_arg,
        launch_rviz_arg,
        enable_tcp_map_streamer_arg,
        enable_video_streamer_arg,
        enable_px4_bridge_arg,
        pixhawk_device_arg,
        baud_arg,
        camera_launch,
        video_streamer_node,
        odom_launch,
        px4_bridge_node,
        slam_launch,
        map_thinning_node,
        wall_boundary_node,
        tcp_map_streamer_node,
        rviz_node,
    ])

