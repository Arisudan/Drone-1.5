import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
        default_value='0.05',
        description='Occupancy grid cell resolution (meters)'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    # Step 1: Launch D435i Camera Node
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'd435i_stereo_imu.launch.py')
        )
    )

    # Step 2: Launch Stereo Odometry Node (delay 3 seconds for camera stream init)
    odom_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'stereo_inertial_odom.launch.py')
                )
            )
        ]
    )

    # Step 3: Launch RTAB-Map SLAM Node (delay 6 seconds for odom init)
    slam_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'rtabmap_slam.launch.py')
                )
            )
        ]
    )

    # Step 4: Launch RViz2 (delay 8 seconds)
    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        min_obstacle_height_arg,
        max_obstacle_height_arg,
        cell_size_arg,
        launch_rviz_arg,
        camera_launch,
        odom_launch,
        slam_launch,
        rviz_node,
    ])
