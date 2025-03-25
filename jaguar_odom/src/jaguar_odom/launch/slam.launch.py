#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                'src/sllidar_ros2/launch/view_sllidar_a2m12_launch.py'
            )
        ),
        # Odometry | Node 
        Node(
            package='jaguar_odom',
            executable='encoder_odom_node',
            name='odom_node'
        ),
        # Static transform | Publisher | LiDAR 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_footprint', 'laser'],
            name='lidar_tf_publisher'
        ),
        # SLAM | Toolbox | Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'slam_mode': True,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'odom_topic': '/odom',
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'minimum_time_interval': 1.0,
                'transform_publish_period': 0.1,
                'message_filter_queue_size': 200000,
                'load_state_filename': '',
                'map_file_name': '',
                'enable_interactive_mode': False
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
