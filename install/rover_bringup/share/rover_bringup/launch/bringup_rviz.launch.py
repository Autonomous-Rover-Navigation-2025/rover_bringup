import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    config_name = LaunchConfiguration('config_name', default='realsense_example.rviz')
    global_frame = LaunchConfiguration('global_frame', default='odom')

    # Full path to the RViz config file
    config_path = PathJoinSubstitution([
        get_package_share_directory('nvblox_examples_bringup'),
        'config', 'rviz', config_name
    ])

    # RViz node with config and frame
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_path, '-f', global_frame],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
