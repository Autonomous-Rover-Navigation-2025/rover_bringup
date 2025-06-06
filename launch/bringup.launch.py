import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    rover_bringup_dir = get_package_share_directory('rover_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(rover_bringup_dir, 'launch', 'realsense_camera.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(rover_bringup_dir, 'launch', 'visual_slam.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(rover_bringup_dir, 'launch', 'nvblox.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(rover_bringup_dir, 'launch', 'nav2.launch.py'))),
    ])