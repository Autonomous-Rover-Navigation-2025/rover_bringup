import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rover_bringup_dir = get_package_share_directory('rover_bringup')
    realsense_config = os.path.join(rover_bringup_dir, 'params', 'realsense_params.yaml')

    # RealSense Camera Node
    realsense_camera_node = Node(
        package='realsense2_camera',
        namespace='camera',  # Static camera namespace
        name='camera',        # Static node name
        executable='realsense2_camera_node',
        parameters=[realsense_config],    # Directly using the static parameters
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],  # Static log level
        emulate_tty=True,
    )

    realsense_splitter_node = Node(
        namespace="camera",
        name="realsense_splitter_node",
        package="realsense_splitter",
        executable="realsense_splitter_node",
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA'
        }],
        remappings=[
            ('input/infra_1', '/camera/infra1/image_rect_raw'),
            ('input/infra_1_metadata', '/camera/infra1/metadata'),
            ('input/infra_2', '/camera/infra2/image_rect_raw'),
            ('input/infra_2_metadata', '/camera/infra2/metadata'),
            ('input/depth', '/camera/depth/image_rect_raw'),
            ('input/depth_metadata', '/camera/depth/metadata'),
            ('input/pointcloud', '/camera/depth/color/points'),
            ('input/pointcloud_metadata', '/camera/depth/metadata')
        ],
        output="screen",  # Print logs to the screen
    )
    return LaunchDescription([
        realsense_camera_node,
        realsense_splitter_node,
        ])
