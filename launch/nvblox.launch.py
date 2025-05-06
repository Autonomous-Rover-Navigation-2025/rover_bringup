import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.actions import SetParameter, SetParametersFromFile, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rover_bringup_dir = get_package_share_directory('rover_bringup')

    # Config file
    base_config = os.path.join(rover_bringup_dir, 'params', 'nvblox_params.yaml')

    # Composable node container for managing composable nodes
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )
    
    load_composable_nodes = LoadComposableNodes(
        target_container=nvblox_container,
        composable_node_descriptions=[
            ComposableNode(
                name='nvblox_node',
                package='nvblox_ros',
                plugin='nvblox::NvbloxNode')
        ])

    group_action = GroupAction([

        # Set parameters with specializations
        SetParametersFromFile(base_config),
        SetParameter(name='global_frame', value=LaunchConfiguration('global_frame', default='odom')),

        # Remappings for realsense data
        SetRemap(src=['depth/image'], dst=['/camera/realsense_splitter_node/output/depth']),
        SetRemap(src=['depth/camera_info'], dst=['/camera/depth/camera_info']),
        SetRemap(src=['color/image'], dst=['/camera/color/image_raw']),
        SetRemap(src=['color/camera_info'], dst=['/camera/color/camera_info']),

        # Include the node container
        load_composable_nodes
    ])

    return LaunchDescription([nvblox_container, group_action])
