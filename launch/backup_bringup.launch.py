import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.actions import SetParameter, SetParametersFromFile, SetRemap
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    rover_bringup_dir = get_package_share_directory('rover_bringup')

    # ========== RealSense ==========
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

    # VSLAM Node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera_link',
                    'input_imu_frame': 'camera_gyro_optical_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 22.00,
                    'frame_delta_threshold': 100.0
                    }],
        remappings=[('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu'),
                    ('visual_slam/tracking/odometry', '/odom')]
    )    

    # VSLAM container
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    # ========== Nvblox ==========
    nvblox_config = os.path.join(rover_bringup_dir, 'params', 'nvblox_params.yaml')

    # Nvblox container
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )
    
    # Nvblox node loader
    nvblox_loader = LoadComposableNodes(
        target_container=nvblox_container,
        composable_node_descriptions=[
            ComposableNode(
                name='nvblox_node',
                package='nvblox_ros',
                plugin='nvblox::NvbloxNode')
        ])

    nvblox_group = GroupAction([

        # Set parameters with specializations
        SetParametersFromFile(nvblox_config),
        SetParameter(name='global_frame', value=LaunchConfiguration('global_frame', default='odom')),

        # Remappings for realsense data
        SetRemap(src=['depth/image'], dst=['/camera/realsense_splitter_node/output/depth']),
        SetRemap(src=['depth/camera_info'], dst=['/camera/depth/camera_info']),
        SetRemap(src=['color/image'], dst=['/camera/color/image_raw']),
        SetRemap(src=['color/camera_info'], dst=['/camera/color/camera_info']),

        # Include nvblox container
        nvblox_loader
    ])

    # ========== Nav2 ==========
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_config = os.path.join(rover_bringup_dir, 'params', 'nav2_params.yaml')

    # Override the global_frame for all of the nav2 nodes
    param_substitutions = {
        'global_frame': LaunchConfiguration('global_frame', default='odom')}
    
    configured_params = RewrittenYaml(
            source_file=nav2_config,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    # Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'False',
                          'params_file': configured_params,
                          'autostart': 'True'}.items())


    return LaunchDescription([
        realsense_camera_node,
        realsense_splitter_node,
        visual_slam_container,
        nvblox_container,
        nvblox_group,
        nav2_launch
        ])
