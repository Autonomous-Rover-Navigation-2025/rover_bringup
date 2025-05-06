import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node, IncludeLaunchDescription
from launch_ros.actions import SetParameter, SetParametersFromFile, SetRemap
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

camera_parameters = [
    {"camera_name": "camera"},
    {"unite_imu_method": "2"},
    {"log_level": "info"},
    {"output": "screen"},
    {"depth_module.profile": "0,0,0"},
    {"enable_depth": True},
    {"rgb_camera.profile": "0,0,0"},
    {"enable_color": True},
    {"enable_infra1": True},
    {"enable_infra2": True},
    {"infra_rgb": False},
    {"tracking_module.profile": "0,0,0"},
    {"enable_fisheye1": True},
    {"enable_fisheye2": True},
    {"enable_confidence": True},
    {"gyro_fps": 200},
    {"accel_fps": 200},
    {"enable_gyro": True},
    {"enable_accel": True},
    {"enable_pose": True},
    {"pose_fps": 200},
    {"pointcloud.enable": False},
    {"pointcloud.stream_filter": 2},
    {"pointcloud.stream_index_filter": 0},
    {"enable_sync": False},
    {"align_depth.enable": False},
    {"colorizer.enable": False},
    {"clip_distance": -2.0},
    {"linear_accel_cov": 0.01},
    {"initial_reset": False},
    {"allow_no_texture_points": False},
    {"ordered_pc": False},
    {"tf_publish_rate": 0.0},
    {"diagnostics_period": 0.0},
    {"decimation_filter.enable": False},
    {"depth_module.exposure.1": 7500},
    {"depth_module.gain.1": 16},
    {"depth_module.exposure.2": 1},
    {"depth_module.gain.2": 16},
    {"wait_for_device_timeout": -1.0},
    {"reconnect_timeout": 6.0},
    {"infra1_module.profile": "640x480x30"},
    {"infra2_module.profile": "640x480x30"}
]

def generate_launch_description():
    rover_bringup_dir = get_package_share_directory('rover_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nvblox_config = os.path.join(rover_bringup_dir, 'params', 'nvblox_params.yaml')
    nav2_param_file = os.path.join(rover_bringup_dir, 'params', 'nav2_params.yaml')

    # RealSense Camera Node
    realsense_camera_node = Node(
        package='realsense2_camera',
        namespace='camera',  # Static camera namespace
        name='camera',        # Static node name
        executable='realsense2_camera_node',
        parameters=camera_parameters,    # Directly using the static parameters
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],  # Static log level
        emulate_tty=True,
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
    


    return LaunchDescription([
        realsense_camera_node,
        visual_slam_container,
        nvblox_container,
        nvblox_group
        ])
