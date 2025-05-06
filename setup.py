from setuptools import find_packages, setup

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
            'launch/bringup_lidar_nav.launch.py',
            'launch/realsense_camera.launch.py',
            'launch/rplidar.launch.py',
            'launch/visual_slam.launch.py',
            'launch/nvblox.launch.py',
            'launch/nav2.launch.py',
            'launch/rviz.launch.py', 
            'launch/point_cloud_xyz.launch.py',
            'launch/rviz_point_cloud_xyz.rviz'
            ]),
            
        ('share/' + package_name + '/params', ['params/nav2_params.yaml','params/nvblox_params.yaml', 'params/realsense_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ias',
    maintainer_email='ias@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
