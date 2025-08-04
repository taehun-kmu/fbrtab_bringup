import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    fbrtab_bringup_pkg = get_package_share_directory('fbrtab_bringup')

    # Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            "initial_reset": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "rgb_camera.color_profile": "848,480,60",
            "depth_module.profile": "848,480,60",
            "publish_tf": "true",
        }.items(),
    )

    # IMU Filter
    imu_filter_config = os.path.join(fbrtab_bringup_pkg, 'config', 'imu_filter.yaml')
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[imu_filter_config],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'),
            ('/imu/data', '/rtabmap/imu')
        ]
    )

    # RTAB-Map
    rtabmap_config = os.path.join(fbrtab_bringup_pkg, 'config', 'rtabmap.yaml')
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'args': '-d',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'camera_link',
            'wait_imu_to_init': 'true',
            'imu_topic': '/rtabmap/imu',
            'config_path': rtabmap_config
        }.items()
    )

    return LaunchDescription([
        realsense_launch,
        imu_filter_node,
        rtabmap_launch
    ])
