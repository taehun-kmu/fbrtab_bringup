import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    fbrtab_bringup_pkg = get_package_share_directory('fbrtab_bringup')

    # Realsense
    realsense_config = os.path.join(
        fbrtab_bringup_pkg,
        'config',
        'realsense.yaml',
    )
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py',
            )
        ),
        launch_arguments={
            'config_file': realsense_config,
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

    # Static transform from robot base to camera optical base
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.02',  # x offset (2 cm forward)
            '0.0',   # y offset
            '0.0',   # z offset
            '0.0',   # roll
            '0.0',   # pitch
            '0.0',   # yaw
            'base_link',
            'camera_link',
        ],
    )

    # Toggle RTAB-Map visualization UI
    rtabmap_viz_argument = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='true',
        description='Enable the RTAB-Map visualization interface.',
    )

    # RTAB-Map
    rtabmap_config = os.path.join(fbrtab_bringup_pkg, 'config', 'rtabmap.yaml')
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py',
            )
        ),
        launch_arguments={
            'args': '-d',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            'wait_imu_to_init': 'true',
            'imu_topic': '/rtabmap/imu',
            'config_path': rtabmap_config,
            'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
        }.items()
    )

    return LaunchDescription([
        rtabmap_viz_argument,
        realsense_launch,
        imu_filter_node,
        static_tf_node,
        rtabmap_launch
    ])
