import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Realsense
    realsense2_camera_pkg = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense2_camera_pkg, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'initial_reset': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
            'rgb_camera.color_profile': "'848,480,60'",
            'align_depth.enable': 'true',
            'publish_tf': 'true'
        }.items()
    )

    # IMU Filter
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[
            {'use_mag': False},
            {'publish_tf': False}
        ],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'),
            ('/imu/data', '/rtabmap/imu')
        ]
    )

    # RTAB-Map
    rtabmap_parameters = {
        'Rtabmap/StartMapOnLoopClosure': 'true',
        'Odom/Strategy': '1',
        'Odom/FeatureType': '6',
        'Odom/MinInliers': '15',
        'GFTT/MinDistance': '7',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/LoopClosureReextractFeatures': 'true',
        'Vis/MinInliers': '15',
        'Grid/RangeMax': '4.0'
    }
    rtabmap_launch_file = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    )
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file),
        launch_arguments={
            'args': '-d',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'camera_link',
            'wait_imu_to_init': 'true',
            'imu_topic': '/rtabmap/imu',
            'parameters': str(rtabmap_parameters)
        }.items()
    )

    return LaunchDescription([
        realsense_launch,
        imu_filter_node,
        rtabmap_launch
    ])
