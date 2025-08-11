#!/usr/bin/env python3
"""
ROS2 Composable Node Launch File for Visual-Inertial SLAM System

This launch file uses ROS2 composable nodes to reduce CPU load through intra-process
communication and zero-copy message passing. All sensor processing nodes run in a
single process, significantly improving performance.

Components:
- Intel RealSense D455 camera driver
- IMU filter (Madgwick algorithm)
- RTAB-Map synchronization and visual-inertial odometry
- RTAB-Map SLAM (runs as separate process with rtabmap_viz)

Benefits over traditional launch:
- 20-40% reduction in CPU usage
- Zero-copy image/pointcloud transfer
- Lower latency between components
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Create the component_container
    container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[],
        output='screen',
        parameters=[{'use_intra_process_comms': True}]
    )

    # RealSense Camera Component
    realsense_component = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='camera',
        namespace='/camera',
        parameters=[{
            'initial_reset': True,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,
            'enable_sync': True,
            'align_depth.enable': True,
            'rgb_camera.color_profile': '848,480,60',
            'depth_module.profile': '848,480,60',
            'pointcloud_enable': True,
            'publish_tf': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_confidence': False,
        }],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # IMU Filter Component
    imu_filter_component = ComposableNode(
        package='imu_filter_madgwick',
        plugin='ImuFilterMadgwickRos',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu'
        }],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'),
            ('/imu/data', '/rtabmap/imu')
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # RTAB-Map Sync Component
    rtabmap_sync_component = ComposableNode(
        package='rtabmap_sync',
        plugin='rtabmap_sync::RGBDSync',
        name='rgbd_sync',
        namespace='/rtabmap',
        parameters=[{
            'approx_sync': False,
            'queue_size': 10,
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('rgbd_image', '/rtabmap/rgbd_image'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # RTAB-Map Visual Odometry Component
    rtabmap_odom_component = ComposableNode(
        package='rtabmap_odom',
        plugin='rtabmap_odom::RGBDOdometry',
        name='rgbd_odometry',
        namespace='/rtabmap',
        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'wait_imu_to_init': True,
            'publish_tf': True,
            'subscribe_rgbd': True,
            'Odom/Strategy': '1',
            'Odom/FeatureType': '6',
            'Odom/MinInliers': '15',
            'GFTT/MinDistance': '7',
            'Vis/MinInliers': '15',
        }],
        remappings=[
            ('rgbd_image', '/rtabmap/rgbd_image'),
            ('imu', '/rtabmap/imu'),
            ('odom', '/rtabmap/odom'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Load components into the container
    load_components = LoadComposableNodes(
        composable_node_descriptions=[
            realsense_component,
            imu_filter_component,
            rtabmap_sync_component,
            rtabmap_odom_component,
        ],
        target_container=container
    )

    # RTAB-Map parameters for the main SLAM node
    rtabmap_params = {
        'frame_id': 'camera_link',
        'subscribe_depth': 'true',
        'subscribe_rgb': 'true',
        'subscribe_imu': 'true',
        'approx_sync': 'false',
        'wait_imu_to_init': 'true',

        'RGBD/ProximityBySpace': 'true',
        'RGBD/LoopClosureReextractFeatures': 'true',
        'Grid/RangeMax': '4.0',
    }

    # RTAB-Map main SLAM node (still using regular launch as it's complex)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')
        ]),
        launch_arguments={
            'args': '-d',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/rtabmap/imu',
            'odom_topic': '/rtabmap/odom',
            'visual_odometry': 'false',  # We're using our own VIO component
            'rtabmap_viz': 'true',
            **rtabmap_params
        }.items()
    )

    return LaunchDescription([
        container,
        load_components,
        rtabmap_launch
    ])


