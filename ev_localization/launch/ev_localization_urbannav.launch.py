import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ev_localization')
    
    # Path to params_urbannav.yaml
    params_file = os.path.join(pkg_dir, 'config', 'params_urbannav.yaml')
    
    # urbannav_converter node
    urbannav_converter_node = Node(
        package='ev_localization',
        executable='urbannav_converter',
        name='urbannav_converter',
        parameters=[params_file]
    )
    
    # gps_monitor node
    gps_monitor_node = Node(
        package='ev_localization',
        executable='gps_monitor',
        name='gps_monitor',
        parameters=[params_file]
    )
    
    # monocular_vio node
    monocular_vio_node = Node(
        package='ev_localization',
        executable='monocular_vio',
        name='monocular_vio',
        parameters=[params_file],
        remappings=[('/camera/image_raw', '/zed2/camera/left/image_raw')]
    )
    
    # landmark_ghost node
    landmark_ghost_node = Node(
        package='ev_localization',
        executable='landmark_ghost',
        name='landmark_ghost',
        parameters=[params_file],
        remappings=[('/camera/image_raw', '/zed2/camera/left/image_raw')]
    )
    
    # ekf_fusion node
    ekf_fusion_node = Node(
        package='ev_localization',
        executable='ekf_fusion',
        name='ekf_fusion',
        parameters=[params_file]
    )
    
    # lane_detector node
    lane_detector_node = Node(
        package='ev_localization',
        executable='lane_detector',
        name='lane_detector',
        parameters=[params_file],
        remappings=[('/camera/image_raw', '/zed2/camera/left/image_raw')]
    )
    
    # uturn_detector node
    uturn_detector_node = Node(
        package='ev_localization',
        executable='uturn_detector',
        name='uturn_detector',
        parameters=[params_file]
    )
    
    # Static TF Publishers
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
    )
    
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['--x', '0', '--y', '0', '--z', '1.5', '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708', '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
    )
    
    static_tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gps',
        arguments=['--x', '0', '--y', '0', '--z', '0.5', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'gps_link']
    )
    
    return LaunchDescription([
        urbannav_converter_node,
        gps_monitor_node,
        monocular_vio_node,
        landmark_ghost_node,
        ekf_fusion_node,
        lane_detector_node,
        uturn_detector_node,
        static_tf_map_to_odom,
        static_tf_base_to_camera,
        static_tf_base_to_gps
    ])
