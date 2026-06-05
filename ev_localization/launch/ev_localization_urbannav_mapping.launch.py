import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ev_localization')
    
    # Path to params_urbannav.yaml
    params_file = os.path.join(pkg_dir, 'config', 'params_urbannav.yaml')
    
    # 1. urbannav_converter node (Override sim_gps_loss to False to maintain perfect GT pose during mapping)
    urbannav_converter_node = Node(
        package='ev_localization',
        executable='urbannav_converter',
        name='urbannav_converter',
        parameters=[params_file, {'sim_gps_loss': False}]
    )
    
    # 2. ekf_fusion node to fuse GPS and Odom into /ekf/pose
    ekf_fusion_node = Node(
        package='ev_localization',
        executable='ekf_fusion',
        name='ekf_fusion',
        parameters=[params_file]
    )
    
    # 3. yolo_detector node
    yolo_detector_node = Node(
        package='ev_localization',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[params_file],
        remappings=[('/camera/image_raw', '/zed2/camera/left/image_raw')]
    )
    
    # 4. landmark_builder node (our new mapping node)
    landmark_builder_node = Node(
        package='ev_localization',
        executable='landmark_builder',
        name='landmark_builder',
        parameters=[params_file]
    )
    
    # 5. Static TF Publishers
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
        ekf_fusion_node,
        yolo_detector_node,
        landmark_builder_node,
        static_tf_map_to_odom,
        static_tf_base_to_camera,
        static_tf_base_to_gps
    ])
