import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ev_localization')
    
    # Path to params.yaml
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
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
        parameters=[params_file]
    )
    
    # landmark_ghost node
    landmark_ghost_node = Node(
        package='ev_localization',
        executable='landmark_ghost',
        name='landmark_ghost',
        parameters=[params_file]
    )
    
    # ekf_fusion node
    ekf_fusion_node = Node(
        package='ev_localization',
        executable='ekf_fusion',
        name='ekf_fusion',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        gps_monitor_node,
        monocular_vio_node,
        landmark_ghost_node,
        ekf_fusion_node
    ])