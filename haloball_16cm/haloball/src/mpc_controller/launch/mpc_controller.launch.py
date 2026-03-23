import os.path
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'mpc_controller'
    frame_id = 'odom'   # easy change of frame_id for certain nodes
    
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'nmpc.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
    )

    odom_path_publisher_node = Node(
        package=package_name,
        executable='odom_path_publisher',
        parameters=[
            {'frame_id': frame_id},   # change accordingly
            {'odom_topic': "/odom"},
            {'odom_path_topic': "/odom_path"},
        ]
    )
    
    trajectory_publisher_node = Node(
        package=package_name,
        executable='trajectory_publisher_node',
        parameters=[
            {'frame_id': frame_id},   # change accordingly
            {'trajectory_type': "circle"}, # "circle", "fig8", "oval", ""
        ]
    )

    mpc_controller_node = Node(
        package=package_name,
        executable='mpc_controller',
        parameters=[
            {'frame_id': frame_id},   # change accordingly
            {'weight_x': 5.0},            # X position tracking
            {'weight_y': 5.0},            # Y position tracking
            {'weight_yaw': 0.75},         # yaw tracking
            {'weight_linear_vel': 10.0},
            {'weight_angular_vel': 0.2}
        ],
        remappings=[
            # Remap the default topic to desired topic /visual_local_trajectory
            ('/odom', '/odom'),
            ('/global_path', '/global_path'),
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(rviz2_node)
    ld.add_action(odom_path_publisher_node)
    ld.add_action(trajectory_publisher_node)
    ld.add_action(mpc_controller_node)

    return ld
