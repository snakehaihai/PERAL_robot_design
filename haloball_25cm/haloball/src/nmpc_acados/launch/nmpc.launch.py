import os.path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    package_name = 'nmpc_acados'
    frame_id = 'odom'   # easy change of frame_id for certain nodes

    rviz = LaunchConfiguration('rviz')

    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz is set to True')
    
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'nmpc.rviz')
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz)
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
        ]
    )

    nmpc_acados_node = Node(
        package=package_name,
        executable='nmpc_acados_node',
        parameters=[
            {'frame_id': frame_id},   # change accordingly
            {'weight_x': 5.0},            # X position tracking
            {'weight_y': 5.0},            # Y position tracking
            {'weight_yaw': 0.015},         # yaw tracking
            {'weight_right_wheel_angular_vel': 0.025},
            {'weight_left_wheel_angular_vel': 0.025}
        ],
        remappings=[
            # Remap the default topic to desired topic /visual_local_trajectory
            ('/odom', '/odom'),
            ('/trajectory', '/trajectory'),
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz)

    ld.add_action(rviz2_node)
    ld.add_action(odom_path_publisher_node)
    ld.add_action(trajectory_publisher_node)
    ld.add_action(nmpc_acados_node)

    return ld
