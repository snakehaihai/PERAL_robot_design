import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description() -> LaunchDescription:

    # Package name
    package_name='haloball_bringup' 

    # Set use_sim_time to false
    use_sim_time = 'false'

    ####################### Livox_ros_driver2 parameters start #######################
    xfer_format   = 1    # 0-PointCloud2Msg(PointXYZRTL), 1-LivoxCustomMsg, 2-PclPxyziMsg, 3-LivoxImuMsg, 4-AllMsg
    multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src      = 0    # 0-lidar, others-Invalid data src
    publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    cur_config_path = cur_path + '../config'
    user_config_path = os.path.join(cur_config_path, 'MID360_config.json') # Modify this config ip address under haloball_bringup/config (line28)

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    ####################### Livox_ros_driver2 parameters end #########################

    # Launch livox_ros_driver2_node
    livox_ros_driver2_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # Launch fast_lio_mapping
    fast_lio_mapping = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py'
                )]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': "False",
                }.items()
            )
        ]
    )

    # Launch base_link_tf_node
    base_link_tf_node = Node(
        package=package_name,       # package name
        executable='base_link_tf',   # executable name
        name='base_link_tf',            # node name
        parameters=[
            {'lidar_to_base_link_distance': 0.021185229}, # default value is 0.0m, my base_link is center of ball to utilise similar triangle property
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch nodes

    ld.add_action(livox_ros_driver2_node)
    ld.add_action(fast_lio_mapping)
    ld.add_action(base_link_tf_node)

    return ld