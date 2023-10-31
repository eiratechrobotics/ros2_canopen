import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    package_name='canopen_tests' #<--- CHANGE ME

    ld = LaunchDescription()

    map_file = map_server_config_path = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'ten_metre_map.yaml'
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}])

    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    map_odom_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["0","0","0","0","0","0","map","odom"])


    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(map_odom_transform)

    return ld