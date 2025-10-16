import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pathplanner_dir = os.path.join(get_package_share_directory('tb4_path_planner_server'), 'config')
    localization_dir = os.path.join(get_package_share_directory('tb4_localization_server'), 'config')
    # map_config_dir = os.path.join(get_package_share_directory('tb4_map_server'), 'config')
    # filters_yaml = os.path.join(map_config_dir, 'filters.yaml')
    # map_keepout_yaml = os.path.join(map_config_dir, 'map_keepout.yaml')
    # map_speeds_yaml = os.path.join(map_config_dir, 'map_speeds.yaml')

    localization_yaml = os.path.join(localization_dir, 'amcl_config.yaml')
    controller_yaml = os.path.join(pathplanner_dir, 'controller.yaml')
    bt_navigator_yaml = os.path.join(pathplanner_dir, 'bt_navigator.yaml')
    planner_yaml = os.path.join(pathplanner_dir, 'planner.yaml')
    behaviour_yaml = os.path.join(pathplanner_dir, 'recoveries.yaml')
    wp_follower_yaml = os.path.join(pathplanner_dir, 'waypoint_follower.yaml')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map_config')
    rviz_config = LaunchConfiguration('rviz_config')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # params_file = LaunchConfiguration('params_config')

    # remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    # param_substitutions = {'autostart': autostart}

    # configured_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=params_file,
    #         root_key=namespace,
    #         param_rewrites=param_substitutions,
    #         convert_types=True,
    #     ),
    #     allow_substs=True,
    # )

    load_nodes = GroupAction(
        actions=[
            # PushROSNamespace(namespace),

            # Map & Filter nodes
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'yaml_filename': map_yaml}]),
            # Node(
            #     package='nav2_map_server',
            #     executable='map_server',
            #     name='filter_mask_server',
            #     output='screen',
            #     emulate_tty=True,
            #     parameters=[filters_yaml],
            #     remappings=[
            #         ('/keepout_filter_mask', map_keepout_yaml),
            #         ('/speed_filter_mask', map_speeds_yaml)]),
            # Node(
            #     package='nav2_map_server',
            #     executable='costmap_filter_info_server',
            #     name='costmap_filter_info_server',
            #     output='screen',
            #     emulate_tty=True,
            #     parameters=[filters_yaml]),

            # Localization
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[localization_yaml]),

            # Planning
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_yaml]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen'),
            
            # Controllers
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml, 
                            {'use_sim_time': use_sim_time}]),
            
            # BT
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[behaviour_yaml, 
                            {'use_sim_time': use_sim_time}]),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml, 
                            {'use_sim_time': use_sim_time}]),
            
            # Waypoint follower application
            # Node(
            #     package='nav2_waypoint_follower',
            #     executable='waypoint_follower',
            #     name='waypoint_follower',
            #     output='screen',
            #     parameters=[wp_follower_yaml]),

            # Rviz
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                name='rviz2_node',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn']),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['map_server',
                                            'amcl',
                                            'planner_server',
                                            'smoother_server',
                                            'controller_server',
                                            'bt_navigator',
                                            'behavior_server',
                                            # 'costmap_filter_info_server',
                                            # 'filter_mask_server',
                                            ]}])
        ]
    )
    # Create the launch description and launch all of the navigation nodes
    ld = LaunchDescription()
    ld.add_action(load_nodes)
    return ld
