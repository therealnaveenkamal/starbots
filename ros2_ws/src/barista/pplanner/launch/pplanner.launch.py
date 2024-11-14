import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    controller_yaml_barista_1 = os.path.join(get_package_share_directory(
        'pplanner'), 'config', 'controller_barista_1.yaml')
    bt_navigator_yaml_barista_1 = os.path.join(get_package_share_directory(
        'pplanner'), 'config', 'bt_navigator_barista_1.yaml')
    planner_yaml_barista_1 = os.path.join(get_package_share_directory(
        'pplanner'), 'config', 'planner_server_barista_1.yaml')
    recovery_yaml_barista_1 = os.path.join(get_package_share_directory(
        'pplanner'), 'config', 'recovery_barista_1.yaml')

    barista_1_config = os.path.join(get_package_share_directory('pplanner'), 'config', 'barista_1_amcl_config.yaml')

    map_file = os.path.join(get_package_share_directory('pplanner'), 'config', 'starbots_complete_sim.yaml')

    filters_sim_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters_sim.yaml')


    return LaunchDescription([

        # Nodes for barista_1

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', PathJoinSubstitution([FindPackageShare('pplanner'), 'rviz_config', 'pathplanning.rviz'])],
        ),

        TimerAction(
            period=5.0,
            actions=[

                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': True}, 
                                {'topic_name':"map"},
                                {'frame_id':"map"},
                                {'yaml_filename':map_file}]
                ),
                    
                Node(
                    namespace='barista_1',
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[barista_1_config],
                    remappings=[('/cmd_vel', '/barista_1/cmd_vel')]
                ),

                Node(
                    namespace='barista_1',
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[controller_yaml_barista_1],
                    remappings=[('/cmd_vel', '/barista_1/cmd_vel')]),

                Node(
                    namespace='barista_1',
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[planner_yaml_barista_1],
                    remappings=[('/cmd_vel', '/barista_1/cmd_vel')]),

                Node(
                    namespace='barista_1',
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    parameters=[recovery_yaml_barista_1],
                    remappings=[('/cmd_vel', '/barista_1/cmd_vel')],
                    output='screen'),

                Node(
                    namespace='barista_1',
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[bt_navigator_yaml_barista_1],
                    remappings=[('/cmd_vel', '/barista_1/cmd_vel')]),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='barista_1_lifecycle_manager_pathplanner',
                    output='screen',
                    parameters=[{'autostart': True},
                                {'bond_timeout': 0.0},
                                {'node_names': [
                                    'map_server', 
                                    'barista_1/amcl',
                                    'barista_1/planner_server',
                                    'barista_1/controller_server',
                                    'barista_1/behavior_server',
                                    'barista_1/bt_navigator'
                                ]}])
            ],
        )
    ])
