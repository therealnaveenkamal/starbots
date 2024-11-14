from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    sim_moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    real_moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation '
        ),
        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_node',
            name='basic_grasping_perception_node',
            output='screen',
            parameters=[{'debug_topics': True}],
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        ),
        Node(
            package='simple_grasping',
            executable='basic_grasping_perception_real_node',
            name='basic_grasping_perception_real_node',
            output='screen',
            parameters=[{'debug_topics': True}],
            condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('get_cube_pose'), 'rviz_config', 'point_viz.rviz'])],
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('get_cube_pose'), 'rviz_config', 'point_viz_real.rviz'])],
            condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    name="pick_and_place_perception_sim",
                    package="moveit2_scripts",
                    executable="pick_and_place_perception_sim",
                    output="screen",
                    parameters=[
                        sim_moveit_config.robot_description,
                        sim_moveit_config.robot_description_semantic,
                        sim_moveit_config.robot_description_kinematics,
                        {'use_sim_time': True},
                    ],
                    condition=IfCondition(LaunchConfiguration('use_sim_time'))
                ),
                Node(
                    name="pick_and_place_perception_real",
                    package="moveit2_scripts",
                    executable="pick_and_place_perception_real",
                    output="screen",
                    parameters=[
                        real_moveit_config.robot_description,
                        real_moveit_config.robot_description_semantic,
                        real_moveit_config.robot_description_kinematics,
                        {'use_sim_time': False},
                    ],
                    condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
                ),
        ])
    ])
