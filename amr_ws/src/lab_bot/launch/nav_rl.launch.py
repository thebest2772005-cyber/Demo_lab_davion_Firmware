import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_pkg = get_package_share_directory('lab_bot')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(
        robot_pkg,
        'config',
        'lab_params.yaml'
    )

    rviz_config_file = os.path.join(
        robot_pkg,
        'rviz',
        'robot.rviz'
    )

    map_file = os.path.join(
        robot_pkg,
        'maps',
        'NewLidar_edited.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_pkg,
                'launch',
                'bringup_launch.py'
            )
        ),
        
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'map': map_file,
            'autostart': 'true'
        }.items()
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       output='screen',
       arguments=[
           '-d',
           rviz_config_file
       ],
       parameters=[{
           'use_sim_time': use_sim_time
       }]
    )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_odom',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )

    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_lidar_tf',
    #     output='screen',
    #     arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'lidar_link']
    # )

    return LaunchDescription([
        declare_use_sim_time,
        nav2
        # rviz
        # static_tf
    ])