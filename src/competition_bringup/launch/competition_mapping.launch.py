"""Start the competition simulation, online mapping, and saved RViz view."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('competition_bringup'))
    gui = LaunchConfiguration('gui')
    rviz_enabled = LaunchConfiguration('rviz')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch' / 'competition.launch.py')),
        launch_arguments={'gui': gui, 'rviz': 'false'}.items())
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[str(bringup / 'config' / 'slam_mapping.yaml'),
                    {'use_sim_time': True}])
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', str(bringup / 'rviz' / 'competition_mapping_clear.rviz')],
        parameters=[{'use_sim_time': True}], condition=IfCondition(rviz_enabled))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        GroupAction(actions=[simulation], scoped=True), slam, rviz,
    ])
