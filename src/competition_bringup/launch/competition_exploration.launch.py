"""Run mapping with stationary exploration diagnostics.

Autonomous motion is intentionally blocked until low obstacle clearance can be
validated with the available sensors.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def reject_unsafe_motion(context):
    enabled = LaunchConfiguration('enable_motion').perform(context).lower()
    if enabled not in ('false', '0'):
        raise RuntimeError(
            'Autonomous motion is blocked: the forward camera cannot certify '
            'cube clearance around the entire robot. Use enable_motion:=false.')
    return []


def generate_launch_description():
    bringup = Path(get_package_share_directory('competition_bringup'))
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch' /
                                           'competition_mapping.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
        }.items())
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', str(bringup / 'rviz' / 'competition_perception.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    diagnostics = Node(
        package='competition_bringup', executable='exploration_diagnostics.py',
        parameters=[{'use_sim_time': True}], output='screen')
    detector = Node(
        package='competition_bringup', executable='cube_detector.py',
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('perception')))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('perception', default_value='true'),
        OpaqueFunction(function=reject_unsafe_motion),
        GroupAction(actions=[mapping], scoped=True), detector, diagnostics, rviz,
    ])
