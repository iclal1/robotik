"""Preserved mapping/perception stack plus gated frontier exploration."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE = Path(__file__).resolve().parents[3]
def _competition_servers():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            command = (proc / 'cmdline').read_bytes().replace(b'\0', b' ')
        except (OSError, PermissionError):
            continue
        if b'ign gazebo' in command and b'competition.sdf' in command:
            yield int(proc.name)


def safety_preflight(context):
    if list(_competition_servers()):
        raise RuntimeError('A competition Gazebo server is already running; stop that session first')
    if LaunchConfiguration('use_sim_time').perform(context).lower() not in ('true', '1'):
        raise RuntimeError('Autonomous exploration requires use_sim_time:=true')
    enable = LaunchConfiguration('enable_motion').perform(context).lower()
    if enable not in ('true', '1', 'false', '0'):
        raise RuntimeError('enable_motion must be true or false')
    if enable in ('false', '0'):
        return []
    mode = LaunchConfiguration('validation_mode').perform(context)
    if mode not in ('rotation', 'translation', 'goal', 'none'):
        raise RuntimeError('validation_mode must be rotation, translation, goal, or none')
    if mode == 'none':
        raise RuntimeError('Full autonomous exploration remains blocked until the bounded goal trial passes')
    return []


def generate_launch_description():
    bringup = Path(get_package_share_directory('competition_bringup'))
    existing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch' /
                                           'competition_exploration.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'rviz': 'false',
                          'enable_motion': 'false', 'perception': 'true'}.items())
    def controller_action(context):
        # Resolve before the nested exploration launch declares its own
        # enable_motion:=false argument.
        enabled = LaunchConfiguration('enable_motion').perform(context).lower() in ('true', '1')
        mode = LaunchConfiguration('validation_mode').perform(context)
        return [Node(package='competition_bringup',
                     executable='autonomous_exploration.py',
                     parameters=[{'use_sim_time': True, 'enable_motion': enabled,
                                  'validation_mode': mode}], output='screen')]
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', str(bringup / 'rviz' /
                             'competition_autonomous_exploration.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('validation_mode', default_value='none'),
        OpaqueFunction(function=safety_preflight),
        OpaqueFunction(function=controller_action), existing, rviz,
    ])
