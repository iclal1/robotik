"""Saved-map Nav2 bringup with camera obstacles and a guarded command path."""
import hashlib
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE = Path(__file__).resolve().parents[3]
DEFAULT_MAP = (WORKSPACE / 'maps/competition_mapping/'
               '20261125T103259Z_572d1928/map.yaml')
NAV2_MAP_METADATA = 'phase5b_nav2_map.yaml'
PHASE5B_MAP_SHA256 = {
    'map.yaml': '27d68b0c7a69a677e61da2cab0a546563cd6c64d9e1012fa4ab6aedfced61abc',
    'map.pgm': '1b8051da18f35d084dbcce3dc9fb101110e358c19d805d591dc872ad0c6d6d61',
}


def competition_servers():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            command = (proc / 'cmdline').read_bytes().replace(b'\0', b' ')
        except (OSError, PermissionError):
            continue
        if b'ign gazebo' in command and b'competition.sdf' in command:
            yield int(proc.name)


def preflight(context):
    if list(competition_servers()):
        raise RuntimeError('Competition Gazebo is already running')
    if LaunchConfiguration('use_sim_time').perform(context).lower() not in ('true','1'):
        raise RuntimeError('Saved-map Nav2 requires use_sim_time:=true')
    enabled = LaunchConfiguration('enable_motion').perform(context).lower()
    if enabled not in ('true','false','1','0'):
        raise RuntimeError('enable_motion must be true or false')
    map_yaml = Path(LaunchConfiguration('map').perform(context))
    if not map_yaml.is_file() or not (map_yaml.parent/'map.pgm').is_file():
        raise RuntimeError(f'Saved map is incomplete: {map_yaml}')
    if map_yaml.resolve() == DEFAULT_MAP.resolve():
        for name, expected in PHASE5B_MAP_SHA256.items():
            if hashlib.sha256((map_yaml.parent/name).read_bytes()).hexdigest() != expected:
                raise RuntimeError(f'Phase 5B saved map changed: {map_yaml.parent/name}')
    return []


def make_nodes(context):
    bringup = Path(get_package_share_directory('competition_bringup'))
    params = str(bringup/'config/nav2_saved_map.yaml')
    map_yaml = Path(LaunchConfiguration('map').perform(context))
    # Use the original PGM with metadata that retains gray-205 unknown cells.
    # The user's saved YAML and image remain untouched.
    if map_yaml.resolve() == DEFAULT_MAP.resolve():
        map_yaml = bringup/'config'/NAV2_MAP_METADATA
    map_yaml = str(map_yaml)
    enabled = LaunchConfiguration('enable_motion').perform(context).lower() in ('true','1')
    navigation = LaunchConfiguration('start_navigation').perform(context).lower() in ('true','1')
    nodes = [
        Node(package='competition_bringup',executable='cube_detector.py',
             parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='competition_bringup',executable='cube_tracking_diagnostics.py',
             parameters=[{'use_sim_time':True}],output='screen'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',
             parameters=[params,{'yaml_filename':map_yaml}],output='screen'),
        Node(package='nav2_amcl',executable='amcl',name='amcl',
             parameters=[params],output='screen'),
        Node(package='competition_bringup',executable='initialize_amcl_pose.py',
             parameters=[{'use_sim_time':True}],output='screen'),
    ]
    # The supervisor gates pre-push Nav2 motion. The pushing task
    # (competition_task.launch.py) sets supervisor:=false and arbitrates
    # /nav2_cmd_vel itself; cmd_vel_guard stays the final stage in both cases.
    if LaunchConfiguration('supervisor').perform(context).lower() in ('true','1'):
        nodes.append(Node(package='competition_bringup',executable='nav2_safety_supervisor.py',
                          parameters=[{'use_sim_time':True,'enable_motion':enabled}],
                          output='screen'))
    lifecycle = ['map_server','amcl']
    if navigation:
        nodes.extend([
            Node(package='nav2_planner',executable='planner_server',
                 name='planner_server',parameters=[params],output='screen'),
            Node(package='nav2_controller',executable='controller_server',
                 name='controller_server',parameters=[params],output='screen',
                 remappings=[('cmd_vel','/nav2_cmd_vel')]),
            Node(package='nav2_behaviors',executable='behavior_server',
                 name='behavior_server',parameters=[params],output='screen',
                 remappings=[('cmd_vel','/nav2_cmd_vel')]),
            Node(package='nav2_bt_navigator',executable='bt_navigator',
                 name='bt_navigator',parameters=[params],output='screen'),
        ])
        lifecycle.extend(['planner_server','controller_server',
                          'behavior_server','bt_navigator'])
    nodes.append(Node(package='nav2_lifecycle_manager',
                      executable='lifecycle_manager',name='lifecycle_manager_nav2',
                      parameters=[{'use_sim_time':True,'autostart':True,'bond_timeout':20.0,
                                   'node_names':lifecycle}],output='screen'))
    return nodes


def generate_launch_description():
    bringup = Path(get_package_share_directory('competition_bringup'))
    simulation = GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup/'launch/competition.launch.py')),
        launch_arguments={'gui':LaunchConfiguration('gui'),'rviz':'false'}.items())],
        scoped=True)
    rviz = Node(package='rviz2',executable='rviz2',output='screen',
                arguments=['-d',str(bringup/'rviz/competition_nav2_clear.rviz')],
                parameters=[{'use_sim_time':True}],
                condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('enable_motion',default_value='false'),
        DeclareLaunchArgument('start_navigation',default_value='true'),
        DeclareLaunchArgument('supervisor',default_value='true'),
        DeclareLaunchArgument('map',default_value=str(DEFAULT_MAP)),
        OpaqueFunction(function=preflight),
        OpaqueFunction(function=make_nodes),simulation,rviz,
    ])
