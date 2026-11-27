"""Full autonomous task: Gazebo + saved-map Nav2 + cube_pusher.

    ros2 launch competition_bringup competition_task.launch.py gui:=true

cube_pusher waits for Nav2 and AMCL by itself, then delivers the cubes in
the given order. No Gazebo ground truth is used by any node started here.
"""
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAP_REL = 'maps/competition_mapping/20261125T103259Z_572d1928/nav2_portable_example.yaml'


def default_map():
    # Works for both symlink and copy installs: try the source tree, then
    # the workspace that contains install/competition_bringup.
    roots = [Path(__file__).resolve().parents[3],
             Path(get_package_prefix('competition_bringup')).resolve().parents[1]]
    for root in roots:
        if (root / MAP_REL).is_file():
            return str(root / MAP_REL)
    return str(roots[-1] / MAP_REL)


def generate_launch_description():
    bringup = Path(get_package_share_directory('competition_bringup'))
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup / 'launch/competition_nav2.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'map': LaunchConfiguration('map'),
            'enable_motion': 'true',
            'supervisor': 'false',
        }.items())
    pusher = Node(package='competition_bringup', executable='cube_pusher.py',
                  parameters=[{'use_sim_time': True,
                               'order': LaunchConfiguration('order')}],
                  output='screen', emulate_tty=True)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map()),
        DeclareLaunchArgument('order', default_value='green,red,blue'),
        nav2, pusher,
    ])
