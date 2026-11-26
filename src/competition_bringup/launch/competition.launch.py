"""Start the Fortress arena and its ROS interfaces."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    bringup = Path(get_package_share_directory('competition_bringup'))
    world = str(Path(get_package_share_directory('competition_simulation')) / 'worlds/competition.sdf')
    robot = str(Path(get_package_share_directory('competition_robot_description')) / 'urdf/pusher_robot.urdf.xacro')
    description = xacro.process_file(robot).toxml()

    server = str(Path(get_package_prefix('competition_bringup')) / 'lib/competition_bringup/fortress_server.py')
    gazebo = ExecuteProcess(cmd=[server, world], output='screen')
    gazebo_gui = ExecuteProcess(
        cmd=['env', '-u', 'IGN_CONFIG_PATH', 'ign', 'gazebo', '-g'], output='screen',
        condition=IfCondition(LaunchConfiguration('gui')))
    state = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        prefix='setsid',
        parameters=[{'robot_description': description, 'use_sim_time': True}])

    def bridge(name, config, extra=None):
        params = {'config_file': str(bringup / 'config' / config), 'use_sim_time': True}
        if extra:
            params.update(extra)
        return Node(package='ros_gz_bridge', executable='parameter_bridge', name=name,
                    parameters=[params], output='screen')

    core = bridge('core_bridge', 'core_bridges.yaml')
    joint_states = Node(package='competition_bringup', executable='joint_state_rate_limiter.py',
                        output='screen')
    lidar = bridge('lidar_bridge', 'lidar_bridge.yaml', {
        'override_frame_id': 'lidar_link',
        'qos_overrides./scan.publisher.reliability': 'best_effort',
        'qos_overrides./scan.publisher.depth': 5,
    })
    camera = bridge('camera_bridge', 'camera_bridges.yaml', {
        'override_frame_id': 'camera_optical_frame',
        'subscription_heartbeat': 100,
        'qos_overrides./camera/image_raw.publisher.reliability': 'reliable',
        'qos_overrides./camera/image_raw.publisher.depth': 2,
        'qos_overrides./camera/camera_info.publisher.reliability': 'reliable',
        'qos_overrides./camera/camera_info.publisher.depth': 5,
    })
    odometry = Node(package='competition_bringup', executable='odometry_corrector.py',
                    parameters=[{'use_sim_time': False}], output='screen')
    guard = Node(package='competition_bringup', executable='cmd_vel_guard.py',
                 parameters=[{'use_sim_time': False, 'timeout_sec': 0.30}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'pusher_robot', '-topic', 'robot_description',
                            '-x', '-2.40', '-y', '-1.40', '-z', '0', '-Y', '0'], output='screen')
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', str(bringup / 'rviz' / 'competition.rviz')],
                parameters=[{'use_sim_time': True}], output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')))
    def stop_if_process_exits(event, context):
        return [] if context.is_shutdown else [Shutdown(reason='required simulator process exited')]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        gazebo, gazebo_gui, state, core, joint_states, lidar, camera, odometry, guard, rviz,
        TimerAction(period=3.0, actions=[spawn]),
        *[RegisterEventHandler(OnProcessExit(target_action=process, on_exit=stop_if_process_exits))
          for process in (gazebo, core, joint_states, lidar, camera, odometry, guard)],
    ])
