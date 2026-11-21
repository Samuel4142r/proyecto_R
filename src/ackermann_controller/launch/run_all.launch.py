"""Todo en uno: Gazebo + robot + puente + RViz2 + Pure Pursuit + registro CSV.

  ros2 launch ackermann_controller run_all.launch.py
  ros2 launch ackermann_controller run_all.launch.py headless:=true rviz:=false run_name:=fig8_run1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = os.path.join(get_package_share_directory('ackermann_gazebo'), 'launch', 'sim.launch.py')
    ctrl = os.path.join(get_package_share_directory('ackermann_controller'),
                        'launch', 'controller.launch.py')
    lc = LaunchConfiguration
    args = ['headless', 'rviz', 'use_meshes', 'path_type', 'linear_speed',
            'lookahead_distance', 'laps', 'odom_topic', 'log', 'output_dir', 'run_name']
    defaults = {'headless': 'false', 'rviz': 'true', 'use_meshes': 'false',
                'path_type': 'figure8', 'linear_speed': '0.5', 'lookahead_distance': '0.6',
                'laps': '1', 'odom_topic': '/odom', 'log': 'true', 'output_dir': os.path.join(os.getcwd(), 'results'),
                'run_name': ''}
    return LaunchDescription(
        [DeclareLaunchArgument(a, default_value=defaults[a]) for a in args] + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim),
                launch_arguments={k: lc(k) for k in ['headless', 'rviz', 'use_meshes']}.items()),
            # Espera a que Gazebo haya cargado el robot
            TimerAction(period=6.0, actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ctrl),
                launch_arguments={k: lc(k) for k in args[3:]}.items())]),
        ])
