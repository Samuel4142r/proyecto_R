"""Lanza el controlador Pure Pursuit y el nodo de registro (la simulacion debe estar corriendo).

  ros2 launch ackermann_controller controller.launch.py
  ros2 launch ackermann_controller controller.launch.py path_type:=oval run_name:=oval_1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ackermann_controller'),
                          'config', 'pure_pursuit.yaml')
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('path_type', default_value='figure8',
                              description='figure8 | oval | custom'),
        DeclareLaunchArgument('linear_speed', default_value='0.5'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.6'),
        DeclareLaunchArgument('laps', default_value='1'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='/odom (ruedas) o /ground_truth/odom (pose perfecta)'),
        DeclareLaunchArgument('log', default_value='true', description='guardar CSV'),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(os.getcwd(), 'results')),
        DeclareLaunchArgument('run_name', default_value=''),

        Node(package='ackermann_controller', executable='pure_pursuit', output='screen',
             parameters=[lc('params_file'), {
                 'path_type': lc('path_type'),
                 'linear_speed': lc('linear_speed'),
                 'lookahead_distance': lc('lookahead_distance'),
                 'laps': lc('laps'),
                 'odom_topic': lc('odom_topic'),
                 'wait_for_logger': lc('log')}]),

        Node(package='ackermann_controller', executable='data_logger', output='screen',
             parameters=[lc('params_file'), {
                 'output_dir': lc('output_dir'), 'run_name': lc('run_name')}],
             condition=IfCondition(lc('log'))),
    ])
