"""Lanza Gazebo Harmonic con el robot Ackermann, robot_state_publisher y ros_gz_bridge.

Uso:
  ros2 launch ackermann_gazebo sim.launch.py              # con GUI de Gazebo + RViz2
  ros2 launch ackermann_gazebo sim.launch.py headless:=true rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gazebo(context):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = f"-r {'-s ' if headless else ''}{world}"
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    desc_share = get_package_share_directory('ackermann_description')
    gz_share = get_package_share_directory('ackermann_gazebo')

    xacro_file = os.path.join(desc_share, 'urdf', 'ackermann.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_meshes:=', LaunchConfiguration('use_meshes')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(
            gz_share, 'worlds', 'ackermann_world.sdf')),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = solo servidor de Gazebo (sin GUI)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_meshes', default_value='true',
                              description='true = usar los STL del CAD de B'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        # Para que Gazebo encuentre package://ackermann_description/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share)),

        OpaqueFunction(function=_gazebo),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'ackermann_robot',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', '0.06', '-Y', LaunchConfiguration('yaw')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gz_share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}]),

        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(desc_share, 'rviz', 'sim.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
