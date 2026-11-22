"""Visualiza el URDF en RViz2 con sliders para los joints (sin Gazebo)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('ackermann_description')
    use_meshes = LaunchConfiguration('use_meshes')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'ackermann.urdf.xacro']),
                 ' use_meshes:=', use_meshes]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_meshes', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=[
            '-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])]),
    ])
