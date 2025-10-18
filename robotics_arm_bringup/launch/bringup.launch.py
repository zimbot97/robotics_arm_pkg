import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    pkg_share = FindPackageShare('robotics_arm_moveit').find('robotics_arm_moveit')

    # Load robot description
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'config', 'robotics_arm.urdf.xacro'),
        ' ',
        'is_sim:=false'
    ])

    # Controller Manager Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        },
        pkg_share + '/config/ros2_controllers.yaml'],
        output='screen'
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    # MoveIt (optional)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robotics_arm_bringup'), '/launch/moveit.launch.py'
        ])
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        moveit_launch
    ])