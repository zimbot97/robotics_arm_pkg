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
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim = LaunchConfiguration("is_sim")
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
        parameters=[{
            'use_sim_time': False
        }],
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_state_publisher_spawner = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": is_sim,
            "robot_description": robot_description,
        }]
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
    )

    # MoveIt (optional)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robotics_arm_bringup'), '/launch/moveit.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': is_sim,
        }.items(),
    )

    return LaunchDescription([
        is_sim_arg,
        control_node,
        joint_state_broadcaster_spawner,
        robot_state_publisher_spawner,
        static_tf_node,
        arm_controller_spawner,
        gripper_controller_spawner,
        moveit_launch
    ])