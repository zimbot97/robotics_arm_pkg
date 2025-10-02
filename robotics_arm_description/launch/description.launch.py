import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    # Path to URDF
    # Absolute path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory("robotics_arm_description"),
        "urdf",
        "robotics_arm_with_gripper.urdf"
    )

    # Read URDF contents
    with open(urdf_file, "r") as infp:
        robot_description_content = infp.read()



   

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])
    static_transform_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gripper_base_to_end_effector_broadcaster",
            arguments=["0.0", "-0.095", "0.0", "0", "0", "0", "1", "gripper_base", "end_effector_lin"],
        )
    ld.add_action(robot_state_publisher_node)
    ld.add_action(static_transform_node)
   
    
    return ld
