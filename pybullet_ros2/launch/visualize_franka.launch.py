import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    robot_name = DeclareLaunchArgument("robot_name", default_value=TextSubstitution(text="franka"))
    rviz_bringup = DeclareLaunchArgument("rviz_bringup", default_value=TextSubstitution(text="True"))
    rviz_config = DeclareLaunchArgument("rviz_config", default_value=TextSubstitution(
        text=os.path.join(get_package_share_directory("pybullet_ros2"), "config/franka.rviz")))
    urdf_path = DeclareLaunchArgument("urdf_path", default_value=TextSubstitution(
        text=os.path.join(get_package_share_directory("franka_panda_description"), "robots", "panda_arm.urdf.xacro")))

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("robot_name"),
        output="screen",
        parameters=[{
            "urdf_path": LaunchConfiguration("urdf_path"),
            "robot_description": Command(
                ["xacro ", LaunchConfiguration("urdf_path"), " arm_id:=", LaunchConfiguration("robot_name"),
                 " xyz:='0 0 0' rpy:='0 0 0'"])
        }],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=LaunchConfiguration("robot_name"),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz_bringup")),
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_name,
        rviz_bringup,
        rviz_config,
        urdf_path,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
