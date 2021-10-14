import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = list()
    # args that can be set from the command line or a default will be used
    declared_arguments.append(DeclareLaunchArgument("robot_config", default_value=TextSubstitution(
        text=os.path.join(get_package_share_directory("pybullet_ros2"), "config/dual_franka_config.yaml"))))

    # plugin_import_prefix = DeclareLaunchArgument("plugin_import_prefix",
    #                                              default_value=TextSubstitution(text="pybullet_ros.plugins"))
    # environment = DeclareLaunchArgument("environment", default_value=TextSubstitution(text="environment"))
    declared_arguments.append(DeclareLaunchArgument("rviz_bringup", default_value=TextSubstitution(text="True")))
    declared_arguments.append(DeclareLaunchArgument("rviz_config", default_value=TextSubstitution(
        text=os.path.join(get_package_share_directory("pybullet_ros2"), "config/dual_franka.rviz"))))
    declared_arguments.append(DeclareLaunchArgument("pybullet_gui", default_value=TextSubstitution(text="True")))
    declared_arguments.append(DeclareLaunchArgument("gui_options", default_value=TextSubstitution(text="")))
    declared_arguments.append(DeclareLaunchArgument("start_paused", default_value=TextSubstitution(text="False")))
    declared_arguments.append(DeclareLaunchArgument("loop_rate", default_value="500.0"))
    declared_arguments.append(DeclareLaunchArgument("parallel_plugin_execution",
                                                    default_value=TextSubstitution(text="True")))
    # use_deformable_world = DeclareLaunchArgument("use_deformable_world", default_value=TextSubstitution(text="False"))

    sim_loader_node = Node(
        package="pybullet_ros2",
        executable="simulation_loader",
        name="simulation_loader",
        output="screen",
        parameters=[{
            "robot_config": LaunchConfiguration("robot_config"),
            "rviz_bringup": LaunchConfiguration("rviz_bringup"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "pybullet_gui": LaunchConfiguration("pybullet_gui"),
            "gui_options": LaunchConfiguration("gui_options"),
            "start_paused": LaunchConfiguration("start_paused"),
            "loop_rate": LaunchConfiguration("loop_rate"),
            "parallel_plugin_execution": LaunchConfiguration("parallel_plugin_execution"),
        }]
    )

    return LaunchDescription(declared_arguments + [sim_loader_node])
