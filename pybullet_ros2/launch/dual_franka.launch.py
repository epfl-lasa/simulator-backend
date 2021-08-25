import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    robot_config = DeclareLaunchArgument("robot_config", default_value=TextSubstitution(
        text=os.path.join(get_package_share_directory("pybullet_ros2"), "config/dual_franka_config.yaml")))

    # plugin_import_prefix = DeclareLaunchArgument("plugin_import_prefix",
    #                                              default_value=TextSubstitution(text="pybullet_ros.plugins"))
    # environment = DeclareLaunchArgument("environment", default_value=TextSubstitution(text="environment"))
    rviz_bringup = DeclareLaunchArgument("rviz_bringup", default_value=TextSubstitution(text="True"))
    rviz_config = DeclareLaunchArgument("rviz_config", default_value=TextSubstitution(
        text=os.path.join(get_package_share_directory("pybullet_ros2"), "config/dual_franka.rviz")))
    pybullet_gui = DeclareLaunchArgument("pybullet_gui", default_value=TextSubstitution(text="True"))
    gui_options = DeclareLaunchArgument("gui_options", default_value=TextSubstitution(text=""))
    start_paused = DeclareLaunchArgument("start_paused", default_value=TextSubstitution(text="False"))
    loop_rate = DeclareLaunchArgument("loop_rate", default_value="500.0")
    parallel_plugin_execution = DeclareLaunchArgument("parallel_plugin_execution",
                                                      default_value=TextSubstitution(text="True"))
    # use_deformable_world = DeclareLaunchArgument("use_deformable_world", default_value=TextSubstitution(text="False"))

    preloader_node = Node(
        package="pybullet_ros2",
        executable="preloader",
        name="preloader_node",
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

    return LaunchDescription([
        robot_config,
        rviz_bringup,
        rviz_config,
        pybullet_gui,
        gui_options,
        start_paused,
        loop_rate,
        parallel_plugin_execution,
        preloader_node
    ])
