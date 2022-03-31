#!/usr/bin/env python3

import launch_ros.actions
import os
import rclpy
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.substitutions import Command
from rclpy.node import Node


class SimulationLoader(Node):
    """
    Helper class to spin up all required nodes from the launch file and the simulation configuration.
    """

    def __init__(self, node_name):
        """
        Constructor of the SimulationLoader class.

        :param node_name: Name of the node
        :type node_name: str
        """
        super().__init__(node_name)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_config", rclpy.Parameter.Type.STRING),
                ("rviz_bringup", rclpy.Parameter.Type.BOOL),
                ("rviz_config", rclpy.Parameter.Type.STRING),
                ("pybullet_gui", rclpy.Parameter.Type.BOOL),
                ("gui_options", rclpy.Parameter.Type.STRING),
                ("start_paused", rclpy.Parameter.Type.BOOL),
                ("loop_rate", rclpy.Parameter.Type.DOUBLE),
                ("parallel_plugin_execution", rclpy.Parameter.Type.BOOL),
            ])
        path = self.get_parameter("robot_config").get_parameter_value().string_value
        with open(path, "r") as stream:
            try:
                self.yaml = yaml.safe_load(stream)
            except yaml.YAMLError as ex:
                self.get_logger().error(ex)
                self.destroy_node()

    @property
    def robot_config(self):
        return self.yaml


def main():
    rclpy.init()
    preloader = SimulationLoader("sim_loader")
    yaml_content = preloader.robot_config

    nodes = []
    for robot in yaml_content.keys():
        nodes.append(launch_ros.actions.Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=robot,
            output={"both": "log"},
            parameters=[{
                "robot_description": Command(
                    ["xacro ", os.path.join(get_package_share_directory(yaml_content[robot]["urdf"]["package"]),
                                            yaml_content[robot]["urdf"]["path"]), " prefix:=", robot, "_ xyz:='",
                     " ".join(str(pos) for pos in yaml_content[robot]["position"]), "' rpy:='",
                     " ".join(str(pos) for pos in yaml_content[robot]["rpy"]), "'"])
            }],
        ))

    if preloader.get_parameter("rviz_bringup").get_parameter_value().bool_value:
        nodes.append(launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", preloader.get_parameter("rviz_config").get_parameter_value().string_value],
            output={"both": "log"},
        ))

    nodes.append(launch_ros.actions.Node(
        package="pybullet_ros2",
        executable="pybullet_ros2",
        name="pybullet_ros2",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "robot_config": preloader.get_parameter("robot_config").get_parameter_value().string_value,
            "pybullet_gui": preloader.get_parameter("pybullet_gui").get_parameter_value().bool_value,
            "gui_options": preloader.get_parameter("gui_options").get_parameter_value().string_value,
            "start_paused": preloader.get_parameter("start_paused").get_parameter_value().bool_value,
            "loop_rate": preloader.get_parameter("loop_rate").get_parameter_value().double_value,
            "parallel_plugin_execution": preloader.get_parameter(
                "parallel_plugin_execution").get_parameter_value().bool_value,
        }]
    ))

    launch_description = LaunchDescription(nodes)

    preloader.destroy_node()

    ls = LaunchService()
    ls.include_launch_description(launch_description)
    return ls.run()


if __name__ == "__main__":
    main()
