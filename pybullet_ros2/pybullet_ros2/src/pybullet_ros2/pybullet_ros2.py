#!/usr/bin/env python3

import importlib
import os
import rclpy
import rclpy.node
import yaml
from ament_index_python import get_package_share_directory
from rcl_interfaces.srv import GetParameters

from . import PyBulletRobotROS
from . import PyBulletSimROS


class PyBulletRosWrapper(rclpy.node.Node):
    """ROS wrapper class for pybullet simulator"""

    def __init__(self, node_name):
        """
        Constructor of the PyBulletRosWrapper class.

        :param node_name: Name of the ROS2 node
        :type node_name: str
        """
        super().__init__(node_name)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_config", rclpy.Parameter.Type.STRING),
                ("pybullet_gui", rclpy.Parameter.Type.BOOL),
                ("gui_options", rclpy.Parameter.Type.STRING),
                ("start_paused", rclpy.Parameter.Type.BOOL),
                ("loop_rate", rclpy.Parameter.Type.DOUBLE),
                ("parallel_plugin_execution", rclpy.Parameter.Type.BOOL),
            ])
        self._pb = importlib.import_module("pybullet")
        self._simulation = PyBulletSimROS(self)

        path = self.get_parameter("robot_config").get_parameter_value().string_value
        with open(path, "r") as stream:
            try:
                robot_config = yaml.safe_load(stream)
            except yaml.YAMLError as ex:
                self.get_logger().error(ex)
                self.destroy_node()

        robot_names = robot_config.keys()
        self._robots = {}
        self._plugins = []
        for robot_name in robot_names:
            client = self.create_client(GetParameters, "/" + robot_name + "/robot_state_publisher/get_parameters")
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Service {} not available, waiting again...".format(
                    "/" + robot_name + "/robot_state_publisher/get_parameters"))
            req = GetParameters.Request()
            req.names = ["robot_description"]
            response = client.call_async(req)
            rclpy.spin_until_future_complete(self, response)
            robot_config[robot_name]["urdf"]["robot_description"] = response.result().values[0].string_value
            robot_config[robot_name]["urdf"]["full_path"] = os.path.join(
                get_package_share_directory(robot_config[robot_name]["urdf"]["package"]),
                robot_config[robot_name]["urdf"]["path"])
            robot = PyBulletRobotROS(name=robot_name, sim_uid=self._simulation.uid,
                                     robot_config=robot_config[robot_name])
            self._robots[robot_name] = robot

            # import plugins dynamically
            for plugin in robot_config[robot_name]["plugins"]:
                plugin_ = plugin.copy()
                module_ = plugin_.pop("module")
                class_ = plugin_.pop("class")
                params_ = plugin_.copy()
                self.get_logger().info(
                    "[PyBulletRosWrapper::init] Loading plugin: {} class from {} for robot {}".format(class_, module_,
                                                                                                      robot_name))
                # create object of the imported file class
                obj = getattr(importlib.import_module(module_), class_)(self, self._pb, robot, **params_)
                # store objects in member variable for future use
                self._plugins.append(obj)

        self._pb.setGravity(0, 0, -9.81)
        self._pb.loadURDF('plane.urdf')
        self.get_logger().info("[PyBulletRosWrapper::init] PyBullet ROS wrapper initialized.")

    def _start_pybullet_ros_wrapper_sequential(self):
        """
        This function is deprecated, the use of parallel plugin execution is recommended.
        """
        if not self._simulation.is_paused():
            for task in self._plugins:
                task.execute()
            # perform all the actions in a single forward dynamics simulation step
            self._simulation.step()

    def _start_pybullet_ros_wrapper_parallel(self, loop_rate):
        """
        Execute plugins in parallel, however watch their execution time and warn if they exceed the loop rate.

        :param loop_rate: Loop rate of parallel execution
        :type loop_rate: float
        """
        for plugin in self._plugins:
            self.create_timer(1. / loop_rate, plugin.execute)
        self.get_logger().info(
            "[PyBulletZmqWrapper::start_pybullet_ros_wrapper_parallel] Starting parallel execution of plugins.")
        self.create_timer(1. / loop_rate, self._simulation.step_simulation)

    def start_pybullet_ros_wrapper(self):
        """
        Start execution of plugins.
        """
        loop_rate = self.get_parameter("loop_rate").get_parameter_value().double_value
        if self.get_parameter("parallel_plugin_execution").get_parameter_value().bool_value:
            self._start_pybullet_ros_wrapper_parallel(loop_rate)
        else:
            self.create_timer(1. / loop_rate, self._start_pybullet_ros_wrapper_sequential)


def main():
    rclpy.init()
    node = PyBulletRosWrapper("pybullet_ros2")
    node.start_pybullet_ros_wrapper()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
