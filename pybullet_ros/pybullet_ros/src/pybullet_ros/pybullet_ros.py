#!/usr/bin/env python3

import importlib

import rospy
from pybullet_simulation import FuncExecManager

from .ros_robot import ROSRobot
from .ros_simulation import ROSSimulation


class PyBulletRosWrapper(object):
    """ROS wrapper class for pybullet simulator"""

    def __init__(self, ):
        self._pb = importlib.import_module("pybullet")
        self._simulation = ROSSimulation()

        #     # create object of environment class for later use
        #     env_plugin = rospy.get_param("~environment", "environment")  # default : plugins/environment.py
        #     plugin_import_prefix = rospy.get_param("~plugin_import_prefix", "pybullet_ros.plugins")
        #     self._environment = getattr(importlib.import_module(f"{plugin_import_prefix}.{env_plugin}"), "Environment")(
        #         self._pb)
        #     rospy.loginfo("[PyBulletRosWrapper::init] Loading environment.")
        #     # set gravity and ground plane
        #     self._environment.load_environment()
        self._pb.setGravity(0, 0, rospy.get_param("~gravity", -9.81))
        ground = self._pb.loadURDF('plane.urdf')
        self._pb.changeDynamics(ground, -1, restitution=0.9)

        robot_names = rospy.get_param("~robots", None)
        self._robots = {}
        self._plugins = []
        for robot_name in robot_names:
            robot_config = dict()
            robot_config["urdf"] = {}
            robot_config["urdf"]["robot_description"] = rospy.get_param("/" + robot_name + "/robot_description")
            robot_config["urdf"]["full_path"] = rospy.get_param("/" + robot_name + "/urdf_path")
            robot_config["fixed_base"] = rospy.get_param("/" + robot_name + "/fixed_base")
            robot_config["use_inertia_from_file"] = rospy.get_param("/" + robot_name + "/use_inertia_from_file")
            robot = ROSRobot(name=robot_name, sim_uid=self._simulation.uid, robot_config=robot_config)
            self._robots[robot_name] = robot

            plugins = rospy.get_param("~robots/" + robot_name)
            # import plugins dynamically
            for plugin in plugins:
                plugin_ = plugin.copy()
                module_ = plugin_.pop("module")
                class_ = plugin_.pop("class")
                params_ = plugin_.copy()
                rospy.loginfo(
                    "[PyBulletRosWrapper::init] Loading plugin: {} class from {} for robot {}".format(class_, module_,
                                                                                                      robot_name))
                # create object of the imported file class
                obj = getattr(importlib.import_module(module_), class_)(self._pb, robot, **params_)
                # store objects in member variable for future use
                self._plugins.append(obj)
        rospy.loginfo("[PyBulletRosWrapper::init] PyBullet ROS wrapper initialized.")

    def _start_pybullet_ros_wrapper_sequential(self, loop_rate):
        """
        This function is deprecated, we recommend the use of parallel plugin execution
        """
        rate = rospy.Rate(loop_rate)
        while not rospy.is_shutdown():
            if not self._simulation.is_paused():
                for task in self._plugins:
                    task.execute()
                self._simulation.step()
            rate.sleep()
        rospy.logwarn("[PyBulletROSWrapper::start_pybullet_ros_wrapper_sequential] Killing all programs now.")
        if self._simulation.is_alive():
            del self._simulation

    def _start_pybullet_ros_wrapper_parallel(self, loop_rate):
        """
        Execute plugins in parallel, however watch their execution time and warn if exceeds the deadline (loop rate)
        """
        exec_manager_obj = FuncExecManager(self._plugins, rospy.is_shutdown, self._simulation.step,
                                           self._simulation.is_paused, log_info=rospy.loginfo,
                                           log_warn=rospy.logwarn, log_debug=rospy.logdebug)
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate)

    def start_pybullet_ros_wrapper(self):
        loop_rate = rospy.get_param("~loop_rate", 250.0)
        if rospy.get_param("~parallel_plugin_execution", False):
            self._start_pybullet_ros_wrapper_parallel(loop_rate)
        else:
            self._start_pybullet_ros_wrapper_sequential(loop_rate)


def main():
    rospy.init_node("pybullet_ros", anonymous=False)
    node = PyBulletRosWrapper()
    node.start_pybullet_ros_wrapper()


if __name__ == '__main__':
    main()
