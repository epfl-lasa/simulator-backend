import os

import pybullet as pb
import pybullet_data as pb_data
import rclpy
from std_srvs.srv import Trigger


class PyBulletSim(object):
    """
    The Simulation class creates the PyBullet physics server and optionally a GUI.
    Available methods (for usage, see documentation at function definition):
        - uid
        - is_alive
        - step
        - add_pybullet_path
        - add_search_path
    """

    def __init__(self, node):
        """
        Constructor of the Simulation class. This class creates the PyBullet server / GUI and steps the simulation.
        :param node: ROS2 node
        :type node: rclpy.node.Node
        """
        assert isinstance(node, rclpy.node.Node), "[PyBulletSim::init] Parameter 'node' has an incorrect type."
        self.node_ = node

        if self.node_.get_parameter("pybullet_gui").get_parameter_value().bool_value:
            self.node_.get_logger().info("[PyBulletSim::init] Running PyBullet with GUI")
            self.node_.get_logger().info("-------------------------")
            self._uid = pb.connect(pb.GUI,
                                   options=self.node_.get_parameter("gui_options").get_parameter_value().string_value)
        else:
            self.node_.get_logger().info("[PyBulletSim::init] Running PyBullet without GUI")
            self.node_.get_logger().info("-------------------------")
            self._uid = pb.connect(pb.DIRECT)

        self._simulation_paused = self.node_.get_parameter("start_paused").get_parameter_value().bool_value

        self.add_pybullet_path()

        # setup services to pause/unpause and restart simulation
        self.node_.create_service(Trigger, os.path.join(self.node_.get_name(), "reset_simulation"),
                                  self.reset_simulation)
        self.node_.create_service(Trigger, os.path.join(self.node_.get_name(), "pause_simulation"),
                                  self.pause_simulation)
        self.node_.create_service(Trigger, os.path.join(self.node_.get_name(), "unpause_simulation"),
                                  self.unpause_simulation)

    @property
    def uid(self):
        """
        Get UID of physics server
        :rtype: int
        """
        return self._uid

    def is_alive(self):
        """
        Check if the physics server is still connected
        :rtype: bool
        """
        return pb.isConnected(self._uid)

    def is_paused(self):
        """
        Check if the simulation if paused.
        :rtype: bool
        """
        return self._simulation_paused

    def step(self):
        """
        Step the simulation.
        """
        pb.stepSimulation(self._uid)

    def reset_simulation(self, req, res):
        """
        Reset the simulation.
        """
        self.pause_simulation(Trigger)
        self.node_.get_logger().info("[PyBulletSim::reset_simulation] Resetting simulation.")
        # FIXME calling this will cause the simulation to crash because the joint state plugin cannot get the state
        # pb.resetSimulation(physicsClientId=self._uid)
        self.unpause_simulation(Trigger)
        res.success = True
        res.message = "Simulation reset"
        return res

    def pause_simulation(self, req, res):
        """
        Pause the simulation, i.e. disable the stepSimulation() call.
        """
        self.node_.get_logger().info("[PyBulletSim::pause_simulation] Pausing simulation.")
        self._simulation_paused = True
        res.success = True
        res.message = "Simulation paused"
        return res

    def unpause_simulation(self, req, res):
        """
        Continue the simulation, i.e. enable the stepSimulation() call.
        """
        self.node_.get_logger().info("[PyBulletSim::unpause_simulation] Unpausing simulation.")
        self._simulation_paused = False
        res.success = True
        res.message = "Simulation unpaused"
        return res

    @staticmethod
    def add_pybullet_path():
        """
        Adds PyBullets in-built models path to the PyBullet path for easily retrieving the models.
        """
        pb.setAdditionalSearchPath(pb_data.getDataPath())

    def add_search_path(self, path):
        """
        Add the specified directory (absolute path) to PyBullets search path for easily adding models from the path.
        :param path: The absolute path to the directory
        :type path: str
        :return: isdir: Boolean if action was successful
        :rtype: isdir: bool
        """
        assert isinstance(path, str), "[PyBulletSim::add_search_path] Parameter 'path' has an incorrect type."
        if os.path.isdir(path):
            pb.setAdditionalSearchPath(path)
            self.node_.get_logger().info("[PyBulletSim::add_search_path] Added {} to PyBullet path.".format(path))
            return True
        else:
            self.node_.get_logger().error(
                "[PyBulletSim::add_search_path] Error adding to PyBullet path! {} not a directory.".format(path))
            return False
