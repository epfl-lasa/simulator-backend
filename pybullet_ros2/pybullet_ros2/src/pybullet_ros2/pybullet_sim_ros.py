import os

import rclpy
from std_srvs.srv import Trigger
from pybullet_simulation import Simulation


class PyBulletSimROS(Simulation):
    """
    PyBulletSimROS wrapping PybulletSim for ROS usage and adding services.
    """

    def __init__(self, node):
        """
        Constructor of the PyBulletSimROS class. This class creates the PyBullet server / GUI and steps the simulation.
        :param node: ROS2 node
        :type node: rclpy.node.Node
        """
        assert isinstance(node, rclpy.node.Node), "[PyBulletSimROS::init] Argument 'node' has an incorrect type."
        self.node_ = node

        super().__init__(self.node_.get_parameter("pybullet_gui").get_parameter_value().bool_value,
                         self.node_.get_parameter("gui_options").get_parameter_value().string_value,
                         self.node_.get_parameter("start_paused").get_parameter_value().bool_value,
                         self.node_.get_logger().info,
                         self.node_.get_logger().warn,
                         self.node_.get_logger().error)

        # to avoid warnings in services
        self._simulation_paused = self.node_.get_parameter("start_paused").get_parameter_value().bool_value

        # setup services to pause/unpause and restart simulation
        self.node_.create_service(Trigger, os.path.join(self.node_.get_name(), "reset_simulation"),
                                  self.reset_simulation)
        self.node_.create_service(Trigger, os.path.join(self.node_.get_name(), "pause_simulation"),
                                  self.pause_simulation)
        self.node_.create_service(Trigger, os.path.join(self.node_.get_name(), "unpause_simulation"),
                                  self.unpause_simulation)

    def reset_simulation(self, req, res):
        """
        Reset the simulation.
        """
        self.pause_simulation(Trigger)
        self.node_.get_logger().info("[PyBulletSimROS::reset_simulation] Resetting simulation.")
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
        self.node_.get_logger().info("[PyBulletSimROS::pause_simulation] Pausing simulation.")
        self._simulation_paused = True
        res.success = True
        res.message = "Simulation paused"
        return res

    def unpause_simulation(self, req, res):
        """
        Continue the simulation, i.e. enable the stepSimulation() call.
        """
        self.node_.get_logger().info("[PyBulletSimROS::unpause_simulation] Unpausing simulation.")
        self._simulation_paused = False
        res.success = True
        res.message = "Simulation unpaused"
        return res
