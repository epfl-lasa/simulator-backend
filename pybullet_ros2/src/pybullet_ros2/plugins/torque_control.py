#!/usr/bin/env python3

"""
Position, velocity and effort control for all joints on the robot
"""

from std_msgs.msg import Float64MultiArray


class TorControl:
    """Helper class to receive position, velocity or effort control commands"""

    def __init__(self, node, namespace):
        """
        Create multiple subscribers to receive commands
        :param node: ROS2 node
        :param namespace: namespace of the robot
        :type node: rclpy.node.Node
        :type namespace: str
        """
        self._subscription = node.create_subscription(Float64MultiArray, namespace + "torque_controller/command",
                                                      self._control_cb, 10)
        self._cmd = 0.0
        self._data_available = False

    def _control_cb(self, msg):
        """Callback to receive commands."""
        self._data_available = True
        self._cmd = msg.data

    def get_last_cmd(self):
        """Method to get the last received command"""
        self._data_available = False
        return self._cmd

    def get_is_data_available(self):
        """Method to get flag that indicates if a command has been received"""
        return self._data_available


class TorqueControl:
    def __init__(self, node, pybullet, robot):
        self._node = node
        self._pb = pybullet
        self._robot = robot

        self._last_command_type = ""
        self._force_commands = [100] * len(self._robot.joint_indices)

        # setup subscribers
        self._subscriber = TorControl(node, self._robot.namespace)

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        if self._subscriber.get_is_data_available():
            if self._last_command_type != "effort":
                self._last_command_type = "effort"
                self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices, self._pb.VELOCITY_CONTROL,
                                                   forces=[0] * len(self._robot.joint_indices))
            control_params["controlMode"] = self._pb.TORQUE_CONTROL
            control_params["forces"] = self._robot.compensate_gravity(self._subscriber.get_last_cmd())
        elif self._last_command_type == "effort" and not self._subscriber.get_is_data_available():
            self._last_command_type = ""
            control_params["controlMode"] = self._pb.VELOCITY_CONTROL
            control_params["targetVelocities"] = [0] * len(self._robot.joint_indices)
            control_params["forces"] = self._force_commands

        if "controlMode" in control_params.keys():
            self._pb.setJointMotorControlArray(**control_params)
