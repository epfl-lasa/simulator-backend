from network_interfaces.control_type import ControlType
from network_interfaces.zmq import network


class Control:
    """
    Control the robot in position, velocity, and torque mode.
    """

    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        """
        Constructor of the Control plugin.

        :param zmq_context: ZMQ context to create publisher
        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type zmq_context: zmq.Context
        :type pybullet: types.ModuleType
        :type robot: pybullet_simulation.Robot
        """
        self._pb = pybullet
        self._robot = robot
        self._subscriber = network.configure_subscriber(zmq_context, str(kwargs["URI"]), False)

        self._control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        self._last_command_type = ControlType.UNDEFINED
        self._force_commands = [100] * self._robot.nb_joints

    def execute(self):
        """
        Execution function of the plugin.
        """
        control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        command = network.receive_command(self._subscriber)
        if command:
            if len(set(command.control_type)) > 1:
                raise ValueError("Different control types per joint are currently not allowed. "
                                 "Make sure all the joints have the same control type.")
            if command.control_type[0] == ControlType.POSITION.value:
                self._last_command_type = ControlType.POSITION
                control_params["controlMode"] = self._pb.POSITION_CONTROL
                control_params["targetPositions"] = command.joint_state.get_positions()
                control_params["forces"] = self._force_commands
            elif command.control_type[0] == ControlType.VELOCITY.value:
                self._last_command_type = ControlType.VELOCITY
                control_params["controlMode"] = self._pb.VELOCITY_CONTROL
                control_params["targetVelocities"] = command.joint_state.get_velocities()
                control_params["forces"] = self._force_commands
            elif command.control_type[0] == ControlType.EFFORT.value:
                if self._last_command_type != ControlType.EFFORT:
                    self._last_command_type = ControlType.EFFORT
                    self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices,
                                                       self._pb.VELOCITY_CONTROL,
                                                       forces=[0] * self._robot.nb_joints)
                control_params["controlMode"] = self._pb.TORQUE_CONTROL
                control_params["forces"] = self._robot.compensate_gravity(command.joint_state.get_torques())

        if not command and self._last_command_type == ControlType.EFFORT:
            control_params["controlMode"] = self._pb.TORQUE_CONTROL
            control_params["forces"] = self._robot.compensate_gravity([0.0] * self._robot.nb_joints)

        if "controlMode" in control_params.keys():
            self._pb.setJointMotorControlArray(**control_params)
