from pybullet_simulation import network


class Control:
    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        self._pb = pybullet
        self._robot = robot
        self._subscriber = network.configure_subscriber(zmq_context, "0.0.0.0:" + str(kwargs["URI"]))

        self._control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        self._last_command_type = 0
        self._force_commands = [100] * len(self._robot.joint_indices)

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        command = network.poll_command(self._subscriber)
        # TODO this currently only supports effort control
        if command:
            if self._last_command_type != 3:
                self._last_command_type = 3
                self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices,
                                                   self._pb.VELOCITY_CONTROL,
                                                   forces=[0] * len(self._robot.joint_indices))
            self._control_params["controlMode"] = self._pb.TORQUE_CONTROL
            self._control_params["forces"] = self._robot.compensate_gravity(command.joint_state.get_torques().tolist())
        elif self._last_command_type == 3 and not command:
            self._last_command_type = 0
            self._control_params["controlMode"] = self._pb.VELOCITY_CONTROL
            self._control_params["targetVelocities"] = [0] * len(self._robot.joint_indices)
            self._control_params["forces"] = self._force_commands

        if "controlMode" in self._control_params.keys():
            self._pb.setJointMotorControlArray(**self._control_params)
