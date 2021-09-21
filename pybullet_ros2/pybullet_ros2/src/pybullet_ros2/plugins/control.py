"""
Position, velocity and effort control for all joints on the robot
"""

from std_msgs.msg import Float64MultiArray


class PosVelEffControl:
    """Helper class to receive position, velocity or effort control commands"""

    def __init__(self, controller_type, node, namespace):
        """
        Create multiple subscribers to receive commands
        :param controller_type: position, velocity, or effort
        :param node: ROS2 node
        :param namespace: namespace of the robot
        :type controller_type: str
        :type node: rclpy.node.Node
        :type namespace: str
        """
        assert controller_type in ["position", "velocity", "effort"]
        node.create_subscription(Float64MultiArray, namespace + controller_type + "_controller/command",
                                 self._pve_control_cb, 1)
        self._cmd = 0.0
        self._data_available = False

    def _pve_control_cb(self, msg):
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


class Control:
    def __init__(self, node, pybullet, robot):
        self._node = node
        self._pb = pybullet
        self._robot = robot
        # lists to save last received command
        self._position_joint_commands = [0] * len(self._robot.joint_indices)
        self._velocity_joint_commands = [0] * len(self._robot.joint_indices)
        self._effort_joint_commands = [0] * len(self._robot.joint_indices)
        # the max force to apply to the joint, used in velocity control
        # max_effort = rospy.get_param("~max_effort", 100.0) # TODO
        self._force_commands = [100] * len(self._robot.joint_indices)
        self._last_command_type = ""

        # setup subscribers
        self._pc_subscriber = PosVelEffControl("position", node, self._robot.namespace)
        self._vc_subscriber = PosVelEffControl("velocity", node, self._robot.namespace)
        self._ec_subscriber = PosVelEffControl("effort", node, self._robot.namespace)

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        if self._last_command_type == "effort" and not self._ec_subscriber.get_is_data_available():
            self._last_command_type = ""
            control_params["controlMode"] = self._pb.VELOCITY_CONTROL
            control_params["targetVelocities"] = [0] * len(self._robot.joint_indices)
            control_params["forces"] = self._force_commands
        if self._pc_subscriber.get_is_data_available():
            self._last_command_type = "position"
            control_params["controlMode"] = self._pb.POSITION_CONTROL
            control_params["targetPositions"] = self._pc_subscriber.get_last_cmd()
            control_params["forces"] = self._force_commands
        if self._vc_subscriber.get_is_data_available():
            self._last_command_type = "velocity"
            control_params["controlMode"] = self._pb.VELOCITY_CONTROL
            control_params["targetVelocities"] = self._vc_subscriber.get_last_cmd()
            control_params["forces"] = self._force_commands
        if self._ec_subscriber.get_is_data_available():
            if self._last_command_type != "effort":
                self._last_command_type = "effort"
                self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices,
                                                   self._pb.VELOCITY_CONTROL,
                                                   forces=[0] * len(self._robot.joint_indices))
            control_params["controlMode"] = self._pb.TORQUE_CONTROL
            control_params["forces"] = self._robot.compensate_gravity(self._ec_subscriber.get_last_cmd())

        if "controlMode" in control_params.keys():
            self._pb.setJointMotorControlArray(**control_params)
