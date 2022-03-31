from std_msgs.msg import Float64MultiArray


class PosVelEffControl:
    """
    Helper class to receive position, velocity or effort control commands.
    """

    def __init__(self, node, namespace, controller_type):
        """
        Constructor of the PosVelEffControl class.

        :param node: ROS2 node
        :param namespace: namespace of the robot
        :param controller_type: position, velocity, or effort
        :type node: rclpy.node.Node
        :type namespace: str
        :type controller_type: str
        """
        assert controller_type in ["position", "velocity", "effort"]
        node.create_subscription(Float64MultiArray, namespace + controller_type + "_controller/command",
                                 self._pve_control_cb, 10)
        self._cmd = 0.0
        self._data_available = False

    def _pve_control_cb(self, msg):
        """
        Controller callback.

        :param msg: The received message
        :type msg: std_msgs.msg.Float64MultiArray
        """
        self._data_available = True
        self._cmd = msg.data

    def get_last_cmd(self):
        """
        Get last command from subscriber.
        """
        self._data_available = False
        return self._cmd

    @property
    def is_data_available(self):
        """
        Check if new data is available.
        """
        return self._data_available


class Control:
    """
    Control the robot in position, velocity, and torque mode.
    """

    def __init__(self, node, pybullet, robot):
        """
        Constructor of Control plugin.

        :param node: ROS2 node
        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type node: rclpy.node.Node
        :type pybullet: types.ModuleType
        :type robot: pybullet_ros2.PyBulletRobotROS
        """
        self._node = node
        self._pb = pybullet
        self._robot = robot

        # setup subscribers
        self._pc_subscriber = PosVelEffControl(node, self._robot.namespace, "position")
        self._vc_subscriber = PosVelEffControl(node, self._robot.namespace, "velocity")
        self._ec_subscriber = PosVelEffControl(node, self._robot.namespace, "effort")

        self._control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        self._last_command_type = ""
        self._last_torque_command = [0] * self._robot.nb_joints
        self._force_commands = [100] * self._robot.nb_joints

    def execute(self):
        """
        Execution function of the plugin.
        """
        if self._pc_subscriber.is_data_available:
            self._robot.set_torque_control(False)
            self._control_params["controlMode"] = self._pb.POSITION_CONTROL
            self._control_params["targetPositions"] = self._pc_subscriber.get_last_cmd()
            self._control_params["forces"] = self._force_commands
        if self._vc_subscriber.is_data_available:
            self._robot.set_torque_control(False)
            self._control_params["controlMode"] = self._pb.VELOCITY_CONTROL
            self._control_params["targetVelocities"] = self._vc_subscriber.get_last_cmd()
            self._control_params["forces"] = self._force_commands
        if self._ec_subscriber.is_data_available:
            if not self._robot.is_torque_controlled:
                self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices,
                                                   self._pb.VELOCITY_CONTROL,
                                                   forces=[0] * self._robot.nb_joints)
            self._robot.set_torque_control(True)
            self._last_torque_command = self._ec_subscriber.get_last_cmd()

        if self._robot.is_torque_controlled:
            self._control_params["controlMode"] = self._pb.TORQUE_CONTROL
            torques = self._robot.compensate_gravity(feed_forward=self._last_torque_command)
            self._control_params["forces"] = torques
            self._robot.set_applied_motor_torques(torques)

        if "controlMode" in self._control_params.keys():
            self._pb.setJointMotorControlArray(**self._control_params)
