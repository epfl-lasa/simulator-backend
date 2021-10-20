import rospy
from std_msgs.msg import Float64MultiArray


class PosVelEffControl:
    """
    Helper class to receive position, velocity or effort control commands.
    """

    def __init__(self, controller_type, namespace):
        """
        Constructor of the PosVelEffControl class.

        :param controller_type: position, velocity, or effort
        :param namespace: namespace of the robot
        :type controller_type: str
        :type namespace: str
        """
        assert controller_type in ["position", "velocity", "effort"]
        rospy.Subscriber(namespace + controller_type + "_controller/command", Float64MultiArray, self._pve_control_cb,
                         queue_size=1)
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

    def __init__(self, pybullet, robot, **kwargs):
        """
        Constructor of Control plugin.

        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type pybullet: types.ModuleType
        :type robot: pybullet_ros.ROSRobot
        """
        self._pb = pybullet
        self._robot = robot
        self._position_joint_commands = [0] * self._robot.nb_joints
        self._velocity_joint_commands = [0] * self._robot.nb_joints
        self._effort_joint_commands = [0] * self._robot.nb_joints

        # setup subscribers
        self._pc_subscriber = PosVelEffControl("position", self._robot.namespace)
        self._vc_subscriber = PosVelEffControl("velocity", self._robot.namespace)
        self._ec_subscriber = PosVelEffControl("effort", self._robot.namespace)

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
                                                   forces=[0] * len(self._robot.joint_indices))
            self._robot.set_torque_control(True)
            self._last_torque_command = self._ec_subscriber.get_last_cmd()

        if self._robot.is_torque_controlled:
            self._control_params["controlMode"] = self._pb.TORQUE_CONTROL
            torques = self._robot.compensate_gravity(feed_forward=self._last_torque_command)
            self._control_params["forces"] = torques
            self._robot.set_applied_motor_torques(torques)

        if "controlMode" in self._control_params.keys():
            self._pb.setJointMotorControlArray(**self._control_params)
