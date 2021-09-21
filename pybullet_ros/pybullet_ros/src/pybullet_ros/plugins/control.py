"""
Position, velocity and effort control for all joints on the robot
"""
import rospy
from std_msgs.msg import Float64MultiArray


class PosVelEffControl:
    """Helper class to receive position, velocity or effort control commands"""

    def __init__(self, controller_type, namespace):
        """
        Create subscriber to receive commands
        :param controller_type: position, velocity, or effort
        :param namespace: namespace of the robot
        :type controller_type: str
        :type namespace: str
        """
        assert controller_type in ["position", "velocity", "effort"]
        rospy.Subscriber(namespace + controller_type + "_controller/command", Float64MultiArray, self._pvt_control_cb,
                         queue_size=1)
        self._cmd = 0.0
        self._data_available = False

    def _pvt_control_cb(self, msg):
        self._data_available = True
        self._cmd = msg.data

    def get_last_cmd(self):
        self._data_available = False
        return self._cmd

    def get_is_data_available(self):
        return self._data_available


class Control:
    def __init__(self, pybullet, robot, **kwargs):
        self._pb = pybullet
        self._robot = robot
        self._position_joint_commands = [0] * len(self._robot.joint_indices)
        self._velocity_joint_commands = [0] * len(self._robot.joint_indices)
        self._effort_joint_commands = [0] * len(self._robot.joint_indices)

        # the max force to apply to the joint, used in velocity control
        if "max_effort" in kwargs.keys():
            self._force_commands = [kwargs["max_torque"]] * len(self._robot.joint_indices)
        else:
            self._force_commands = [100] * len(self._robot.joint_indices)

        self._last_command_type = ""

        # setup subscribers
        self._pc_subscriber = PosVelEffControl("position", self._robot.namespace)
        self._vc_subscriber = PosVelEffControl("velocity", self._robot.namespace)
        self._tc_subscriber = PosVelEffControl("effort", self._robot.namespace)

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop.
        """
        control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        if self._last_command_type == "effort" and not self._tc_subscriber.get_is_data_available():
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
        if self._tc_subscriber.get_is_data_available():
            if self._last_command_type != "effort":
                self._last_command_type = "effort"
                self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices,
                                                   self._pb.VELOCITY_CONTROL,
                                                   forces=[0] * len(self._robot.joint_indices))
            control_params["controlMode"] = self._pb.TORQUE_CONTROL
            control_params["forces"] = self._robot.compensate_gravity(self._tc_subscriber.get_last_cmd())

        if "controlMode" in control_params.keys():
            self._pb.setJointMotorControlArray(**control_params)
