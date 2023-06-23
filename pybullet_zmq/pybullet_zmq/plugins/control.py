import state_representation as sr
import zmq
from clproto import decode


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
        self._subscriber = zmq_context.socket(zmq.SUB)
        self._subscriber.setsockopt(zmq.CONFLATE, 1)
        self._subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self._subscriber.connect("tcp://" + str(kwargs["URI"]))

        self._control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        self._last_torque_command = [0] * self._robot.nb_joints
        self._force_commands = [100] * self._robot.nb_joints

    def execute(self):
        """
        Execution function of the plugin.
        """
        try:
            msg = self._subscriber.recv(zmq.DONTWAIT)
        except zmq.error.Again:
            return
        if not msg:
            return
        try:
            joint_command = decode(msg)
        except Exception as e:
            print(e)
            return
        if joint_command.get_type() == sr.StateType.JOINT_POSITIONS:
            self._robot.set_torque_control(False)
            self._control_params["controlMode"] = self._pb.POSITION_CONTROL
            self._control_params["targetPositions"] = joint_command.get_positions()
            self._control_params["forces"] = self._force_commands
        elif joint_command.get_type() == sr.StateType.JOINT_VELOCITIES:
            self._robot.set_torque_control(False)
            self._control_params["controlMode"] = self._pb.VELOCITY_CONTROL
            self._control_params["targetVelocities"] = joint_command.get_velocities()
            self._control_params["forces"] = self._force_commands
        elif joint_command.get_type() == sr.StateType.JOINT_TORQUES:
            self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices,
                                               self._pb.VELOCITY_CONTROL,
                                               forces=[0] * self._robot.nb_joints)
            self._robot.set_torque_control(True)
            self._last_torque_command = joint_command.get_torques()

        if self._robot.is_torque_controlled:
            self._control_params["controlMode"] = self._pb.TORQUE_CONTROL
            torques = self._robot.compensate_gravity(feed_forward=self._last_torque_command)
            self._control_params["forces"] = torques
            self._robot.set_applied_motor_torques(torques)

        if "controlMode" in self._control_params.keys():
            self._pb.setJointMotorControlArray(**self._control_params)
