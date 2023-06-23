import zmq
from clproto import encode, MessageType


class RobotStatePublisher:
    """
    Query robot state and publish it with the ZMQ socket.
    """

    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        """
        Constructor of the RobotStatePublisher plugin.

        :param zmq_context: ZMQ context to create publisher
        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type zmq_context: zmq.Context
        :type pybullet: types.ModuleType
        :type robot: pybullet_simulation.Robot
        """
        self._pb = pybullet
        self._robot = robot
        self._publisher = zmq_context.socket(zmq.PUB)
        self._publisher.connect("tcp://" + str(kwargs["URI"]))

    def execute(self):
        """
        Execution function of the plugin.
        """
        msg = encode(self._robot.get_joint_state(), MessageType.JOINT_STATE_MESSAGE)
        self._publisher.send(msg)
