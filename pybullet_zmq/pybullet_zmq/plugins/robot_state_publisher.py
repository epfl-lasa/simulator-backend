from pybullet_simulation import network


class RobotStatePublisher:
    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        self._pb = pybullet
        self._robot = robot
        self._publisher = network.configure_publisher(zmq_context, "*:" + str(kwargs["URI"]))

    def execute(self):
        state = network.StateMessage(self._robot.get_ee_link_state(), self._robot.get_joint_state())
        network.send_state(state, self._publisher)
