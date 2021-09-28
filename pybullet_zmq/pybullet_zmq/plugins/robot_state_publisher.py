from network_interfaces.zmq import network
from state_representation import Parameter, StateType


class RobotStatePublisher:
    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        self._pb = pybullet
        self._robot = robot
        self._publisher = network.configure_publisher(zmq_context, str(kwargs["URI"]), False)

    def execute(self):
        joint_state = self._robot.get_joint_state()
        mass = Parameter(self._robot.name + "_mass", self._robot.get_inertia(joint_state.get_positions()),
                         StateType.PARAMETER_MATRIX)
        state = network.StateMessage(self._robot.get_ee_link_state(), joint_state,
                                     self._robot.get_jacobian(joint_state.get_positions()), mass)
        network.send_state(state, self._publisher)
