import numpy as np
import rospy
import tf
from state_representation import CartesianPose
from std_srvs.srv import Trigger, TriggerResponse


class Ball:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        self._listener = tf.TransformListener()
        self._attached = False
        self._let_go_service = rospy.Service("/ball/let_go", Trigger, self._let_go)
        self._attach_service = rospy.Service("/ball/attach", Trigger, self._attach)

        self._ball_to_base = CartesianPose("ball", self._pb.getJointInfo(self._robot.id, 0)[-3], "ball_base")
        self._grip_transform = CartesianPose("ball", "panda_link8")
        self._grip_transform.set_position(0, 0, 0.05)

    def _let_go(self, request):
        self._attached = False
        self._pb.changeDynamics(self._robot.id, -1, mass=10.0)
        return TriggerResponse(success=True, message="Throwing ball")

    def _attach(self, request):
        self._attached = True
        self._pb.changeDynamics(self._robot.id, -1, mass=0.01)
        return TriggerResponse(success=True, message="Attaching ball")

    def execute(self):
        if self._attached:
            try:
                trans, rot = self._listener.lookupTransform("odom", "panda_link8", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                print(ex)
                return
            panda_in_odom = CartesianPose("panda_link8", "odom")
            panda_in_odom.set_position(trans)
            panda_in_odom.set_orientation([rot[3]] + rot[:3])
            ball_base_in_odom = panda_in_odom * self._grip_transform * self._ball_to_base.inverse()
            quat = np.roll(ball_base_in_odom.get_orientation(), 3)
            self._pb.resetBasePositionAndOrientation(self._robot.id, ball_base_in_odom.get_position().tolist(),
                                                     quat.tolist())
