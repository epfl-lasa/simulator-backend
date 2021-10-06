import rospy
from geometry_msgs.msg import Twist


class BaseControl:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot

        self._cmd = Twist()
        self._subscriber = rospy.Subscriber(self._robot.namespace + "base/cmd_vel", Twist, self._base_control_cb,
                                            queue_size=1)

    def _base_control_cb(self, msg):
        self._cmd = msg

    def execute(self):
        self._pb.resetBaseVelocity(self._robot.id, [self._cmd.linear.x, self._cmd.linear.y, self._cmd.linear.z],
                                   [self._cmd.angular.x, self._cmd.angular.y, self._cmd.angular.z])
