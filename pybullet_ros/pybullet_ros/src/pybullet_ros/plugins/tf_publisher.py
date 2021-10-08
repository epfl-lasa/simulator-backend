import numpy as np
import rospy
import tf


class TFPublisher:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        self._br = tf.TransformBroadcaster()

    def execute(self):
        state = self._robot.get_ee_link_state()
        quat = np.roll(state.get_orientation(), 3)
        self._br.sendTransform(state.get_position(), quat, rospy.Time.now(), "ball", "odom")
