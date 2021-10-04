import rospy
from sensor_msgs.msg import JointState


class JointStatePublisher:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        self._publisher = rospy.Publisher(robot.namespace + "joint_states", JointState, queue_size=10)

    def execute(self):
        joint_msg = self._robot.get_joint_state_msg()
        joint_msg.header.stamp = rospy.Time.now()
        self._publisher.publish(joint_msg)
