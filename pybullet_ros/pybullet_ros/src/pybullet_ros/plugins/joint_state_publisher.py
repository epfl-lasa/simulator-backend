"""
Query robot state and publish position, velocity and torque values to /robot_name/joint_states.
"""
import rospy
from sensor_msgs.msg import JointState


class JointStatePublisher:
    def __init__(self, pybullet, robot):
        self._pb = pybullet
        self._robot = robot
        self._publisher = rospy.Publisher(robot.namespace + "joint_states", JointState, queue_size=10)

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        joint_msg = self._robot.get_joint_state_msg()
        joint_msg.header.stamp = rospy.Time.now()
        self._publisher.publish(joint_msg)
