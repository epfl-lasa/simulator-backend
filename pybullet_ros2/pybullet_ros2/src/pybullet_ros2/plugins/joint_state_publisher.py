"""
Query robot state and publish position, velocity and effort values to /robot_name/joint_states.
"""

from sensor_msgs.msg import JointState


class JointStatePublisher:
    def __init__(self, node, pybullet, robot):
        self._node = node
        self._pb = pybullet
        self._robot = robot
        self._publisher = node.create_publisher(JointState, robot.namespace + "joint_states", 10)

    def execute(self):
        """
        Execute the plugin. This function is called from main update loop in the pybullet ros node.
        """
        joint_msg = self._robot.get_joint_state_msg()
        joint_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(joint_msg)
