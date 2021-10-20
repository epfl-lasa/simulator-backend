import rospy
from sensor_msgs.msg import JointState


class JointStatePublisher:
    """
    Query robot state and publish position, velocity and effort values to /robot_name/joint_states.
    """

    def __init__(self, pybullet, robot):
        """
        Constructor of the JointStatePublisher plugin.

        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type pybullet: types.ModuleType
        :type robot: pybullet_ros.ROSRobot
        """
        self._pb = pybullet
        self._robot = robot
        self._publisher = rospy.Publisher(robot.namespace + "joint_states", JointState, queue_size=10)

    def execute(self):
        """
        Execution function of the plugin.
        """
        joint_msg = self._robot.get_joint_state_msg()
        joint_msg.header.stamp = rospy.Time.now()
        self._publisher.publish(joint_msg)
