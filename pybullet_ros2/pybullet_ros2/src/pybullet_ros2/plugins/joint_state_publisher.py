from sensor_msgs.msg import JointState


class JointStatePublisher:
    """
    Query robot state and publish position, velocity and effort values to /robot_name/joint_states.
    """

    def __init__(self, node, pybullet, robot):
        """
        Constructor of the JointStatePublisher plugin.

        :param node: ROS2 node
        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type node: rclpy.node.Node
        :type pybullet: types.ModuleType
        :type robot: pybullet_ros2.PyBulletRobotROS
        """
        self._node = node
        self._pb = pybullet
        self._robot = robot
        self._publisher = node.create_publisher(JointState, robot.namespace + "joint_states", 10)

    def execute(self):
        """
        Execution function of the plugin.
        """
        joint_msg = self._robot.get_joint_state_msg()
        joint_msg.header.stamp = self._node.get_clock().now().to_msg()
        self._publisher.publish(joint_msg)
