import rospy
import tf
from geometry_msgs.msg import TransformStamped, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry


class Odom:
    def __init__(self, pybullet, robot, **kwargs):
        self._pb = pybullet
        self._robot = robot
        self._link = kwargs["frame_name"]
        self._odom = Odometry()
        self._odom.header.frame_id = "odom"
        self._odom.child_frame_id = self._link

        self._odom_trans = TransformStamped()
        self._odom_trans.header.frame_id = "odom"
        self._odom_trans.child_frame_id = self._link

        self._publisher = rospy.Publisher(robot.namespace + "odom", Odometry, queue_size=10)
        self._tf_broadcaster = tf.TransformBroadcaster(10)

    def execute(self):
        time = rospy.Time.now()
        base_state = self._robot.get_base_state()
        trans = Vector3(base_state.get_position()[0], base_state.get_position()[1], base_state.get_position()[2])
        rot = Quaternion(base_state.get_orientation()[1], base_state.get_orientation()[2],
                         base_state.get_orientation()[3], base_state.get_orientation()[0])

        self._odom_trans.header.stamp = time
        self._odom_trans.transform.translation = trans
        self._odom_trans.transform.rotation = rot

        self._odom.header.stamp = time
        self._odom.pose.pose.position = trans
        self._odom.pose.pose.orientation = rot
        linear = Vector3(base_state.get_linear_velocity()[0], base_state.get_linear_velocity()[1],
                         base_state.get_linear_velocity()[2])
        angular = Vector3(base_state.get_angular_velocity()[0], base_state.get_angular_velocity()[1],
                          base_state.get_angular_velocity()[2])
        twist = Twist(linear, angular)
        self._odom.twist.twist = twist

        self._tf_broadcaster.sendTransformMessage(self._odom_trans)
        self._publisher.publish(self._odom)
