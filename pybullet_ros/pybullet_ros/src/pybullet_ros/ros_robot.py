from pybullet_simulation import Robot
from pybullet_simulation import test_valid_robot_name
from sensor_msgs.msg import JointState


class ROSRobot(Robot):
    """
    ROSRobot wrapping Robot for ROS usage.
    """

    def __init__(self, sim_uid, name, robot_config, log_info=print, log_warn=print, log_err=print):
        """
        Constructor of the PyBulletRobot class.
        :param sim_uid: ID of the physics client
        :param name: Name of the robot
        :param log_info: Function handle for info logging
        :param log_warn: Function handle for warning logging
        :param log_err: Function handle for error logging
        :type sim_uid: int
        :type name: str
        :type log_info: T
        :type log_warn: T
        :type log_err: T
        """
        test_valid_robot_name(name)
        self._namespace = "/" + name + "/"

        urdf_path = self._get_urdf_path(name, robot_config["urdf"])
        if urdf_path is None:
            return

        super().__init__(sim_uid, name, urdf_path, robot_config["fixed_base"],
                         robot_config["use_inertia_from_file"], log_info, log_warn, log_err)

    @property
    def namespace(self):
        """
        Getter of the robot namespace.
        :rtype: str
        """
        return self._namespace

    def get_joint_state_msg(self):
        joint_state = self.get_joint_state()
        msg = JointState()
        msg.name = joint_state.get_names()
        msg.position = joint_state.get_positions().tolist()
        msg.velocity = joint_state.get_velocities().tolist()
        msg.effort = joint_state.get_torques().tolist()
        return msg

    @staticmethod
    def _get_urdf_path(name, urdf_info):
        """
        Get robot urdf path from parameter server and create urdf file from robot_description param, if necessary.
        Return None if one of the operations failed.
        :param name: name of the robot
        :type name: str
        :rtype: str
        """
        urdf_path = urdf_info["full_path"][:-11] + "_" + name + ".urdf"
        try:
            with open(urdf_path, 'w') as urdf_file:
                urdf_file.write(urdf_info["robot_description"])
            return urdf_path
        except Exception as ex:
            raise Exception(
                "[ROSRobot::get_robot_urdf_path] Failed to create urdf file from param '{}robot_description', " +
                "cannot write file, exiting now: {}".format(name, ex))
