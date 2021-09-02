import pybullet as pb

from .pybullet_robot_description import PyBulletRobotDescription


class PyBulletRobot(PyBulletRobotDescription):
    """
    PyBulletRobot extending from PyBulletRobotDescription.
    This is a collection of methods that gather 'real-time' information about the robot state and take control over the
    robot. In other words, these are the methods called (at each time step) in the simulation loop.
    Available methods (for usage, see documentation at function definition):
        - get_joint_state
        - get_link_state
        - get_base_state
    """

    def __init__(self, sim_uid, name, urdf_path, fixed_base=True, use_inertia_from_file=True, log_info=print,
                 log_warn=print, log_err=print):
        """
        Constructor of the PyBulletRobot class.
        :param sim_uid: ID of the physics client
        :param name: Name of the robot
        :param urdf_path: Absolute path of the robot urdf file
        :param fixed_base: Use fixed base robot
        :param use_inertia_from_file: Use inertial tags from urdf file
        :param log_info: Function handle for info logging
        :param log_warn: Function handle for warning logging
        :param log_err: Function handle for error logging
        :type sim_uid: int
        :type name: str
        :type urdf_path: str
        :type fixed_base: bool
        :type use_inertia_from_file: bool
        :type log_info: T
        :type log_warn: T
        :type log_err: T
        """
        # assert isinstance(name, str), "[PyBulletRobot::init] Parameter 'name' has an incorrect type."
        # assert isinstance(uid, int), "[PyBulletRobot::init] Parameter 'uid' has an incorrect type."
        self._sim_uid = sim_uid
        super().__init__(self._sim_uid, name, urdf_path, fixed_base, use_inertia_from_file)

        self._log_info = log_info
        self._log_warn = log_warn
        self._log_err = log_err

    def get_joint_state(self, joint_id=None):
        """
        Get joint state(s) (position, velocity, force, effort).
        :param joint_id: Optional parameter, if different from None, then only the joint state of the desired joint is
                         returned (if it exists), otherwise the joint states of all joints are returned.
        :type joint_id: int
        :return: Joint positions, velocities, reaction forces, and efforts of all movable joints given by PyBullet
        :rtype: tuple of float
        """
        if joint_id is None:
            joint_positions = []
            joint_velocities = []
            joint_reaction_forces = []
            joint_efforts = []

            for idx in self.joint_indices:
                joint_state = pb.getJointState(
                    self._id, idx, physicsClientId=self._sim_uid)
                joint_positions.append(joint_state[0])
                joint_velocities.append(joint_state[1])
                joint_reaction_forces.append(joint_state[2])
                joint_efforts.append(joint_state[3])
            return joint_positions, joint_velocities, joint_reaction_forces, joint_efforts

        else:
            if joint_id in self.joint_indices or joint_id in self.fixed_joint_indices:
                joint_state = pb.getJointState(
                    self._id, joint_id, physicsClientId=self._sim_uid)
                joint_positions = joint_state[0]
                joint_velocities = joint_state[1]
                joint_reaction_forces = joint_state[2]
                joint_efforts = joint_state[3]
                return joint_positions, joint_velocities, joint_reaction_forces, joint_efforts
            else:
                self._log_warn("[PyBulletRobotDescription::get_joint_state] Desired joint ID does not exist!")
                return None

    def get_link_state(self, link_id):
        """
        Get state of desired link.
        :param link_id: Index of desired link
        :type link_id: int
        :return: State of the link (cartesian position of link frame in robot description,
                                    cartesian orientation of link frame in robot description in quaternion xyzw,
                                    cartesian linear velocity, cartesian angular velocity)
        :rtype: list of float
        """
        if link_id not in self._all_joint_indices:
            self._log_warn("[PyBulletRobotDescription::get_link_state] Desired link ID does not exist!")
            return False
        else:
            link_state = pb.getLinkState(self._id, link_id, computeLinkVelocity=1, physicsClientId=self._sim_uid)
            return link_state[4], link_state[5], link_state[6], link_state[7]

    def get_base_state(self):
        """
        Get state of the base.
        :return: State of the base (cartesian position, cartesian orientation in quaternion xyzw,
                                    cartesian linear velocity, cartesian angular velocity)
        :rtype: list of float
        """
        return pb.getBasePositionAndOrientation(self._id, self._sim_uid) + pb.getBaseVelocity(self._id, self._sim_uid)
