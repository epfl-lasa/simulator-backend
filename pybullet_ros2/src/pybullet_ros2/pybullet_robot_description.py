import os

import pybullet as pb


class PyBulletRobotDescription(object):
    """
    PyBulletRobotDescription for a robotic manipulator.
    This is a collection of methods that gather and store information from the robot description urdf
    such that this information only has to be requested from the pybullet physics engine once.
    Available methods (for usage, see documentation at function definition):
        - is_initialized
        - id
        - namespace
        - all_joint_names
        - all_joints_dict
        - link_names
        - link_dict
        - get_joint_index_by_name
        - get_link_index_by_name
        - nb_joints
        - joint_indices
        - nb_fixed_joints
        - fixed_joint_indices
    """

    def __init__(self, name, robot_config, uid):
        """
        Constructor of the Robot Description class. Gathers information from the robot urdf description
        regarding joints and links and implements simple getter functions for robot information.
        :param name: name of the robot
        :param uid: server id of pybullet
        :type name: str
        :type uid: int
        """
        self._initialized = False
        self._namespace = "/" + name + "/"
        self._uid = uid

        urdf_path = self._get_urdf_path(name, robot_config["urdf"])
        if urdf_path is None:
            return

        # load robot from URDF model, user decides if inertia is computed automatically by pybullet or custom
        # self collision is on by default
        if robot_config["use_inertia_from_file"]:
            urdf_flags = pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_SELF_COLLISION
        else:
            urdf_flags = pb.URDF_USE_SELF_COLLISION
        self._id = pb.loadURDF(urdf_path, useFixedBase=robot_config["fixed_base"], flags=urdf_flags,
                               physicsClientId=self._uid)

        # get joint and link info
        self._all_joint_info = self._get_joint_info()
        self._all_joint_names = [joint['jointName'].decode("utf-8") for joint in self._all_joint_info]
        self._all_joint_indices = [joint['jointIndex'] for joint in self._all_joint_info]
        self._all_joint_dict = dict(zip(self._all_joint_names, self._all_joint_indices))

        self._all_link_names = [joint['linkName'].decode("utf-8") for joint in self._all_joint_info]
        self._all_link_dict = dict(zip(self._all_link_names, self._all_joint_indices))

        self._movable_joint_indices = self._find_movable_joints()
        self._fixed_joint_indices = self._find_fixed_joints()

        self._joint_lower_position_limits = [pb.getJointInfo(self._id, joint, physicsClientId=self._uid)[8] for joint in
                                             self._movable_joint_indices]
        self._joint_upper_position_limits = [pb.getJointInfo(self._id, joint, physicsClientId=self._uid)[9] for joint in
                                             self._movable_joint_indices]
        self._joint_velocity_limits = [pb.getJointInfo(self._id, joint, physicsClientId=self._uid)[11] for joint in
                                       self._movable_joint_indices]
        self._joint_effort_limits = [pb.getJointInfo(self._id, joint, physicsClientId=self._uid)[10] for joint in
                                     self._movable_joint_indices]

        self._joint_limits = [{'pos_lower': x[0], 'pos_upper': x[1], 'velocity': x[2], 'effort': x[3]}
                              for x in zip(self._joint_lower_position_limits,
                                           self._joint_upper_position_limits,
                                           self._joint_velocity_limits, self._joint_effort_limits)]

        for i, joint_id in enumerate(self._movable_joint_indices):
            pb.resetJointState(self._id, joint_id,
                               (self._joint_upper_position_limits[i] + self._joint_lower_position_limits[i]) / 2)

        # if everything went well, the robot is ready
        self._initialized = True

    @property
    def is_initialized(self):
        """
        Getter of the initialized attribute.
        :rtype: bool
        """
        return self._initialized

    @property
    def id(self):
        """
        Getter of the robot ID.
        :rtype: int
        """
        return self._id

    @property
    def namespace(self):
        """
        Getter of the robot namespace.
        :rtype: str
        """
        return self._namespace

    @property
    def all_joint_names(self):
        """
        Get list with the names of joints in the robot description.
        :return: List of all joint names in the robot description
        :rtype: list of str
        """
        return self._all_joint_names

    @property
    def all_joints_dict(self):
        """
        Get a dict with all joint names mapping to their joint index in the robot description.
        :return: Dict with all joint names and corresponding index
        :rtype: dict[str, int]
        """
        return self._all_joint_dict

    @property
    def link_names(self):
        """
        Get list with the names of all link in the robot description.
        :return: List of all link names in the robot description
        :rtype: list of str
        """
        return self._all_link_names

    @property
    def link_dict(self):
        """
        Get a dict with all link names mapping to their link index in the robot description.
        :return: Dict with all link names and corresponding index
        :rtype: dict[str, int]
        """
        return self._all_link_dict

    def _get_joint_info(self):
        """
        Get a dicts with all the information obtained from pybullet getJointInfo method for all joints.
        :return: getJointInfo() method return values from pybullet for all joints
        :rtype: list of dict
        """
        attribute_list = ['jointIndex', 'jointName', 'jointType',
                          'qIndex', 'uIndex', 'flags',
                          'jointDamping', 'jointFriction', 'jointLowerLimit',
                          'jointUpperLimit', 'jointMaxForce', 'jointMaxVelocity', 'linkName',
                          'jointAxis', 'parentFramePos', 'parentFrameOrn', 'parentIndex']

        joint_information = []
        for idx in range(pb.getNumJoints(self._id, physicsClientId=self._uid)):
            info = pb.getJointInfo(self._id, idx, physicsClientId=self._uid)
            joint_information.append(dict(zip(attribute_list, info)))
        return joint_information

    def get_joint_index_by_name(self, joint_name):
        """
        Get a joint index by the joint's name.
        :param joint_name: Name of joint
        :type joint_name: str
        :return: Joint index of given joint in robot description.
        :rtype: int
        """
        if joint_name in self._all_joint_dict:
            return self._all_joint_dict[joint_name]
        else:
            raise Exception("[PyBulletRobotDescription::get_link_index_by_name] Joint name does not exist!")

    def get_link_index_by_name(self, link_name):
        """
        Get a link index by the link's name.
        :param link_name: Name of link
        :type link_name: str
        :return: Link index of given link in robot description.
        :rtype: int
        """
        if link_name in self._all_link_dict:
            return self._all_link_dict[link_name]
        else:
            raise Exception("[PyBulletRobotDescription::get_link_index_by_name] Link name does not exist!")

    @property
    def nb_joints(self):
        """
        Get number of movable joints in the robot description.
        :return: Number of movable joints in the robot description.
        :rtype: int
        """
        return len(self._movable_joint_indices)

    @property
    def joint_indices(self):
        """
        Get joint indices of all movable joints in the robot description.
        :return: Indices of all movable joints in the robot description.
        :rtype: list of int
        """
        return self._movable_joint_indices

    @property
    def nb_fixed_joints(self):
        """
        Get number of fixed joints in the robot description.
        :return: Number of fixed joints in the robot description
        :rtype: int
        """
        return len(self._fixed_joint_indices)

    @property
    def fixed_joint_indices(self):
        """
        Get joint indices of all movable joints in the robot description.
        :return: Indices of all movable joints in the robot description.
        :rtype: list of int
        """
        return self._fixed_joint_indices

    @property
    def joint_limits(self):
        """
        Get joint position (lower and upper), velocity, and effort limits for all movable joints.
        :return: Joint position (lower and upper), velocity and effort limits for all movable joints.
        :rtype: list of dict
        """
        return self._joint_limits

    def _find_movable_joints(self):
        """
        Get joint indices of all movable joints.
        :return: Indices of all movable joints in the robot description.
        :rtype: list of int
        """
        movable_joints = []
        for i in self._all_joint_indices:
            joint_info = pb.getJointInfo(
                self._id, i, physicsClientId=self._uid)
            # all movable joints have type bigger than 0, -1 is a fixed joint
            if joint_info[3] > -1:
                movable_joints.append(i)
        return movable_joints

    def _find_fixed_joints(self):
        """
        Get joint indices of all fixed joints.
        :return: Indices of all movable joints in the robot description.
        :rtype: list of int
        """
        fixed_joints = []
        for i in self._all_joint_indices:
            joint_info = pb.getJointInfo(
                self._id, i, physicsClientId=self._uid)
            # all fixed joints have type -1
            if joint_info[3] == -1:
                fixed_joints.append(i)
        return fixed_joints

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
            print(
                "[PyBulletRobotDescription::get_robot_urdf_path] Failed to create urdf file from param '{}robot_description', " +
                "cannot write file, exiting now: {}".format(name, ex))
            return None
