import numpy as np
import pybullet as pb
from state_representation import CartesianState, JointState, Jacobian

from .robot_description import RobotDescription


class Robot(RobotDescription):
    """
    Robot extending from RobotDescription.
    This is a collection of methods that gather 'real-time' information about the robot state and apply joint commands
    to the robot. In other words, those are the methods called (at each time step) in the simulation loop.
    Available methods (for usage, see documentation at function definition):
        - get_joint_state
        - get_link_state
        - get_ee_link_state
        - get_base_state
        - get_jacobian
        - get_inertia
        - compensate_gravity
    """

    def __init__(self, sim_uid, name, urdf_path, fixed_base=True, use_inertia_from_file=True, log_info=print,
                 log_warn=print, log_err=print):
        """
        Constructor of the Robot class.

        :param sim_uid: ID of the physics client
        :param name: Name of the robot
        :param urdf_path: Absolute path of the robot urdf file
        :param fixed_base: Use fixed base robot
        :param use_inertia_from_file: Use inertial tags from urdf file
        :param log_info: Function for info logging
        :param log_warn: Function for warning logging
        :param log_err: Function for error logging
        :type sim_uid: int
        :type name: str
        :type urdf_path: str
        :type fixed_base: bool
        :type use_inertia_from_file: bool
        :type log_info: T
        :type log_warn: T
        :type log_err: T
        """
        assert isinstance(sim_uid, int), "[Robot::init] Argument 'sim_uid' has an incorrect type."
        assert isinstance(name, str), "[Robot::init] Argument 'name' has an incorrect type."
        assert isinstance(urdf_path, str), "[Robot::init] Argument 'urdf_path' has an incorrect type."
        assert isinstance(fixed_base, bool), "[Robot::init] Argument 'fixed_base' has an incorrect type."
        assert isinstance(use_inertia_from_file,
                          bool), "[Robot::init] Argument 'use_inertia_from_file' has an incorrect type."

        self._sim_uid = sim_uid
        super().__init__(self._sim_uid, name, urdf_path, fixed_base, use_inertia_from_file)

        self._log_info = log_info
        self._log_warn = log_warn
        self._log_err = log_err

    def get_joint_state(self):
        """
        Get joint state of the robot (position, velocity, effort).

        :return: The current joint state
        :rtype: JointState
        """
        joint_positions = []
        joint_velocities = []
        joint_efforts = []

        for idx in self.joint_indices:
            joint_state = pb.getJointState(
                self._id, idx, physicsClientId=self._sim_uid)
            joint_positions.append(joint_state[0])
            joint_velocities.append(joint_state[1])
            joint_efforts.append(joint_state[3])
        state = JointState(self.name, self.joint_names)
        state.set_positions(joint_positions)
        state.set_velocities(joint_velocities)
        state.set_torques(joint_efforts)
        return state

    def get_link_state(self, link_id):
        """
        Get state of desired link.

        :param link_id: Index of desired link
        :type link_id: int
        :return: State of the link
        :rtype: CartesianState
        """
        if link_id not in self._all_joint_indices:
            self._log_warn("[Robot::get_link_state] Desired link ID does not exist!")
            return False
        else:
            link_state = pb.getLinkState(self._id, link_id, computeLinkVelocity=1, physicsClientId=self._sim_uid)
            state = CartesianState(self.link_names[link_id], self.name + "_base")
            state.set_position(link_state[4])
            state.set_orientation([link_state[5][-1]] + list(link_state[5][:3]))
            state.set_linear_velocity(link_state[6])
            state.set_angular_velocity(link_state[7])
            return state

    def get_ee_link_state(self):
        """
        Get state of the end effector link (last index in the robot description).

        :return: State of the end effector link
        :rtype: CartesianState
        """
        return self.get_link_state(self._all_joint_indices[-1])

    def get_base_state(self):
        """
        Get state of the base.

        :return: State of the base
        :rtype: CartesianState
        """
        position, orientation = pb.getBasePositionAndOrientation(self._id, self._sim_uid)
        state = CartesianState(self.name + "_base")
        state.set_position(position)
        state.set_orientation([orientation[-1]] + list(orientation[:3]))
        lin_vel, ang_vel = pb.getBaseVelocity(self._id, self._sim_uid)
        state.set_linear_velocity(lin_vel)
        state.set_angular_velocity(ang_vel)
        return state

    def get_jacobian(self, joint_positions):
        """
        Get the Jacobian of the robot for given joint positions.

        :param joint_positions: Joint positions at which the Jacobian should be evaluated
        :type joint_positions: np.ndarray
        :return: Jacobian for the given robot configuration
        :rtype: Jacobian | bool
        """
        if len(joint_positions) is not self.nb_joints:
            self._log_warn("[Robot::get_jacobian] Invalid number of elements in your input.")
            return False
        else:
            linear_jac, angular_jac = pb.calculateJacobian(bodyUniqueId=self._id,
                                                           linkIndex=self._all_joint_indices[-1],
                                                           localPosition=[0.0, 0.0, 0.0],
                                                           objPositions=joint_positions.tolist(),
                                                           objVelocities=[0] * self.nb_joints,
                                                           objAccelerations=[0] * self.nb_joints,
                                                           physicsClientId=self._sim_uid)

        data = np.vstack([np.array(linear_jac), np.array(angular_jac)])
        return Jacobian(self.name, self.joint_names, self.link_names[-1], data, self.name + "_base")

    def get_inertia(self, joint_positions):
        """
        Compute the inertia matrix for given joint positions.

        :param joint_positions: Joint positions at which the inertia matrix should be evaluated
        :type joint_positions: np.ndarray
        :return: Inertia matrix for the given robot configuration
        :rtype: numpy.ndarray | bool
        """
        if len(joint_positions) is not self.nb_joints:
            self._log_warn("[Robot::get_inertia] Invalid number of elements in your input.")
            return False
        else:
            return np.array(pb.calculateMassMatrix(self._id, joint_positions.tolist()))

    def compensate_gravity(self, raw_command):
        """
        Compensate gravity for a given effort command.

        :param raw_command: Raw effort command
        :type raw_command: list of float
        :return: Gravity compensated command
        :rtype: list of float
        """
        joint_state = self.get_joint_state()
        gravity_compensation = pb.calculateInverseDynamics(self._id, joint_state.get_positions().tolist(),
                                                           joint_state.get_velocities().tolist(),
                                                           [0] * len(self.joint_indices), self._sim_uid)
        return [a + b for a, b in zip(raw_command, gravity_compensation)]
