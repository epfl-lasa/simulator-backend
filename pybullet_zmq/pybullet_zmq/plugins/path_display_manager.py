from network_interfaces.control_type import ControlType
from network_interfaces.zmq import network
import math
import zmq
import numpy as np


class PathDisplayManager:
    """
    Display trajectories planned or taken by the robot 
    """

    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        """
        Constructor of the Control plugin.

        :param zmq_context: ZMQ context to create publisher
        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type zmq_context: zmq.Context
        :type pybullet: types.ModuleType
        :type robot: pybullet_simulation.Robot
        """
        self._pb = pybullet
        self._robot = robot
        self._subscriber = network.configure_subscriber(zmq_context, str(kwargs["URI"]), True)

        self.ds_traj_id = []
        self.robot_traj_id = []

        self.interval = 10
        self.distance_threshold = 5

        self.interval_robot = 10
        self.first_pose = None
        self.last_displayed_pose = None

    def zmq_try_recv(self):
        try:
            msg_dict = self._subscriber.recv_json(flags=zmq.DONTWAIT)
            return msg_dict
        except zmq.Again as e:
            # No message received
            return None
        except Exception as e:
            print(f"Error receiving ZMQ message: {e}")
            return None

    def display_trajectory(self, msg_dict):
        poses = msg_dict['poses']
        for i in range(len(poses) - 1):
            start_pos = [
                poses[i]['pose']['position']['x'],
                poses[i]['pose']['position']['y'],
                poses[i]['pose']['position']['z'],
            ]
            end_pos = [
                poses[i+1]['pose']['position']['x'],
                poses[i+1]['pose']['position']['y'],
                poses[i+1]['pose']['position']['z'],
            ]

            line_id = self._pb.addUserDebugLine(start_pos, end_pos, msg_dict['color'], lineWidth=2)

            if msg_dict['color'] == [1,0,0]: ## red = ds
                self.ds_traj_id.append(line_id)
            elif msg_dict['color'] == [0,1,0]:  ## green = robot
                self.robot_traj_id.append(line_id)
    
    def calculate_distance(self, pos1, pos2):
        return math.sqrt(
            (pos1['x'] - pos2['x'])**2 +
            (pos1['y'] - pos2['y'])**2 +
            (pos1['z'] - pos2['z'])**2
        )

    def execute(self):
        """
        Execution function of the plugin.
        """

        msg_dict = self.zmq_try_recv()

        if msg_dict is not None:
            
            if len(msg_dict['poses']) >= self.interval:  # received traj to plot 
                    
                    if msg_dict['color'] == [1,0,0]: ## red = ds
                        ## Reduce number of points depending on interval
                        new_poses = msg_dict['poses'][::self.interval]
                        new_msg_dict = {'color' : [1,0,0], 'poses' : []}
                        new_msg_dict['poses'] = new_poses
                        self.display_trajectory(new_msg_dict)

                    elif msg_dict['color'] == [0,1,0]: ## green = robot
                        
                        if len(msg_dict['poses']) >= self.interval_robot:
                            if self.first_pose is None: ## first time receiving msg
                                self.first_pose = msg_dict['poses'][0]
                                self.last_displayed_pose = msg_dict['poses'][-1]
                                self.last_displayed_idx = len(msg_dict['poses'])

                                new_poses = msg_dict['poses'][::self.interval_robot]
                                new_msg_dict = {'color' : [0,1,0], 'poses' : []}
                                new_msg_dict['poses'] = new_poses
                                self.display_trajectory(new_msg_dict)


                            elif (len(msg_dict['poses'])-self.last_displayed_idx) > self.interval_robot: 
                            
                                new_msg_dict = {'color' : [0,1,0], 'poses' : []}
                                new_msg_dict['poses'].append(self.last_displayed_pose)
                                new_msg_dict['poses'].append(msg_dict['poses'][-1])
                                self.last_displayed_pose = msg_dict['poses'][-1]
                                self.last_displayed_idx = len(msg_dict['poses'])
                                self.display_trajectory(new_msg_dict)

            else:
                if msg_dict['color'] == [0,1,0]:
                    for line_id in self.robot_traj_id:
                        self._pb.removeUserDebugItem(line_id)
                    self.robot_traj_id.clear()
        
                elif msg_dict['color'] == [1,0,0]:
                    for line_id in self.ds_traj_id:
                        self._pb.removeUserDebugItem(line_id)
                    self.ds_traj_id.clear()
                    self.first_pose = None
                    self.last_displayed_pose = None
                    self.last_displayed_idx = None
            
            return




