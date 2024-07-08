from network_interfaces.control_type import ControlType
from network_interfaces.zmq import network
import math
import zmq


class ObstacleManager:
    """
    Control the robot in position, velocity, and torque mode.
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

        self.spheres_id = []
        self.spheres_size = []
        self.matlab_id = []

        self.color = [.7, .1, .1, 1]
        self.color = [.63, .07, .185, 1]

        print("init obstacel manager")
        ## circle movement 
        # self.center = [0,0,0.5]
        # self.radius = 0.5
        # self.angle = 0.0

    # def create_sphere(self, position, radius, color):
    #     sphere = self._pb.createVisualShape(self._pb.GEOM_SPHERE,
    #                                        radius=radius,
    #                                        rgbaColor=color, specularColor=[0, 0, 0, 1])
    #     sphere = self._pb.createMultiBody(baseVisualShapeIndex=sphere,
    #                                      basePosition=position)
    #     self.spheres_id.append(sphere)

    # def initialize_spheres(self, obstacle_array):
    #     for obstacle in obstacle_array:
    #         self.create_sphere(obstacle[0:3], obstacle[3], self.color)

    def delete_sphere(self, idx):
        self._pb.removeBody(self.spheres_id[idx])
        del self.spheres_id[idx]
        del self.spheres_size[idx]
        del self.matlab_id[idx]

    def zmq_try_recv(self):
        try:
            msg_dict = self._subscriber.recv_json(flags=zmq.DONTWAIT)
            # print("Received ZMQ message")
            return msg_dict
        except zmq.Again as e:
            # No message received
            return None
        except Exception as e:
            print(f"Error receiving ZMQ message: {e}")
            return None

    def add_sphere(self, msg_dict):

        matlab_id = msg_dict['id']
        position = [msg_dict['pose']['position']['x'],  msg_dict['pose']['position']['y'],  msg_dict['pose']['position']['z']]
        radius = msg_dict['scale']['x']/2
        color =[msg_dict['color']['r'], msg_dict['color']['g'], msg_dict['color']['b'], msg_dict['color']['a']]
        object_type = msg_dict['type']

        if object_type == 2:
            # Add a simple sphere obstacle
            sphere_collision_shape = self._pb.createCollisionShape(shapeType=self._pb.GEOM_SPHERE, radius=radius)
            sphere_visual_shape = self._pb.createVisualShape(shapeType=self._pb.GEOM_SPHERE, radius=radius, rgbaColor=color)
            new_id = self._pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=sphere_collision_shape, 
                                                baseVisualShapeIndex=sphere_visual_shape, basePosition=position)
        elif object_type == 3:
            # Create a cylinder
            visual_shape_id = self._pb.createVisualShape(shapeType=self._pb.GEOM_CYLINDER, radius=radius, length=2.5, 
                                                         rgbaColor=color)
            #collision_shape_id = self._pb.createCollisionShape(shapeType=self._pb.GEOM_CYLINDER, radius=radius, height=2.5)
            new_id = self._pb.createMultiBody(baseMass=1, baseCollisionShapeIndex=-1 , #collision_shape_id, 
                                        baseVisualShapeIndex=visual_shape_id, basePosition=position)

        elif object_type == 1:
            # Create a visual plane using a large flat box
            scale = [msg_dict['scale']['x'], msg_dict['scale']['y'], msg_dict['scale']['z']]
            visual_plane_shape = self._pb.createVisualShape(shapeType=self._pb.GEOM_BOX, 
                                                            halfExtents=scale,
                                                            rgbaColor=color)
            # Add the visual plane as a multibody with no mass
            new_id = self._pb.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_plane_shape, basePosition=position)

        self.spheres_id.append(new_id)
        self.spheres_size.append(radius)
        self.matlab_id.append(matlab_id)

    def update_sphere(self, msg_dict):

        matlab_id = msg_dict['id']
        new_position = [msg_dict['pose']['position']['x'],  msg_dict['pose']['position']['y'],  msg_dict['pose']['position']['z']]
        new_radius = msg_dict['scale']['x']/2
        new_color = [msg_dict['color']['r'], msg_dict['color']['g'], msg_dict['color']['b'], msg_dict['color']['a']]

        idx = self.matlab_id.index(matlab_id)
        sphere_id = self.spheres_id[idx]
        sphere_radius = self.spheres_size[idx]

        pos, _ = self._pb.getBasePositionAndOrientation(sphere_id)
        
        if pos != new_position: ## update new_position
            self._pb.resetBasePositionAndOrientation(sphere_id, new_position, [1, 0, 0, 0])

        if sphere_radius != new_radius: ## update new size
            print("UPDATING SIZE")
            self.delete_sphere(idx)
            self.add_sphere(msg_dict)

        if new_color == [0,0,0,0]: ##deleted in matlab, remove here too
            print( "Removing Sphere")
            self.delete_sphere(idx)

    def execute(self):
        """
        Execution function of the plugin.
        """

        ##TODO : make thing to toggle collision OR remove it entirely ?

        msg_dict = self.zmq_try_recv()

        if msg_dict is not None:

            new_id = msg_dict['id']

            if new_id in self.matlab_id :  # Object already exists
                self.update_sphere(msg_dict)
            else :  # Object does not exist and must be created
                print("Number of spheres and obstacles do not match")
                #self.delete_spheres()
                #for id in range(existing_id_count + 1, new_id + 1):  # Start from the next ID to the new one
                self.add_sphere(msg_dict)






