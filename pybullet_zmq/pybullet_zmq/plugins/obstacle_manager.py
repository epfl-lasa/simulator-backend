from network_interfaces.control_type import ControlType
from network_interfaces.zmq import network
import math
import zmq


class ObstacleManager:
    """
    Display the obstacles for the robot as either spheres, cylinders or 'planes'
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

        self.obstacles_id = []
        self.obstacles_scale = []
        self.matlab_id = []

        self.color = [.7, .1, .1, 1]
        self.color = [.63, .07, .185, 1]

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
        
    def delete_obstacle(self, idx):
        self._pb.removeBody(self.obstacles_id[idx])
        del self.obstacles_id[idx]
        del self.obstacles_scale[idx]
        del self.matlab_id[idx]

    def add_obstacle(self, msg_dict):

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

        self.obstacles_id.append(new_id)
        self.obstacles_scale.append(radius)
        self.matlab_id.append(matlab_id)

    def update_obstacle(self, msg_dict):

        matlab_id = msg_dict['id']
        new_position = [msg_dict['pose']['position']['x'],  msg_dict['pose']['position']['y'],  msg_dict['pose']['position']['z']]
        new_radius = msg_dict['scale']['x']/2
        new_color = [msg_dict['color']['r'], msg_dict['color']['g'], msg_dict['color']['b'], msg_dict['color']['a']]

        idx = self.matlab_id.index(matlab_id)
        obstacle_id = self.obstacles_id[idx]
        obstacle_scale = self.obstacles_scale[idx]

        pos, _ = self._pb.getBasePositionAndOrientation(obstacle_id)
        
        if pos != new_position: ## update new_position
            self._pb.resetBasePositionAndOrientation(obstacle_id, new_position, [1, 0, 0, 0])

        if obstacle_scale != new_radius: ## update new size
            print("UPDATING SIZE")
            self.delete_obstacle(idx)
            self.add_obstacle(msg_dict)

        if new_color == [0,0,0,0]: ##deleted in matlab, remove here too
            print( "Removing Sphere")
            self.delete_obstacle(idx)

    def execute(self):
        """
        Execution function of the plugin.
        """

        ##TODO : make thing to toggle collision OR remove it entirely ?

        msg_dict = self.zmq_try_recv()

        if msg_dict is not None:

            new_id = msg_dict['id']

            if new_id in self.matlab_id :  # Object already exists
                self.update_obstacle(msg_dict)
            else :  # Object does not exist and must be created
                print("Number of spheres and obstacles do not match")
                #self.delete_obstacles()
                #for id in range(existing_id_count + 1, new_id + 1):  # Start from the next ID to the new one
                self.add_obstacle(msg_dict)






