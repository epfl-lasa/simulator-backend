import rospy
from pybullet_simulation import Simulation
from std_srvs.srv import Trigger, TriggerResponse


class ROSSimulation(Simulation):
    """
    ROSSimulation wrapping Simulation for ROS usage and adding services.
    """

    def __init__(self):
        """
        Constructor of the ROSSimulation class. This class creates the PyBullet server / GUI and steps the simulation.
        """
        super().__init__(rospy.get_param("~pybullet_gui", True), rospy.get_param("~gui_options", ""),
                         rospy.get_param("~start_paused", False), rospy.loginfo, rospy.logwarn, rospy.logerr)

        # to avoid warnings in services
        self._simulation_paused = rospy.get_param("~start_paused", False)

        # setup services to pause/unpause and restart simulation
        rospy.Service("~reset_simulation", Trigger, self.reset_simulation)
        rospy.Service("~pause_simulation", Trigger, self.pause_simulation)
        rospy.Service("~unpause_simulation", Trigger, self.unpause_simulation)

    def reset_simulation(self, req):
        """
        Reset the simulation.
        """
        self.pause_simulation(Trigger)
        rospy.loginfo("[ROSSimulation::reset_simulation] Resetting simulation.")
        # FIXME calling this will cause the simulation to crash because the joint state plugin cannot get the state
        # pb.resetSimulation(physicsClientId=self._uid)
        self.unpause_simulation(Trigger)
        res = TriggerResponse(True, "Simulation reset")
        return res

    def pause_simulation(self, req):
        """
        Pause the simulation, i.e. disable the stepSimulation() call.
        """
        rospy.loginfo("[ROSSimulation::pause_simulation] Pausing simulation.")
        self._simulation_paused = True
        res = TriggerResponse(True, "Simulation paused")
        return res

    def unpause_simulation(self, req):
        """
        Continue the simulation, i.e. enable the stepSimulation() call.
        """
        rospy.loginfo("[ROSSimulation::unpause_simulation] Unpausing simulation.")
        self._simulation_paused = False
        res = TriggerResponse(True, "Simulation unpaused")
        return res
