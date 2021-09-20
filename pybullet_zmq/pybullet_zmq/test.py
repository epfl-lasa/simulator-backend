import os
import time

import zmq
from pybullet_simulation import Robot
from pybullet_simulation import Simulation
from pybullet_simulation import network


def main():
    context = zmq.Context(1)
    subscriber_address = "0.0.0.0:1602"
    publisher_address = "*:1601"
    command_subscriber, state_publisher = network.configure_sockets(context, subscriber_address, publisher_address)

    desired_frequency = 500.0

    simulation = Simulation(gui=True)

    script_dir = os.path.dirname(os.path.realpath(__file__))
    robot = Robot(simulation.uid, "panda",
                  os.path.join(script_dir, "robot_descriptions/franka_panda_description/urdf/panda_arm.urdf"))

    start = time.time()
    k = 0
    while simulation.is_alive():
        now = time.time()
        simulation.step()

        ee_state = robot.get_ee_link_state()
        joint_state = robot.get_joint_state()
        state = network.StateMessage(ee_state, joint_state)
        network.send_state(state, state_publisher)

        command = network.poll_command(command_subscriber)
        if command:
            print("received command")
            # print(command.control_type)
            print(command.joint_state)

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1
        print("Average rate: ", k / (time.time() - start))


if __name__ == "__main__":
    main()
