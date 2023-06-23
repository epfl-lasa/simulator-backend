import time

import zmq
from pybullet_simulation import Robot
from pybullet_simulation import Simulation
import clproto


def main():
    context = zmq.Context(1)
    state_publisher = context.socket(zmq.PUB)
    state_publisher.connect("tcp://0.0.0.0:1601")
    command_subscriber = context.socket(zmq.SUB)
    command_subscriber.setsockopt(zmq.CONFLATE, 1)
    command_subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    command_subscriber.connect("tcp://0.0.0.0:1602")

    desired_frequency = 500.0

    simulation = Simulation(gui=False)

    robot = Robot(simulation.uid, "panda", "/home/ros2/robot_descriptions/franka_panda_description/urdf/panda_arm.urdf")

    start = time.time()
    k = 0
    while simulation.is_alive():
        now = time.time()
        simulation.step()

        msg = clproto.encode(robot.get_joint_state(), clproto.MessageType.JOINT_STATE_MESSAGE)
        state_publisher.send(msg)

        try:
            msg = command_subscriber.recv(zmq.DONTWAIT)
        except zmq.error.Again:
            return
        if not msg:
            return
        try:
            joint_command = clproto.decode(msg)
            print("received command")
            print(joint_command)
        except Exception as e:
            print(e)
            return

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1
        print("Average rate: ", k / (time.time() - start))


if __name__ == "__main__":
    main()
