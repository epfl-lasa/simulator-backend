#!/usr/bin/env python3
import time

import pybullet as p
import pybullet_data as pd

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
offset = 0
ball = p.loadURDF("soccerball.urdf", [0, 0, 1], globalScaling=0.2)
p.changeDynamics(ball, -1, mass=0.2, rollingFriction=0.001, spinningFriction=0.001,
                 restitution=0.9)
p.changeVisualShape(ball, -1, rgbaColor=[0.8, 0.8, 0.8, 1])

ground = p.loadURDF("plane.urdf")
p.changeDynamics(ground, -1, restitution=0.9)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)
timer = time.time()
thrown = False
while p.isConnected():
    # p.getCameraImage(320,200, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    time.sleep(0.5)
    if time.time() - timer > 5 and not thrown:
        thrown = True
        p.resetBaseVelocity(ball, [1, 0, 4], [0, 0, 0])
