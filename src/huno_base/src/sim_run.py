import numpy as np
import pybullet as p
import pybullet_data
import time
from ik import leg_ik
from sim import HunoSim
from zmp import ZMPGaitGenerator


zmp = ZMPGaitGenerator()
t, com_x, zmp_x = zmp.generate(num_steps=8)

sim = HunoSim("./urdf/huno.urdf")
p.setGravity(0, 0, 0)
# planeId = p.loadURDF("./urdf/plane.urdf", [0, 0, 0])

dt = 0.01
step_frames = int(0.6 / dt)

for i in range(len(t)):
    # COM 기준 발 위치
    support_leg = "l" if (i // step_frames) % 2 == 0 else "r"
    swing_leg = "r" if support_leg == "l" else "l"

    x = com_x[i] - zmp_x[i]
    z = -0.05

    hip, knee, ankle = leg_ik(x, z)

    sim.set_leg_angles(support_leg, hip, knee, ankle)
    sim.set_leg_angles(swing_leg, -hip, knee, -ankle)

    p.stepSimulation()
    time.sleep(dt)
