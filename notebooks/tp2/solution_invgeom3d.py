import time

import numpy as np
from pinocchio.utils import se3ToXYZQUAT
from scipy.optimize import fmin_bfgs

# Define an init config
robot.q0 = np.matrix([0, -1.5, 0, 0, 0, 0]).T  # noqa

target = np.matrix([0.5, 0.1, 0.2]).T  # x,y,z


def cost(q):
    q = np.matrix(q).T
    p = robot.placement(q, 6).translation  # noqa
    return np.linalg.norm(p - target)


def callback(q):
    q = np.matrix(q).T
    robot.display(q)  # noqa
    gv.applyConfiguration('world/blue', se3ToXYZQUAT(robot.placement(q, 6)))  # noqa
    time.sleep(.1)


robot.display(robot.q0)  # noqa
qopt = fmin_bfgs(cost, robot.q0, callback=callback)  # noqa
