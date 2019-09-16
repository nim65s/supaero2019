import time

import numpy as np
import pinocchio as pin
from pinocchio.utils import eye, se3ToXYZQUAT
from scipy.optimize import fmin_bfgs

# Define an init config
robot.q0 = np.matrix([0, -1.5, 0, 0, 0, 0]).T  # noqa

Mtarget = pin.SE3(eye(3), np.matrix([0.5, 0.1, 0.2]).T)  # x,y,z


def cost(q):
    q = np.matrix(q).T
    M = robot.placement(q, 6)  # noqa
    return np.linalg.norm(pin.log(M.inverse() * Mtarget).vector)


def callback(q):
    q = np.matrix(q).T
    robot.display(q)  # noqa
    gv.applyConfiguration('world/blue', se3ToXYZQUAT(robot.placement(q, 6)))  # noqa
    time.sleep(.1)


robot.display(robot.q0)  # noqa
qopt = fmin_bfgs(cost, robot.q0, callback=callback)  # noqa
