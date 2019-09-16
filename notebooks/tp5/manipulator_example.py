from crocoddyl import *
import pinocchio
import numpy as np

robot = loadTalosArm(modelPath='../models/')
robot.initDisplay(loadModel=True)

from crocoddyl.diagnostic import displayTrajectory
disp = lambda xs, dt: displayTrajectory(robot, xs, dt)
disp.__defaults__ = (.1, )

robot.q0.flat[:] = [2, 1.5, -2, 0, 0, 0, 0]
robot.model.armature[:] = .2
frameId = robot.model.getFrameId('gripper_left_joint')
DT = 1e-2
T = 25

target = np.array([0.4, 0, .4])

robot.viewer.gui.addSphere('world/point', .1, [1, 0, 0, 1])  # radius = .1, RGBA=1001
robot.viewer.gui.applyConfiguration('world/point', target.tolist() + [0, 0, 0, 1])  # xyz+quaternion
robot.viewer.gui.refresh()

# Create the cost functions
costTrack = CostModelFrameTranslation(robot.model, frame=frameId, ref=target)
costXReg = CostModelState(robot.model, StatePinocchio(robot.model))
costUReg = CostModelControl(robot.model)

# Create cost model per each action model
runningCostModel = CostModelSum(robot.model)
terminalCostModel = CostModelSum(robot.model)

# Then let's added the running and terminal cost functions
runningCostModel.addCost(name="pos", weight=1, cost=costTrack)
runningCostModel.addCost(name="xreg", weight=1e-4, cost=costXReg)
runningCostModel.addCost(name="ureg", weight=1e-7, cost=costUReg)
terminalCostModel.addCost(name="pos", weight=1000, cost=costTrack)
terminalCostModel.addCost(name="xreg", weight=1e-4, cost=costXReg)
terminalCostModel.addCost(name="ureg", weight=1e-7, cost=costUReg)

# Create the action model
runningModel = DifferentialActionModelFullyActuated(robot.model, runningCostModel)
terminalModel = DifferentialActionModelFullyActuated(robot.model, terminalCostModel)

# Create the problem
x0 = np.concatenate([m2a(robot.q0), np.zeros(robot.model.nv)])
problem = ShootingProblem(x0, [IntegratedActionModelEuler(runningModel)] * T,
                          IntegratedActionModelEuler(terminalModel))

# Creating the DDP solver for this OC problem, defining a logger
ddp = SolverDDP(problem)
ddp.callback = [CallbackDDPVerbose(), CallbackSolverDisplay(robot, freq=5)]

# Solving it with the DDP algorithm
ddp.solve()

# Visualizing the solution in gepetto-viewer
disp(ddp.xs)

print('Finally reached = ')
print(ddp.datas()[T].differential.costs['pos'].pinocchio.oMf[frameId].translation.T)
