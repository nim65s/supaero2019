WITH_UR5 = True
WITH_ROMEO = False

if WITH_UR5:
    from pinocchio.robot_wrapper import RobotWrapper

    urdf = '/opt/openrobots/share/ur5_description/urdf/ur5_gripper.urdf'
    robot = RobotWrapper.BuildFromURDF(urdf)

if WITH_ROMEO:
    from pinocchio.romeo_wrapper import RomeoWrapper

    urdf = '/opt/openrobots/share/romeo_description/urdf/romeo.urdf'
    robot = RomeoWrapper.BuildFromURDF(urdf)

robot.initDisplay(loadModel=True)
robot.display(robot.q0)
