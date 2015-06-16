import pinocchio as se3
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
robot = RobotWrapper('/home/nmansard/src/pinocchio/pinocchio/models/romeo.urdf')

se3.geometry(robot.model,robot.data,robot.q0)
se3.rnea(robot.model,robot.data,robot.q0,robot.v0,robot.v0)

q = se3.reflexV(zero(3),True)
print q

