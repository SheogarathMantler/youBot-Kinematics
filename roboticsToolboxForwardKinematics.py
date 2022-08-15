import roboticstoolbox as rtb
import math as m
robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(a= -33,  alpha=m.pi/2, d=147, offset=-m.pi/2, flip=True),
        rtb.RevoluteDH(a= 155, alpha=0,      d=0,   offset=m.pi/2,  flip=True),
        rtb.RevoluteDH(a= 135, alpha=0,      d=0,   offset=0,       flip=True),
        rtb.RevoluteDH(a= 0,   alpha=m.pi/2, d=0,   offset=-m.pi/2, flip=True),
        rtb.RevoluteDH(a= 0,   alpha=0,      d=217, offset=-m.pi/2, flip=True),
    ], name= "youbot"    
)
print(robot)
robot.plot([0,m.pi/2,0,0,0], block=True)
