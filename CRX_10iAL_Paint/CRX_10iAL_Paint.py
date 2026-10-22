## CRX-10iA/L Paint Robot

#(abbreviated in future to CRXPaint)

import swift
import os
from math import pi
import roboticstoolbox as rtb
from ir_support import CylindricalDHRobotPlot
import spatialmath.base as spb
from ir_support.robots.UTSMeshRobot import UTSMeshRobot
from spatialmath import SE3



class CRXPaint(UTSMeshRobot):
    def __init__(self, base=None):
        links = self._create_DH()
        super().__init__(
            links=links,
            mesh_stem="link",
            mesh_dir=os.path.abspath(os.path.dirname(__file__)),
            name="CRXPaint",
            home_q=[0, 0, 0, 0, 0, 0],
            link3d_names=dict(link0 = 'link0',
                              link1 = 'link1',
                              link2 = 'link2',
                              link3 = 'link3',
                              link4 = 'link4',
                              link5 = 'link5',
                              link6 = 'link6'),
            base=base,
            qtest_transforms = [spb.transl(0,0,0) @ spb.trotx(-pi/2),
                                spb.transl(0,0,0.127) @ spb.trotz(pi) @ spb.trotx(-pi/2),
                                spb.transl(0,0,0.245) @ spb.trotz(pi) @ spb.trotx(-pi/2),
                                spb.transl(0,0,0.955) @ spb.trotx(pi/2) @ spb.trotz(0),
                                spb.transl(0.54,0,0.955) @ spb.trotz(pi) @ spb.troty(pi) @ spb.trotx(-pi/2),
                                spb.transl(0.54,0.15,0.955) @ spb.troty(-pi/2)@ spb.trotx(pi/2) @ spb.trotz(-pi/2),
                                spb.transl(0.7,0.15,0.955) @ spb.troty(pi) @ spb.trotz(pi)]
        )

    def _create_DH(self):
        return [
            #Joint 1
            rtb.RevoluteMDH(d=0.245, a=0, alpha=0, qlim=[-pi,pi], offset=0.),
            #Joint 2
            rtb.RevoluteMDH(d=0, a=0, alpha=pi/2, qlim=[-pi/2,3*pi/2], offset=pi/2),
            #Joint 3
            rtb.RevoluteMDH(d=0, a=0.710, alpha=0, qlim=[-pi,2*pi], offset=0.),
            #Joint 4
            rtb.RevoluteMDH(d=0.540, a=0, alpha=pi/2, qlim=[-19*pi/18,19*pi/18], offset=0.),
            #Joint 5
            rtb.RevoluteMDH(d=0.150, a=0, alpha=pi/2, qlim=[-pi,pi], offset=0.),
            #Joint 6
            rtb.RevoluteMDH(d=0.160, a=0, alpha=-pi/2, qlim=[-5*pi/4,5*pi/4], offset=0.)
        ]

class Test(rtb.DHRobot):
    def __init__(self, base=None):
        links = self._create_DH()
        super().__init__(
            links=links,
            #mesh_stem="link",
            #mesh_dir=os.path.abspath(os.path.dirname(__file__)),
            name="CRXtest",
            base=base,

        )

    def _create_DH(self):
        return [
            #Joint 1
            rtb.RevoluteMDH(d=0.245, a=0, alpha=0, qlim=[-pi,pi]),
            #Joint 2
            rtb.RevoluteMDH(d=0, a=0, alpha=pi/2, qlim=[-pi/2,3*pi/2], offset=pi/2),
            #Joint 3
            rtb.RevoluteMDH(d=0, a=0.710, alpha=0, qlim=[-pi,2*pi]),
            #Joint 4
            rtb.RevoluteMDH(d=0., a=0, alpha=pi/2, qlim=[-19*pi/18,19*pi/18]),
            #Joint 5
            rtb.RevoluteMDH(d=-0.540, a=0, alpha=pi/2, qlim=[-pi,pi]),
            #Joint 6
            rtb.RevoluteMDH(d=0.150, a=0, alpha=-pi/2, qlim=[-5*pi/4,5*pi/4])
        ]

def MOVECRX(GOAL,rob,env):
#Computed matrix of goal
    GOALM = SE3([GOAL[0],GOAL[1],GOAL[2]])*SE3.RPY([GOAL[3],GOAL[4],GOAL[5]],order = 'xyz')
#Computed IKINE joint solution for goal
    GOALIK= rob.ikine_LM(GOALM, q0=rob.q).q
#Create trajectory to goal
    GOALTRAJ = rtb.jtraj(rob.q,GOALIK,100)
#Move through all joint angles
    for q in GOALTRAJ.q:
        rob.q = q
        env.step(0.05)
    return

def CRXRMRC(GOAL,rob,env):
    ik = rob.ikine_LM(GOAL,q0=rob.q)

    #move = 




    return