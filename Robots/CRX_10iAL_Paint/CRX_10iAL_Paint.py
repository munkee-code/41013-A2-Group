## CRX-10iA/L Paint Robot

#(abbreviated in future to CRXPaint)

import swift
import os
from math import pi
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import trapezoidal
from ir_support import CylindricalDHRobotPlot
import spatialmath.base as spb
from ir_support.robots.UTSMeshRobot import UTSMeshRobot
from spatialmath import SE3, SO3
from scipy import linalg


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

def CRXRMRC(GOAL, rob, env, steps):

    #Variables
    incriment = 0.05
    JvelMax = 0.5
    damping = 0.03

    #Start and finsih poses
    Start = rob.fkine(rob.q)
    End = (
        SE3.Trans(*GOAL[:3])
        * SE3.RPY(GOAL[3:6], order='xyz')
    )

    #Create Trajectory and matrix to hold poses
    Traj = trapezoidal(0, 1, steps).q
    
    Joints = np.zeros((steps, 6))
    Joints[0, :] = np.array(rob.q, dtype=float)

    #Translation points
    Tstart = Start.t
    Tend = End.t

    #Orientation points
    Rstart = SO3(Start.R)
    Rend = SO3(End.R)


    for i in range(steps - 1):

        #Trajectory progress at current and next timestep
        Now = Traj[i]
        Next = Traj[i + 1]

        #Linear interp translation
        Tnow = (1 - Now) * Tstart + Now * Tend
        Tnext = (1 - Next) * Tstart + Next * Tend

        #INterpolate rotation
        Rnow = Rstart.interp(Rend, Now)
        Rnext = Rstart.interp(Rend, Next)

        #linear velocity
        LinVelocity = (Tnext - Tnow) / incriment

        #rotational velocity
        RotationChange = Rnext * Rnow.inv()
        theta, axis = RotationChange.angvec()
        RotVelocity = axis * theta / incriment
        
        #Actual pose
        CurrentPose = rob.fkine(Joints[i, :])

        #Translation error
        TError = Tnext - CurrentPose.t

        #Rotation error
        RError = Rnext * SO3(CurrentPose.R).inv()
        thetaError, axisError = RError.angvec()
        ErrorVector = axisError * thetaError

        #Compensate for error
        CompLinVel = LinVelocity + TError
        CompRotVel = RotVelocity + ErrorVector

        Velocities = np.r_[CompLinVel, CompRotVel]
        J = rob.jacob0(Joints[i, :])

        #DLS
        JointVel = J.T @ np.linalg.solve(
            J @ J.T + damping**2 * np.eye(6),
            Velocities
        )

        #Limit joint velocities
        JointVel = np.clip(
            JointVel, -JvelMax, JvelMax
        )

        #Update joint angles
        Joints[i + 1, :] = (
            Joints[i, :]
            + incriment * JointVel
        )

        rob.q = Joints[i + 1, :]
        env.step(incriment)

    return