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

    incriment = 0.05
    max_qdot = 0.25
    damping = 0.05
    Kp = 1.0

    # Starting and goal poses
    Start = rob.fkine(rob.q)

    End = (
        SE3.Trans(*GOAL[:3])
        * SE3.RPY(GOAL[3:6], order='xyz')
    )

    # Trajectory progress
    Traj = trapezoidal(0, 1, steps).q

    q_matrix = np.zeros((steps, 6))
    q_matrix[0, :] = np.array(rob.q, dtype=float)

    # Translation endpoints
    Tstart = Start.t
    Tend = End.t

    # Orientation endpoints
    Rstart = SO3(Start.R)
    Rend = SO3(End.R)

    for i in range(steps - 1):

        # Trajectory progress at current and next timestep
        s_now = Traj[i]
        s_next = Traj[i + 1]

        # Desired positions
        T_now = (1 - s_now) * Tstart + s_now * Tend
        T_next = (1 - s_next) * Tstart + s_next * Tend

        # Desired orientations at consecutive timesteps
        R_now = Rstart.interp(Rend, s_now)
        R_next = Rstart.interp(Rend, s_next)

        # Actual end-effector pose
        T_current = rob.fkine(q_matrix[i, :])

        # Desired linear velocity
        v_desired = (T_next - T_now) / incriment

        # Desired angular velocity between consecutive orientations
        R_change = R_next * R_now.inv()
        theta, axis = R_change.angvec()
        w_desired = axis * theta / incriment

        # Position error: desired next position minus actual
        e_position = T_next - T_current.t

        # Orientation error: desired next orientation minus actual
        R_error = R_next * SO3(T_current.R).inv()
        theta_e, axis_e = R_error.angvec()
        e_orientation = axis_e * theta_e

        # Corrected Cartesian velocity
        v_cmd = v_desired + Kp * e_position
        w_cmd = w_desired + Kp * e_orientation

        xdot = np.r_[v_cmd, w_cmd]

        # Full 6x6 Jacobian
        J = rob.jacob0(q_matrix[i, :])

        # Damped least-squares inverse
        q_dot = J.T @ np.linalg.solve(
            J @ J.T + damping**2 * np.eye(6),
            xdot
        )

        # Limit joint velocities
        q_dot = np.clip(
            q_dot, -max_qdot, max_qdot
        )

        # Update joint angles
        q_matrix[i + 1, :] = (
            q_matrix[i, :]
            + incriment * q_dot
        )

        rob.q = q_matrix[i + 1, :]
        env.step(incriment)

    return q_matrix