###	41013-A2

##Libraries
from matplotlib.table import table
import numpy as np
from pathlib import Path
import swift
from roboticstoolbox import jtraj
from spatialgeometry import Cuboid, Mesh
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from ir_support import CylindricalDHRobotPlot
from ir_support_extra_parts import *
from Robots.AR4_MK5.AR4_MK5 import AR4_MK5
from robot_utils import move_robot
from ir_support.robots import UR3e

##Robots
#from AR4_MK4 import AR4_MK4
from CRX_10iAL_Paint.CRX_10iAL_Paint import CRXPaint, MOVECRX, Test

env = swift.Swift()
env.launch(realtime=True)

CRX = CRXPaint()
CRX.q = CRX.home_q
CRX.base = SE3.Trans(-0.5,0.5,0.1)
cyl_viz = CylindricalDHRobotPlot(CRX, cylinder_radius=0.5, color="blue")
cyl_viz.create_cylinders()
CRX.add_to_env(env)

UR = UR3e()
UR.base = SE3.Trans(-0.5,0.5,0.1)
cyl_viz = CylindricalDHRobotPlot(UR, cylinder_radius=0.5, color="blue")
cyl_viz.create_cylinders()
UR.add_to_env(env)

CRXT = Test()
CRXT.q = (0,0,0,0,0,0)
CRXT.base = SE3.Trans(-0.5,0.5,0.1)
cyl_viz = CylindricalDHRobotPlot(CRXT, cylinder_radius=0.05, color="blue")
cyl_viz.create_cylinders()
env.add(CRXT)
#env.hold()


CRX.q = (0,0,0,0,0,0)
Angle = (pi/2,0,0,0,0,0)
TRAJT = jtraj(CRX.q,Angle,50)
#Move through all joint angles
for q in TRAJT.q:
    CRX.q = q
    env.step(0.05)

CRX.q = (pi/2,0,0,0,0,0)
Angle = (pi/2,pi/2,0,0,0,0)
TRAJT = jtraj(CRX.q,Angle,50)
#Move through all joint angles
for q in TRAJT.q:
    CRX.q = q
    env.step(0.05)

CRX.q = (pi/2,pi/2,0,0,0,0)
Angle = (pi/2,pi/2,pi/2,0,0,0)
TRAJT = jtraj(CRX.q,Angle,50)
#Move through all joint angles
for q in TRAJT.q:
    CRX.q = q
    env.step(0.05)

CRX.q = (pi/2,pi/2,pi/2,0,0,0)
Angle = (pi/2,pi/2,pi/2,pi/2,0,0)
TRAJT = jtraj(CRX.q,Angle,50)
#Move through all joint angles
for q in TRAJT.q:
    CRX.q = q
    env.step(0.05)

CRX.q = (pi/2,pi/2,pi/2,pi/2,0,0)
Angle = (pi/2,pi/2,pi/2,pi/2,pi/2,0)
TRAJT = jtraj(CRX.q,Angle,50)
#Move through all joint angles
for q in TRAJT.q:
    CRX.q = q
    env.step(0.05)

CRX.q = (pi/2,pi/2,pi/2,pi/2,pi/2,0)
Angle = (pi/2,pi/2,pi/2,pi/2,pi/2,pi/2)
TRAJT = jtraj(CRX.q,Angle,50)
#Move through all joint angles
for q in TRAJT.q:
    CRX.q = q
    env.step(0.05)

env.hold()


CRX.q = (0,0,0,0,0,0)
Angle = (0,0,pi/2,0,0,0)
TRAJ = jtraj(CRX.q,Angle,50)
#Move through all joint angles
for q in TRAJ.q:
    CRX.q = q
    env.step(0.05)


CRX.q = (0,0,0,0,0,0)
env.step(0.05)
CRX.q = (0,0,0,0,0,0)
env.step(0.05)
CRX.q = (0,0,0,0,0,0)
CRX.q = (0,0,0,0,0,0)
CRX.q = (0,0,0,0,0,0)

env.hold()
#move_robot(CRX,PoseT,env,steps=50)

