# https://github.com/Annin-Robotics/ar4_ros_driver/tree/main/annin_ar4_description/meshes/ar4_mk5

import numpy as np
import swift
from roboticstoolbox import jtraj

from spatialgeometry import Cuboid
# Compatibility workaround for Swift 1.1.0
# with SpatialGeometry 1.4.1
if not hasattr(Cuboid, "_update_pyb"):
    Cuboid._update_pyb = lambda self: None

from spatialmath import SE3
from spatialmath.base import *
from math import pi
from ir_support import CylindricalDHRobotPlot
from AR4_MK4 import AR4_MK4
from ir_support_extra_parts import *

## Swift Simulation initialisation
env = swift.Swift()
env.launch(realtime=True)

## Robot initialisation
robot_AR4 = AR4_MK4()
robot_AR4.q = np.zeros(6)
robot_AR4.base = SE3(0, 0, 0)
robot_AR4_vis = CylindricalDHRobotPlot(robot_AR4, cylinder_radius=0.025, color="blue")
robot_AR4_vis.create_cylinders()
env.add(robot_AR4)
print(">> AR4 MK4 robot added to the environment")

"""
## Add a table to the environment
asset_table = Cuboid(
    scale = [0.5, 0.5, 0.1],
    pose = SE3.Trans(0.4, 0, 0.15),
    color = [0.5, 0.3, 0.1, 1]
)
env.add(asset_table)
print(">> Table added to the environment")
"""
## Add a cube object to the environment (object to be manipulated)
cube_x = 0.4
cube_y = 0
cube_z = 0.25
cube_scale = 0.05
asset_cube = Cuboid(
    scale = [cube_scale, cube_scale, cube_scale],
    pose = SE3.Trans(cube_x, cube_y, cube_z),
    color = [1, 0, 0, 1],
    collision=True
)
env.add(asset_cube)
print(">> Cube added to the environment")

## Environment setup complete
print(">> Environment setup complete")

## Inverse kinematics for approaching the cube
approach_pose = SE3.Trans(cube_x, cube_y, cube_z + 0.1)
result = robot_AR4.ikine_LM(
    approach_pose,
    q0=robot_AR4.q,
    mask=[1, 1, 1, 1, 0, 1],
    joint_limits=True
)

if result.success:
    print(">> IK solution found")
    print(result.q)
    steps = 50
    trajectory = jtraj(robot_AR4.q, result.q, steps).q

    for q in trajectory:
        robot_AR4.q = q
        env.step(0.05)
else:
    print(">> IK solution not found")

## Lower robot to the cube
pickup_pose = SE3.Trans(cube_x, cube_y, cube_z + 0.05) * SE3.Rx(pi)
pickup_result = robot_AR4.ikine_LM(
    pickup_pose,
    q0=robot_AR4.q,
    mask=[1, 1, 1, 0, 1, 0],
    joint_limits=True
)

if pickup_result.success:
    steps = 50
    print(">> Pickup IK solution found")
    print(pickup_result.q)
    trajectory = jtraj(robot_AR4.q, pickup_result.q, steps).q

    for q in trajectory:
        robot_AR4.q = q
        env.step(0.05)
else:
    print(">> Pickup IK solution not found")

#env.hold()
input(">> Press Enter to exit <<")