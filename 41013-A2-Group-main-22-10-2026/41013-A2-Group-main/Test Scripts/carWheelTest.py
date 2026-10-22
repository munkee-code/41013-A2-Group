import numpy as np
import swift
from roboticstoolbox import jtraj
from pathlib import Path
from spatialgeometry import Cuboid, Mesh
# Compatibility workaround for Swift 1.1.0
# with SpatialGeometry 1.4.1
if not hasattr(Cuboid, "_update_pyb"):
    Cuboid._update_pyb = lambda self: None

from spatialmath import SE3
from spatialmath.base import *
from math import pi
from ir_support import CylindricalDHRobotPlot
from Robots.AR4_MK5.AR4_MK5 import AR4_MK5
from ir_support_extra_parts import *

#---------------------------------------------------------------------------------------------------
# SWIFT SIMULATION
#---------------------------------------------------------------------------------------------------
# swift simulation configuration
env = swift.Swift()
env.launch(realtime=True)

# current file path
current_path = Path(__file__).resolve().parent

#---------------------------------------------------------------------------------------------------
# MESHES
#---------------------------------------------------------------------------------------------------
# Asset: chassis
chassis_path = current_path / "Models" / "car_prt_chassis.stl"
chassis_coord = np.array([0, 0, 0])
chassis_mesh = Mesh(
    filename=str(chassis_path),
    pose=SE3.Trans(chassis_coord),
    color="#637EA1",
    collision=False
)
env.add(chassis_mesh)
print(">> Mesh: chassis_mesh added")

# Asset: wheel
wheel_path = current_path / "Models" / "car_prt_wheel.stl"
wheel_coord = np.array([1, 0, 0.1])
wheel_mesh = Mesh(
    filename=str(wheel_path),
    pose=SE3.Trans(wheel_coord),
    color="#353A41",
    collision=False
)
env.add(wheel_mesh)
print(">> Mesh: wheel_mesh added")

#---------------------------------------------------------------------------------------------------
# TARGETS
#---------------------------------------------------------------------------------------------------
# target coordinates for each wheel
wheel_targets = {
    "FL": SE3.Trans(0.30241, -0.5907, 0.14977) * SE3.Rx(pi/2) * SE3.Rz(pi/2),       # Front Left wheel
    "FR": SE3.Trans(-0.30247, -0.5907, 0.14977)* SE3.Rx(pi/2) * SE3.Rz(-pi/2),      # Front Right wheel
    "RL": SE3.Trans(0.30241, 0.60621, 0.14977) * SE3.Rx(pi/2) * SE3.Rz(pi/2),       # Rear Left wheel
    "RR": SE3.Trans(-0.30247, 0.60619, 0.14977) * SE3.Rx(pi/2) * SE3.Rz(-pi/2),     # Rear Right wheel
}
#wheel_targets = {
#    "FL": SE3.Trans(0,0,0),       # Front Left wheel
#    "FR": SE3.Trans(0,0,0),   # Front Right wheel
#    "RL": SE3.Trans(0,0,0),   # Rear Left wheel
#    "RR": SE3.Trans(0,0,0)   # Rear Right wheel
#}

# transformation from car frame to each wheel target
T_car = SE3(chassis_mesh.T)                                  # Initial pose of the car
T_target_FL = T_car * wheel_targets["FL"]               # Example for the Front Left wheel
T_target_FR = T_car * wheel_targets["FR"]               # Example for the Front Right wheel
T_target_RL = T_car * wheel_targets["RL"]               # Example for the Rear Left wheel
T_target_RR = T_car * wheel_targets["RR"]               # Example for the Rear Right wheel

input(">> Press enter to continue")

wheel_mesh.T = T_target_FL.A
env.step(0.05)
input(">> Press enter to continue to the next wheel")
wheel_mesh.T = T_target_FR.A
env.step(0.05)
input(">> Press enter to continue to the next wheel")
wheel_mesh.T = T_target_RL.A
env.step(0.05)
input(">> Press enter to continue to the next wheel")
wheel_mesh.T = T_target_RR.A
env.step(0.05)


#---------------------------------------------------------------------------------------------------
# END OF SCRIPT
#---------------------------------------------------------------------------------------------------
input(">> Press enter to exit simulation")