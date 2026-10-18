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
from AR4_MK5.AR4_MK5 import AR4_MK5
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
chassis_path = current_path / "Models" / "Blender" / "car_chassis2.stl"
chassis_mesh = Mesh(
    filename=str(chassis_path),
    pose=SE3.Trans(0, 0, 0),
    color="#637EA1",
    collision=False
)
env.add(chassis_mesh)
print(">> Mesh: chassis_mesh added")

input(">> Press enter to exit simulation")







# target positions for each wheel
#wheel_targets = {
#    "FL": SE3.Trans(1.83, -3.13, 0.82),       # Front Left wheel
#    "FR": SE3.Trans(0, 0, 0),       # Front Right wheel
#    "RL": SE3.Trans(0, 0, 0),       # Rear Left wheel
#    "RR": SE3.Trans(0, 0, 0)        # Rear Right wheel
#}
#
#T_car = SE3.Trans(0, 0, 0)  # Initial pose of the car
#T_target = T_car * wheel_targets["FL"]  # Example for the Front Left wheel
#T_target_FR = T_car * wheel_targets["FR"]  # Example for the Front Right wheel
#T_target_RL = T_car * wheel_targets["RL"]  # Example for the Rear Left wheel
#T_target_RR = T_car * wheel_targets["RR"]  # Example for the Rear Right wheel

