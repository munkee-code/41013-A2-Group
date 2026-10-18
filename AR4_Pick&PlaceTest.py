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
wheel_coord = np.array([1.2, -0.6, 0.05])
wheel_mesh = Mesh(
    filename=str(wheel_path),
    pose=SE3.Trans(wheel_coord),
    color="#353A41",
    collision=False
)
env.add(wheel_mesh)
print(">> Mesh: wheel_mesh added")

# Robot: AR4-MK5
AR4_MK5 = AR4_MK5()
AR4_MK5.q = np.zeros(6)
AR4_MK5.base = SE3(0.8, -0.8, 0) * SE3.Rz(pi/2)
AR4_MK5.add_to_env(env)
print(">> Robot: AR4-MK5 added to environment")
print(">> Robot base: ", AR4_MK5.base.t)

#---------------------------------------------------------------------------------------------------
# TARGETS
#---------------------------------------------------------------------------------------------------
# target coordinates for each wheel (NOTE: relative to global origin)
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

#---------------------------------------------------------------------------------------------------
# PICK UP WHEEL
#---------------------------------------------------------------------------------------------------
# Current pose of the wheel and the approach pose for picking it up
T_wheel = SE3(wheel_mesh.T)
T_grasp_offset = SE3.Ry(pi/2)
T_pickup = T_wheel * T_grasp_offset
T_approach = T_wheel * SE3.Trans(0, 0, 0.05) * T_pickup
print(">> Wheel position: ", T_wheel.t)
print(">> Pickup approach position: ", T_approach.t)
input(">> Press enter to continue")

# Inverse kinematics to find joint configuration for the pickup approach
ik_approach = AR4_MK5.ikine_LM(
    T_approach,
    q0=AR4_MK5.q.copy(),
    #mask=[1,1,1,0,0,0],  # Only consider position for IK solution
    joint_limits=True
)

if ik_approach.success:
    q_approach = ik_approach.q
    T_actual = AR4_MK5.fkine(q_approach)
    print(">> Desired end-effector position: ", T_approach.t)
    print(">> Actual end-effector position: ", T_actual.t)

    # for error checking
    #pos_err = np.linalg.norm(T_approach.t - T_actual.t)
    #print(">> Position error: ", pos_err)
    #print(">> IK solution found:\nRadians: ", ik_approach.q, "\nDegrees: ", np.degrees(ik_approach.q))
    
    ik_pickup = AR4_MK5.ikine_LM(
        T_pickup,
        q0=q_approach,
        #mask=[1,1,1,0,0,0],  # Only consider position for IK solution
        joint_limits=True
    )

    if ik_pickup.success:
        q_pickup = ik_pickup.q
        T_actual_pickup = AR4_MK5.fkine(q_pickup)
        print(">> Desired pickup position: ", T_pickup.t)
        print(">> Actual pickup position: ", T_actual_pickup.t)
    else:
        print(">> IK solution for pickup not found")

    env.step(0.05)
else:
    print(">> IK solution not found")
    #print(">> Reason: ", ik_approach.reason)

input(">> Press enter to continue")

#---------------------------------------------------------------------------------------------------
# END OF SCRIPT
#---------------------------------------------------------------------------------------------------
input(">> Press enter to exit simulation")