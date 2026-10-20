
import numpy as np
import swift
from roboticstoolbox import jtraj
from pathlib import Path
from spatialgeometry import Cuboid, Mesh
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from ir_support import CylindricalDHRobotPlot
from AR4_MK5.AR4_MK5 import AR4_MK5
from ir_support_extra_parts import *

# Compatibility workaround for Swift 1.1.0
# with SpatialGeometry 1.4.1
if not hasattr(Cuboid, "_update_pyb"):
    Cuboid._update_pyb = lambda self: None

if not hasattr(Mesh, "_update_pyb"):
    Mesh._update_pyb = lambda self: None

#---------------------------------------------------------------------------------------------------
# SWIFT SIMULATION
#---------------------------------------------------------------------------------------------------

env = swift.Swift()
env.launch(realtime=True)

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
robot = AR4_MK5()
robot.q = np.zeros(6)
robot.base = SE3(0.8, -0.8, 0) * SE3.Rz(pi/2)
robot.add_to_env(env)

print(">> Robot: AR4-MK5 added to environment")
print(">> Robot base:", robot.base.t)

#---------------------------------------------------------------------------------------------------
# TARGETS
#---------------------------------------------------------------------------------------------------

# Wheel mounting poses relative to chassis frame
wheel_targets = {
    "FL": (
        SE3.Trans(0.30241, -0.5907, 0.14977)
        * SE3.Rz(pi/2)
        * SE3.Rx(pi/2)
    ),

    "FR": (
        SE3.Trans(-0.30247, -0.5907, 0.14977)
        * SE3.Rx(pi/2)
        * SE3.Rz(-pi/2)
    ),

    "RL": (
        SE3.Trans(0.30241, 0.60621, 0.14977)
        * SE3.Rx(pi/2)
        * SE3.Rz(pi/2)
    ),

    "RR": (
        SE3.Trans(-0.30247, 0.60619, 0.14977)
        * SE3.Rx(pi/2)
        * SE3.Rz(-pi/2)
    )
}

# Transform mounting poses into world coordinates
T_car = SE3(chassis_mesh.T)

T_target_FL = T_car * wheel_targets["FL"]
T_target_FR = T_car * wheel_targets["FR"]
T_target_RL = T_car * wheel_targets["RL"]
T_target_RR = T_car * wheel_targets["RR"]

input(">> Press Enter to continue")

#---------------------------------------------------------------------------------------------------
# PICK UP WHEEL
#---------------------------------------------------------------------------------------------------

# Current wheel transformation
T_wheel = SE3(wheel_mesh.T)

# End effector faces downwards
T_grasp_offset = SE3.Rx(pi)

# Pickup pose at wheel centre
T_pickup = T_wheel * T_grasp_offset

# Approach 40 mm above wheel centre
T_approach = SE3.Trans(0, 0, 0.04) * T_pickup

print(">> Wheel position:", T_wheel.t)
print(">> Pickup approach position:", T_approach.t)
print(">> Pickup position:", T_pickup.t)

input(">> Press Enter to continue")

#---------------------------------------------------------------------------------------------------
# INVERSE KINEMATICS FOR PICKUP APPROACH
#---------------------------------------------------------------------------------------------------

ik_approach = robot.ikine_LM(
    T_approach,
    q0=robot.q.copy(),
    joint_limits=True
)

if ik_approach.success:

    q_approach = ik_approach.q
    print(">> Approach IK solution found")

    input(">> Press Enter to approach wheel")

    trajectory = jtraj(robot.q.copy(), q_approach, 100)

    for q in trajectory.q:
        robot.q = q
        env.step(0.05)

    #-----------------------------------------------------------------------------------------------
    # INVERSE KINEMATICS FOR PICKUP
    #-----------------------------------------------------------------------------------------------

    ik_pickup = robot.ikine_LM(
        T_pickup,
        q0=robot.q.copy(),
        joint_limits=True
    )

    if ik_pickup.success:

        q_pickup = ik_pickup.q
        print(">> Pickup IK solution found")

        input(">> Press Enter to lower onto wheel")

        trajectory = jtraj(robot.q.copy(), q_pickup, 50)

        for q in trajectory.q:
            robot.q = q
            env.step(0.05)

        #-------------------------------------------------------------------------------------------
        # ATTACH WHEEL TO END EFFECTOR
        #-------------------------------------------------------------------------------------------

        input(">> Press Enter to attach wheel")

        T_ee = robot.fkine(robot.q)
        T_wheel_current = SE3(wheel_mesh.T)

        # Preserve wheel pose relative to end effector
        T_ee_wheel = T_ee.inv() * T_wheel_current

        print(">> Wheel attached")

        #-------------------------------------------------------------------------------------------
        # INVERSE KINEMATICS FOR LIFT
        #-------------------------------------------------------------------------------------------

        # Lift 150 mm above pickup position
        T_lift = SE3.Trans(0, 0, 0.15) * T_pickup

        ik_lift = robot.ikine_LM(
            T_lift,
            q0=robot.q.copy(),
            joint_limits=True
        )

        if ik_lift.success:

            q_lift = ik_lift.q
            print(">> Lift IK solution found")

            input(">> Press Enter to lift wheel")

            trajectory = jtraj(robot.q.copy(), q_lift, 100)

            for q in trajectory.q:

                robot.q = q

                # Wheel follows end effector
                T_ee = robot.fkine(robot.q)
                wheel_mesh.T = (T_ee * T_ee_wheel).A

                env.step(0.05)

            print(">> Wheel lifted")

            #---------------------------------------------------------------------------------------
            # INVERSE KINEMATICS FOR FRONT LEFT WHEEL PLACEMENT
            #---------------------------------------------------------------------------------------

            # Calculate end-effector pose needed to
            # position the attached wheel at T_target_FL
            T_ee_place = T_target_FL * T_ee_wheel.inv()

            print(">> Front-left target position:", T_target_FL.t)
            print(">> Front-left target rotation:")
            print(T_target_FL.R)

            ik_place = robot.ikine_LM(
                T_ee_place,
                q0=robot.q.copy(),
                joint_limits=True
            )

            if ik_place.success:

                q_place = ik_place.q
                print(">> Placement IK solution found")

                #-----------------------------------------------------------------------------------
                # MOVE DIRECTLY TO FRONT LEFT WHEEL TARGET
                #-----------------------------------------------------------------------------------

                input(">> Press Enter to move wheel to front-left mount")

                trajectory = jtraj(
                    robot.q.copy(),
                    q_place,
                    100
                )

                for q in trajectory.q:

                    robot.q = q

                    # Wheel follows end effector
                    T_ee = robot.fkine(robot.q)
                    wheel_mesh.T = (T_ee * T_ee_wheel).A

                    env.step(0.05)

                print(">> Wheel moved to front-left mounting position")

                #-----------------------------------------------------------------------------------
                # RELEASE WHEEL
                #-----------------------------------------------------------------------------------

                input(">> Press Enter to release wheel")

                # Snap wheel to exact final mounting pose
                wheel_mesh.T = T_target_FL.A
                env.step(0.05)

                print(">> Front-left wheel placed successfully")

            else:
                print(">> Placement IK solution not found")
                print(">> Reason:", ik_place.reason)

        else:
            print(">> Lift IK solution not found")
            print(">> Reason:", ik_lift.reason)

    else:
        print(">> Pickup IK solution not found")
        print(">> Reason:", ik_pickup.reason)

else:
    print(">> Approach IK solution not found")
    print(">> Reason:", ik_approach.reason)

#---------------------------------------------------------------------------------------------------
# END OF SCRIPT
#---------------------------------------------------------------------------------------------------

input(">> Press Enter to exit simulation")
