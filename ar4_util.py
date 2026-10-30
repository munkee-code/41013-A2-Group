from math import pi
from spatialmath import SE3
from robot_utils import move_robot
import numpy as np
from ar4_collision import load_collision_meshes, plan_wheel_path, execute_wheel_trajectory

def pick_and_place(env, robot, meshes, gui=None):
    """
    Perform the AR4-MK5 wheel pick-and-place operation.

    Args:
        env: Swift simulation environment.
        robot: AR4-MK5 robot instance.
        meshes: Dictionary containing scene meshes.
        gui: Optional GUI instance.

    Returns:
        bool: True if final placement succeeds, otherwise False.
    """

    # save robot's initial joint configuration
    q_home = robot.q.copy()
    T_home = robot.fkine(q_home)

    # Dictionary to store target poses for components
    comp_tar = {
        "FL":
            (SE3.Trans(3.15, 6.92, 0.4)) * SE3.Ry(pi/2)  # WARNING DO NOT CHANGE!!!
    }

    T_tar_FL = comp_tar["FL"]

    ## APPROACHING WHEEL
    # Wheel approach poses
    T_wheel = SE3.Trans(3.75, 7.45, 0.3)
    T_pickup = T_wheel * SE3.Rx(pi)
    T_approach = SE3.Trans(0, 0, 0.1) * T_pickup

    # Move robot to approach pose
    if move_robot(robot, T_approach, env, steps=10):
        print(">> Approach position reached")
    else:
        print(">> Approach failed")

    ## LOWER TO CONTACT WHEEL
    wheel_thickness = 0.08
    T_contact = SE3.Trans(0, 0, (wheel_thickness / 2)) * T_pickup

    if move_robot(robot, T_contact, env, steps=10):
        print(">> Contact position reached")

        T_ee = robot.fkine(robot.q)
        T_wheel_act = SE3(meshes["wheel_FL"].T)
        T_ee_wheel = T_ee.inv() * T_wheel_act

        print(">> Wheel attached to end-effector")
    else:
        print(">> Contact failed")

    ## LIFT WHEEL
    T_mesh = SE3(meshes["wheel_FL"].T)
    T_mesh_center = T_mesh.inv() * T_wheel

    T_lift = SE3.Trans(0, 0, 0.1) * T_contact

    if move_robot(
        robot,
        T_lift,
        env,
        steps=10,
        attached_mesh=meshes["wheel_FL"],
        grasp_transform=T_ee_wheel,
        gui=gui
    ):
        print(">> Wheel lifted successfully")
    else:
        print(">> Lift failed")

    ## ALIGN
    T_target_wheel = T_tar_FL
    T_mesh = SE3(meshes["wheel_FL"].T)
    T_mesh_center = T_mesh.inv() * T_wheel

    depth = 0.065

    T_target_inserted = SE3.Trans(-depth, 0, 0) * T_target_wheel
    T_target_mesh = T_target_inserted * T_mesh_center.inv()
    T_place = T_target_mesh * T_ee_wheel.inv()
    T_align = SE3.Trans(0.1, 0, 0) * T_place

    if move_robot(
        robot,
        T_align,
        env,
        steps=10,
        attached_mesh=meshes["wheel_FL"],
        grasp_transform=T_ee_wheel,
        gui=gui
    ):
        print(">> Wheel aligned successfully")
    else:
        print(">> Alignment failed")

    ## FINAL PLACEMENT
    if move_robot(
        robot,
        T_place,
        env,
        steps=10,
        attached_mesh=meshes["wheel_FL"],
        grasp_transform=T_ee_wheel,
        gui=gui
    ):
        print(">> Wheel placed successfully")
    else:
        print(">> Direct placement failed")

    ## RETURN TO HOME POSITION
    if move_robot(
        robot,
        T_home,
        env,
        steps=10,
        gui=gui
    ):
        print(">> Robot returned to home position")
        return True
    else:
        print(">> Failed to return to home position")
        return False