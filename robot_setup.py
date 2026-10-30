# general libraries
import numpy as np
from math import pi
from spatialmath import SE3

# robot libraries
from ir_support import UR3e
from Robots.AR4_MK5.AR4_MK5 import AR4_MK5
from Robots.RS003N.RS003Nv4 import RS003N
from Robots.CRX_10iAL_Paint.CRX_10iAL_Paint import CRXPaint

def setup_robots(env, selected=None):
    """
    Initialise and spawn robots into the swift environment.

    Args:
        env: The swift environment instance.
        selected: Optional list of robot names to initialise. If None, all robots are initialised.
    Returns:
        dict: Dictionary of robot instances keyed by their names.
    """
    if selected is None:
        selected = ["AR4_MK5", "RS003N", "UR3e", "CRX"]

    robots = {}

    print(">> Spawning robots into the environment...")

    if "AR4_MK5" in selected:
        robot = AR4_MK5()
        robot.q = np.zeros(robot.n)
        robot.base = SE3.Trans(3.75, 6.9, 0.15) * SE3.Rz(pi)
        robot.add_to_env(env)
        robots["AR4_MK5"] = robot
        print(">> AR4_MK5 spawned.")

    if "RS003N" in selected:
        robot = RS003N()
        robot.q = robot._qtest
        robot.base = SE3.Trans(3.5, 4.9, 0.15) * SE3.Rz(pi/2)
        robot.add_to_env(env)
        robots["RS003N"] = robot
        print(">> RS003N spawned.")

    if "CRX" in selected:
        robot = CRXPaint()
        robot.q = (pi/2, pi/8, -pi/8, 0, 0, 0)
        robot.base = SE3.Trans(3.75, 3.5, 0)
        robot.add_to_env(env)
        robots["CRX"] = robot
        print(">> CRX spawned.")

    if "UR3e" in selected:
        robot = UR3e()
        robot.q = np.zeros(robot.n)
        robot.base = SE3.Trans(1.9, 1.55, 0.5)
        robot.add_to_env(env)
        robots["UR3e"] = robot
        print(">> UR3e spawned.")

    print(">> All selected robots spawned.")
    return robots