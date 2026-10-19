from matplotlib.table import table
import numpy as np
import swift
from roboticstoolbox import jtraj
from spatialgeometry import Cuboid
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from ir_support.robots import UR3e
from ir_support_extra_robots.robots import Turtlebot3Waffle
from ir_support import CylindricalDHRobotPlot
from ir_support_extra_parts import *
from AR4_MK5.AR4_MK5 import AR4_MK5

def main():
	env = swift.Swift()
	env.launch(realtime=True)

	state_system = 0
	
	match state_system:
		case 0:
			print("> State :: 0 || Spawn Assets")
		case 1:
			print("> State :: 1 || ...")
		case 2:
			print("> State :: 2 || ...")
		case 3:
			print("> State :: 3 || ...")
		case 4:
			print("> State :: 4 || ...")
		case _:
			print("Unknown system state")



if __name__ == "__main__":
	main()
