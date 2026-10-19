###	41013-A2

##Libraries
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
from AR4_MK4.AR4_MK4 import AR4_MK4

##Robots
from AR4_MK4 import AR4_MK4
from CRX_10iAL_Paint.CRX_10iAL_Paint import CRXPaint, MOVECRX, Test

##Functions


##Main
def main():
	#Start Swift Env
	env = swift.Swift()
	env.launch(realtime=True)

	#Add Assets to Env

	#CRX
	CRX = CRXPaint()
	CRX.q = np.zeros(6)
	CRX.base = SE3.Trans(0, 0, 0)
	#CRX_vis = CylindricalDHRobotPlot(CRX, cylinder_radius=0.025)
	CRX.q = (0,0,0,0,0,0)
	#CRX_vis.create_cylinders()
	CRX.add_to_env(env)
	#for angle in range(0,90,5):
		#CRX.q = (0,0,angle,0,0,0)
		#env.step(0.15)

	CRXTest = Test()
	CRXTest.q = (0,0,0,0,0,0)
	CRXTest.base = SE3.Trans(0, 0, 0)
	CRXTest_vis = CylindricalDHRobotPlot(CRXTest, cylinder_radius=0.1)
	CRXTest_vis.create_cylinders()
	env.add(CRXTest)
	#env.hold()

	MOVECRX((0.5 ,0.5 ,0.1 ,0 ,0 ,0 ),CRX,env)
	MOVECRX((0.5 ,0.5 ,0.1 ,0.5 ,0.3 ,0 ),CRXTest,env)

	#Turtlebot
	#trt_base = SE3.Trans(0, 0, 0)
	#mount_height = 0.13
	#heading_offset = pi/2
	#trt = Turtlebot3Waffle()
	#trt.base = trt_base * SE3.Rz(heading_offset)
	#trt.add_to_env(env)

	#AR4
	#AR4 = AR4_MK4()
	#AR4.q = np.zeros(6)
	#AR4.base = SE3.Trans(0, 0, mount_height)
	#AR4_vis = CylindricalDHRobotPlot(AR4, cylinder_radius=0.025)
	#AR4_vis.create_cylinders()
	#env.add(AR4)


	env.step(0.05)
	env.hold()

if __name__ == "__main__":
	main()
