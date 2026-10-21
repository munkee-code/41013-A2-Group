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

##Robots
#from AR4_MK4 import AR4_MK4
from CRX_10iAL_Paint.CRX_10iAL_Paint import CRXPaint, MOVECRX, Test

##Functions


##Main
def main():
	#Start Swift Env
	env = swift.Swift()
	env.launch(realtime=True)
	
	state_system = 0
	running = True
	
	while running:
		match state_system:
			case 0:
				print("\n=============================")
				print(">> State :: 0 || Spawn Assets")
				print("=============================\n")

				cur_path = Path(__file__).resolve().parent

				# dictionary to store paths to asset files, NOTE: please add any new assets here
				asset_paths = {
					"warehouse"	: (cur_path / "Models" / "Warehouse" / "Warehouse.dae"),
					"floor"		: (cur_path / "Models" / "Floor" / "Floor.dae"),
					"conveyor"	: (cur_path / "Models" / "Conveyor" / "Conveyor.stl"),
					"chassis"	: (cur_path / "Models" / "Chassis" / "Chassis.stl"),
					"wheel_FL"	: (cur_path / "Models" / "Components" / "Wheel_1.stl"),
					"comp_conv" : (cur_path / "Models" / "Components" / "Conveyor_Components.stl")
				}

				# Check for asset existence
				FLAG_assets_exist = True
				for name, path in asset_paths.items():
					print(f">> Asset '{name}' path: {path}")
					if not path.exists():
						print(f"!! Warning: Asset '{name}' does not exist at path: {path}")
						FLAG_assets_exist = False
				if not FLAG_assets_exist:
					print("!! Error: One or more assets are missing. Please check the asset paths.")
					return
				else:
					print(">> All assets exist. Proceeding with asset spawning...\n")

				print(">> Spawning assets into the environment...")
				# Dictionary to store the created mesh objects
				# NOTE: to access mesh use: meshes["name"].T
				meshes = {
				#	"warehouse": Mesh(
				#	filename=str(asset_paths["warehouse"]), 
				#	pose=SE3.Rx(pi/2), 
				#	collision=False
				#),
					"floor": Mesh(
					filename=str(asset_paths["floor"]), 
					pose=SE3.Trans(2.7, 4.1, 0.01),
					scale=[0.04, 0.04, 1],
					collision=False
				),
					"conveyor": Mesh(
					filename=str(asset_paths["conveyor"]), 
					pose=SE3.Trans(0,0,0), 
					collision=False
				),
					"chassis": Mesh(
					filename=str(asset_paths["chassis"]), 
					pose=SE3.Trans(0,0,0), 
					collision=False
				),
					"wheel_FL": Mesh(
					filename=str(asset_paths["wheel_FL"]), 
					pose=SE3.Trans(0,0,0), 
					collision=False
				),
					"comp_conv": Mesh(
					filename=str(asset_paths["comp_conv"]), 
					pose=SE3.Trans(0,0,0), 
					collision=False
				),
				}

				for name, mesh in meshes.items():
					env.add(mesh)
					print(f">> {name.capitalize()} added to Swift", flush=True)
				print(">> All assets have been spawned into the environment.\n")

				print(">> Spawning Robots into the environment...")
				robot_AR4 = AR4_MK5()
				robot_AR4.q = np.zeros(robot_AR4.n)
				robot_AR4.base = SE3.Trans(3.7, 6.9, 0.15) * SE3.Rz(pi)
				robot_AR4.add_to_env(env)
				print(">> Robot AR4_MK5 added to the environment")
				
				input("Press Enter to proceed to the next state <1>")
				state_system = 1
			case 1:
				print("\n=============================")
				print(">> State :: 1 || Pick & Place")
				print("=============================\n")

				# v   v   v   IMPLEMENT CODE HERE   v   v   v
				
				# dictionary to store target poses for components
				comp_tar = {
					"FL": (SE3.Trans(3.15, 6.92, 0.4)) * SE3.Rx(pi/2)
				}

				T_car = SE3(2.8, 7.5, 0.23)
				T_tar_FL = T_car * comp_tar["FL"]

				## APPROACHING WHEEL
				# wheel approach poses
				T_wheel = SE3.Trans(3.75, 7.45, 0.3)			# current wheel transform
				T_pickup = T_wheel * SE3.Rx(pi)					# wheel center, with offset (SE3.Rx(pi))
				T_approach = SE3.Trans(0, 0, 0.1) * T_pickup	# approach from above

				# move robot to approach pose
				if move_robot(robot_AR4, T_approach, env):
					print(">> Approach position reached")
				else:
					print(">> Approach failed")

				## LOWER TO CONTACT WHEEL
				wheel_thickness = 0.08
				T_contact = SE3.Trans(0, 0, (wheel_thickness / 2)) * T_pickup
				if move_robot(robot_AR4, T_contact, env, steps=50):
					print(">> Contact position reached")
				else:
					print(">> Contact failed")



				# NO ADDITIONAL CODE PAST THIS POINT FOR STATE 1
				input("Press Enter to proceed to the next state <2>")
				state_system = 2
			case 2:
				print("\n=============================")
				print(">> State :: 2 || Fastening")
				print("=============================\n")
				input("Press Enter to proceed to the next state <3>")

				# v   v   v   IMPLEMENT CODE HERE   v   v   v

				state_system = 3
			case 3:
				print("\n=============================")
				print(">> State :: 3 || Spray Painting")
				print("=============================\n")

				# v   v   v   IMPLEMENT CODE HERE   v   v   v
				CRX = CRXPaint()
				CRX.q = np.zeros(6)
				CRX.base = SE3.Trans(-1,0.5,0.1)
				CRX.add_to_env(env)
				

				input("Press Enter to proceed to the next state <4>")
				state_system = 4
			case 4:
				print("\n=============================")
				print(">> State :: 4 || ...")
				print("=============================\n")
				input("Press Enter to proceed to the next state <_>")
				state_system = -1
			case _:
				print(f">> Error := Unknown system state <{state_system}>, ensure state_system is set correctly")
				running = False

if __name__ == "__main__":
	main()
