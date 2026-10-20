from matplotlib.table import table
import numpy as np
from pathlib import Path
import swift
from roboticstoolbox import jtraj
from spatialgeometry import Cuboid, Mesh
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
					"chassis"	: (cur_path / "Models" / "Chassis" / "Chassis.stl")
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
					"warehouse": Mesh(
					filename=str(asset_paths["warehouse"]), 
					pose=SE3.Rx(pi/2), 
					collision=False
				),
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
				)
				}

				for name, mesh in meshes.items():
					env.add(mesh)
					print(f">> {name.capitalize()} added to Swift", flush=True)
				print(">> All assets have been spawned into the environment.\n")

				input("Press Enter to proceed to the next state <1>")
				state_system = 1
			case 1:
				print("\n=============================")
				print(">> State :: 1 || Pick & Place")
				print("=============================\n")

				input("Press Enter to proceed to the next state <2>")
				state_system = 2
			case 2:
				print("\n=============================")
				print(">> State :: 2 || ...")
				print("=============================\n")
				input("Press Enter to proceed to the next state <3>")
				state_system = 3
			case 3:
				print("\n=============================")
				print(">> State :: 3 || ...")
				print("=============================\n")
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
