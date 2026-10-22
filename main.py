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
from Robots.RS003N.RS003Nv4 import RS003N
from Robots.RS003N.fastening import RS003NScrewing
from samuel_fix import move_robot

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
				robot_AR4.base = SE3.Trans(3.75, 6.9, 0.15) * SE3.Rz(pi)
				robot_AR4.add_to_env(env)
				print(">> Robot AR4_MK5 added to the environment")

				RS003 = RS003N()
				RS003.q = RS003._qtest
				RS003.base = SE3.Trans(3.5, 4.9, 0.15) * SE3.Rz(pi/2)
				RS003.add_to_env(env)
				RS003qHome = np.array(RS003.q, dtype=float)

				
				input("Press Enter to proceed to the next state <1>")
				state_system = 1
			case 1:
				print("\n=============================")
				print(">> State :: 1 || Pick & Place")
				print("=============================\n")

				# Camera setup - focus on AR4-MK5
				camera_target = robot_AR4.base.t + np.array([0, 0, 0.35])
				camera_position = robot_AR4.base.t + np.array([1.2, 1.0, 1.2])
				env.set_camera_pose(camera_position, camera_target)

				# v   v   v   IMPLEMENT CODE HERE   v   v   v
				
				# dictionary to store target poses for components
				comp_tar = {
					"FL": 
						(SE3.Trans(3.15, 6.92, 0.4)) * SE3.Ry(pi/2)		# WARNING DO NOT CHANGE!!!
				}

				T_tar_FL = comp_tar["FL"]

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
					T_ee = robot_AR4.fkine(robot_AR4.q)
					T_wheel_act = SE3(meshes["wheel_FL"].T)
					T_ee_wheel = T_ee.inv() * T_wheel_act
					print(">> Wheel attached to end-effector")
				else:
					print(">> Contact failed")

				## LIFT WHEEL
				T_mesh = SE3(meshes["wheel_FL"].T)
				T_mesh_center = T_mesh.inv() * T_wheel
				T_lift = SE3.Trans(0,0,0.1) * T_contact
				if move_robot(robot_AR4, T_lift, env, steps=50,
				              attached_mesh=meshes["wheel_FL"], grasp_transform=T_ee_wheel):
					print(">> Wheel lifted successfully")
				else:
					print(">> Lift failed")

				## ALIGN
				T_target_wheel = T_tar_FL
				T_mesh = SE3(meshes["wheel_FL"].T)
				T_mesh_center = T_mesh.inv() * T_wheel

				depth = 0.065
				T_target_inserted = (SE3.Trans(-depth,0,0) * T_target_wheel)
				T_target_mesh = T_target_inserted * T_mesh_center.inv()
				T_place = T_target_mesh * T_ee_wheel.inv()

				T_align = SE3.Trans(0.1, 0, 0) * T_place
				if move_robot(robot_AR4, T_align, env,
				              attached_mesh=meshes["wheel_FL"], grasp_transform=T_ee_wheel):
					print(">> Wheel aligned successfully")
				else:
					print(">> Alignment failed")
				
				## FINAL PLACEMENT
				if move_robot(robot_AR4, T_place, env,
				              attached_mesh=meshes["wheel_FL"],grasp_transform=T_ee_wheel
				):
					print(">> Wheel placed successfully")
				else:
					print(">> Direct placement failed")

				# Move the assembled car beside the fastening robot.
				car_parts = ["chassis", "wheel_FL"]
				shift = RS003.base.t - robot_AR4.base.t
				T_shift = SE3.Trans(0, shift[1], 0)
				for name in car_parts:
					meshes[name].T = (T_shift * SE3(meshes[name].T)).A
				env.step(0.5)
				print(f">> Car moved next to RS003N (shift = {np.round(shift, 3)})")

				camera_target = RS003.base.t + np.array([0, 0, 0.35])
				camera_position = RS003.base.t + np.array([1.2, 1.0, 1.2])
				env.set_camera_pose(camera_position, camera_target)

				# ===   END CODE STATE 1   ===

				input("Press Enter to proceed to the next state <2>")
				state_system = 2
			case 2:
				print("\n=============================")
				print(">> State :: 2 || Fastening")
				print("=============================\n")

				# v   v   v   IMPLEMENT CODE HERE   v   v   v

				fastening = RS003NScrewing(env, RS003)
				fastening.run()

				# ===   END CODE STATE 2   ===

				input("Press Enter to proceed to the next state <3>")
				state_system = 3
			case 3:
				print("\n=============================")
				print(">> State :: 3 || Spray Painting")
				print("=============================\n")

				# v   v   v   IMPLEMENT CODE HERE   v   v   v


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
