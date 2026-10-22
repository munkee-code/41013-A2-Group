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
from Robots.RS003N.GUI import RobotGui
from robot_utils import move_robot
from ir_support.robots import UR3e

def main():
	env = swift.Swift()
	env.launch(realtime=True)
	
	state_system = 0
	running = True
	
	while running:
		match state_system:
			case 0:

				robot_AR4 = AR4_MK5()
				RS003 = RS003N()
				ur3e = UR3e()
				gui = RobotGui(env, {"AR4_MK5": robot_AR4, "RS003N": RS003, "UR3e": ur3e})
				print("\n=============================")
				print(">> State :: 0 || Spawn Assets")
				print("=============================\n")

				gui.setState(0, "Spawn Assets")

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
				robot_AR4.q = np.zeros(robot_AR4.n)
				robot_AR4.base = SE3.Trans(3.75, 6.9, 0.15) * SE3.Rz(pi)
				robot_AR4.add_to_env(env)
				print(">> Robot AR4_MK5 added to the environment")

				RS003.q = RS003._qtest
				RS003.base = SE3.Trans(3.5, 4.9, 0.15) * SE3.Rz(pi/2)
				RS003.add_to_env(env)
				RS003qHome = np.array(RS003.q, dtype=float)

				#Spawns the UR3 on top of the table
				ur3e.base = SE3.Trans(2.25, 1.55, 0.5)
				ur3e.add_to_env(env)
				qUR3Home = np.asarray(ur3e.q).copy()

				shotglass_start = SE3.Trans(-0.3, -0.30, 0.05) * SE3.Ry(pi)
				shotglass = part_mesh("ShotGlass", pose=RS003.base * shotglass_start)
				env.add(shotglass)
				print(">> Shot glass added to the environment")

				gui.waitForNext("Press 'Next state' in Swift to proceed to state <1>")
				
				state_system = 1
			case 1:
				print("\n=============================")
				print(">> State :: 1 || Pick & Place")
				gui.setState(1, "Pick & Place")
				print("=============================\n")

				# Scene Config := Camera setup (focus on AR4-MK5)
				camera_target = robot_AR4.base.t + np.array([0, 0, 0.35])
				camera_position = robot_AR4.base.t + np.array([1.2, 1.0, 1.2])
				env.set_camera_pose(camera_position, camera_target)

				# v   v   v   IMPLEMENT CODE HERE   v   v   v
				
				## Final Target poses for components
				comp_tar = {
					"FL": 
						(SE3.Trans(3.15, 6.92, 0.4)) * SE3.Ry(pi/2)		# WARNING DO NOT CHANGE!!!
				}

				## Required Poses
				wheel = meshes["wheel_FL"]
				T_wheel = SE3.Trans(3.75, 7.45, 0.3)			# current wheel transform (init)
				T_pickup = T_wheel * SE3.Rx(pi)					# wheel center, with offset (SE3.Rx(pi))
				T_approach = SE3.Trans(0, 0, 0.1) * T_pickup	# approach from above
				T_contact = SE3.Trans(0, 0, 0.04) * T_pickup	# contact point (half of wheel thickness 0.08 / 2)
				T_lift = SE3.Trans(0,0,0.1) * T_contact			# lift the wheel after contact

				T_tar_FL = comp_tar["FL"]

				# WHEEL PICKUP: approach wheel -> contact wheel -> 'attach' wheel
				if move_robot(robot_AR4, T_approach, env, gui=gui) and move_robot(robot_AR4, T_contact, env, steps=10, gui=gui):
					print(">> Wheel pickup position reached")

					# attach wheel to end-effector
					T_ee = robot_AR4.fkine(robot_AR4.q)
					T_ee_wheel = T_ee.inv() * SE3(wheel.T)

					# calculate original mesh offset
					T_mesh_center = SE3(wheel.T).inv() * T_wheel

					depth = 0.065
					T_target = SE3.Trans(-depth, 0, 0) * T_tar_FL
					T_place = T_target * T_mesh_center.inv() * T_ee_wheel.inv()

					# alignment pose
					T_align = SE3.Trans(0.1, 0, 0) * T_place

					# WHEEL PLACEMENT: lift -> align -> place
					# NOTE: using primitive RRT for collision detection
					if (move_robot(robot_AR4, T_lift, env, steps=10, attached_mesh=wheel, grasp_transform=T_ee_wheel, gui=gui)):
						print(">> Wheel Lifted successfully")

						if move_robot(robot_AR4, T_align, env, steps=10, attached_mesh=wheel, grasp_transform=T_ee_wheel, gui=gui):
							print(">> Wheel aligned successfully")

							if move_robot(robot_AR4, T_place, env, steps=10, attached_mesh=wheel, grasp_transform=T_ee_wheel, gui=gui):
								print(">> Wheel placed successfully")
							else:
								print(">> Wheel placement failed")
						else:
							print(">> Wheel alignment failed")
					else:
						print(">> Wheel Lift failed")
				else:
					print(">> Wheel pickup failed")
				
				camera_target = RS003.base.t + np.array([0, 0, 0.35])
				camera_position = RS003.base.t + np.array([1.2, 1.0, 1.2])
				env.set_camera_pose(camera_position, camera_target)

								# Move the assembled car beside the fastening robot.
				car_parts = ["chassis", "wheel_FL"]
				shift = RS003.base.t - robot_AR4.base.t
				T_shift = SE3.Trans(0, shift[1], 0)
				for name in car_parts:
					meshes[name].T = (T_shift * SE3(meshes[name].T)).A
				env.step(0.5)
				print(f">> Car moved next to RS003N (shift = {np.round(shift, 3)})")

				
				# ===   END CODE STATE 1   ===

				gui.waitForNext("Press 'Next state' in Swift to proceed to state <2>")
				state_system = 2

			case 2:
				print("\n=============================")
				print(">> State :: 2 || Fastening")
				print("=============================\n")
				gui.setState(2, "Fastening")

				# v   v   v   IMPLEMENT CODE HERE   v   v   v

				# Pick -> slide in -> screw. run() waits for 'Start RS003N' first.
				fastening = RS003NScrewing(env, RS003, gui, [shotglass])
				if fastening.run():
					print(">> Fastening complete")
				else:
					print(">> Fastening stopped early")

				# ===   END CODE STATE 2   ===

				gui.waitForNext("Press 'Next state' in Swift to proceed to state <3>")
				state_system = 3

			case 3:
				print("\n=============================")
				print(">> State :: 3 || Spray Painting")
				print("=============================\n")
				gui.setState(3, "Spray Painting")
				
				

				# v   v   v   IMPLEMENT CODE HERE   v   v   v


				gui.waitForNext("Press 'Next state' in Swift to proceed to state <4>")
				state_system = 4
			case 4:
				print("\n=============================")
				print(">> State :: 4 || Adhesive Application")
				print("=============================\n")
				gui.setState(4, "Adhesive Application")


				# v   v   v   IMPLEMENT CODE HERE   v   v   v

				# Move the chassis, wheel and shotglass together, in world (x, y, z) metres
				move_x = 0.0
				move_y = -4.8
				move_z = 0.0

				for item in [meshes["chassis"], meshes["wheel_FL"], shotglass]:
					T = item.T.copy()            # the item's current 4x4 pose
					T[0, 3] += move_x            # position column: x
					T[1, 3] += move_y            # y
					T[2, 3] += move_z            # z
					item.T = T                   # put it back
				env.step(0.5)
				print(f">> Car moved by (x, y, z) = ({move_x}, {move_y}, {move_z})")


				# ===   END CODE STATE 4   ===
				
				gui.waitForNext("Press 'Next state' in Swift to proceed to state <5>")
				state_system = 0
			case _:
				print(f">> Error := Unknown system state <{state_system}>, ensure state_system is set correctly")
				running = False

if __name__ == "__main__":
	main()
