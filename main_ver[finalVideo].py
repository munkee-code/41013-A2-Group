# Project: 41013-A2, version: Final Video
# Libraries
## General libraries
import numpy as np
import swift
from roboticstoolbox import jtraj
from spatialmath import SE3
from spatialmath.base import *
from math import pi
from matplotlib.table import table
import threading

## Custom libraries
from robot_utils import move_robot
from scene_assets import load_scene_assets, move_asset
from robot_setup import setup_robots
from Robots.AR4_MK5.AR4_MK5 import AR4_MK5
from Robots.RS003N.RS003Nv4 import RS003N
from Robots.RS003N.fastening import RS003NScrewing
from Robots.RS003N.GUI import RobotGui
from Robots.CRX_10iAL_Paint.CRX_10iAL_Paint import CRXPaint, CRXRMRC
from ar4_util import pick_and_place
#from Estop import Estop

# ignore warning from spatialgeometry regarding _propogate_scene_tree deprecation
import warnings
warnings.filterwarnings(
	"ignore", 
	message=r".*_propogate_scene_tree is deprecated.*", 
	category=FutureWarning, 
	module=r"spatialgeometry\..*"
)

# Main function entry point
def main():
	# Initial configuration
	## Swift environment initialization
	env = swift.Swift()
	env.launch(realtime=True)

	PORT = 'COM3'		# Serial port for physical e-stop (unused in this version)
	state_system = 0	# Initial state of the system
	running = True		# Main loop control flag

	# Main loop for state machine
	while running:
		match state_system:
			case 0:
				print("\n=============================")
				print(">> State :: 0 || Spawn Assets")
				print("=============================\n")

				# physical e-stop setup (unused in this version)
				#estop_thread = threading.Thread(
				#	target=lambda: Estop(Port=PORT, env=env,gui=gui),
				#	daemon=True
				#)
				#estop_thread.start()

				# initialise and spawn robots
				robots = setup_robots(env)

				# retrieve individual robot instances from the dictionary
				robot_AR4 = robots["AR4_MK5"]
				RS003 = robots["RS003N"]
				CRX = robots["CRX"]
				ur3e = robots["UR3e"]

				# initial robot configurations
				T_AR4Zero = robot_AR4.fkine(robot_AR4.q)
				T_RS003Zero = RS003.fkine(RS003.q)
				T_UR3eZero = ur3e.fkine(ur3e.q)
				T_CRXZero = CRX.fkine(CRX.q)

				# initialise gui
				gui = RobotGui(
					env, 
					{"AR4_MK5": robot_AR4, 
					 "RS003N": RS003, 
					 "UR3e": ur3e, 
					 "CRX_10iAL_Paint": CRX}
				)
				gui.setState(0, "Spawn Assets")
				
				# load scene assets into the environment
				meshes = load_scene_assets(env)

				shotglass = meshes["shotglass"]
				print(">> Shot glass added to the environment")

				gui.waitForNext("Press 'Next state' in Swift to proceed to state <1>")
				
				state_system = 1
			case 1:
				print("\n=============================")
				print(">> State :: 1 || Pick & Place")
				print("=============================\n")

				gui.setState(1, "Pick & Place")		# gui configuration for state 1

				# Camera setup - focus on AR4-MK5
				camera_target = robot_AR4.base.t + np.array([0, 0, 0.35])
				camera_position = robot_AR4.base.t + np.array([1.2, 1.0, 1.2])
				env.set_camera_pose(camera_position, camera_target)
				
				placement_success = pick_and_place(
					env=env, 
					robot=robot_AR4, 
					meshes=meshes, 
					gui=gui
				)

				if placement_success:
					print(">> Pick and place operation successful")
				else:
					print(">> Pick and place operation failed")

				# Move the assembled car beside the fastening robot.
				car_parts = ["chassis", "wheel_FL"]
				shift = RS003.base.t - robot_AR4.base.t

				
				car_start_poses = {name: SE3(meshes[name].T) for name in car_parts}
				# chassis centre starts at y = 7.505; pause it at y = 6.35 while the worker turns
				stop_offset = 6.35 - 7.505
				worker_pose = SE3(meshes["worker"].T)
				turn_angles = np.concatenate((np.linspace(0, pi/2, 25), np.full(25, pi/2), np.linspace(pi/2, 0, 25)))
				for seg in range(2):
					if seg == 1:
						for ang in turn_angles:
							meshes["worker"].T = (worker_pose * SE3.Rz(ang)).A
							env.step(0.02)
					y_from, y_to = [(0, stop_offset), (stop_offset, shift[1])][seg]
					for y_offset in jtraj(y_from, y_to, 50).q[:, 0]:
						T_shift = SE3.Trans(0, y_offset, 0)
						for name in car_parts:
							meshes[name].T = (T_shift * car_start_poses[name]).A
						env.step(0.02)
				print(f">> Car moved next to RS003N (shift = {np.round(shift, 3)})")

				if move_robot(robot_AR4, T_AR4Zero, env, gui=gui):
					print(">> AR4 returned to its zero-joint pose")

				camera_target = RS003.base.t + np.array([0, 0, 0.35])
				camera_position = RS003.base.t + np.array([1.2, 1.0, 1.2])
				env.set_camera_pose(camera_position, camera_target)
				
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

				# Move the chassis, wheel and shotglass together, in world (x, y, z) metres
				move_x = 0.0
				move_y = -1.8
				move_z = 0.0

				for item in [meshes["chassis"], meshes["wheel_FL"], shotglass]:
					T = item.T.copy()            # the item's current 4x4 pose
					T[0, 3] += move_x            # position column: x
					T[1, 3] += move_y            # y
					T[2, 3] += move_z            # z
					item.T = T                   # put it back
				env.step(0.05)
				print(f">> Car moved by (x, y, z) = ({move_x}, {move_y}, {move_z})")
				# ===   END CODE STATE 2   ===

				gui.waitForNext("Press 'Next state' in Swift to proceed to state <3>")
				state_system = 3
			case 3:
				print("\n=============================")
				print(">> State :: 3 || Spray Painting")
				print("=============================\n")
				gui.setState(3, "Spray Painting")
				
				

				# v   v   v   IMPLEMENT CODE HERE   v   v   v
				#CRX.q = (pi,pi/8,-pi/8,0,0,0)
				#CRX.base = SE3.Trans(3.75,3.5,0.)
				Tpose1 = SE3.Trans(3.1,3.2,1.2) * SE3.RPY(0,-pi/2,0, order='xyz')

				move_robot(CRX,target_pose=Tpose1,env=env,gui=gui)

				G1 = (3.2,4.0,1.15,0,-pi/2,0)
				CRXRMRC(G1,CRX,env,steps=100)

				G2 = (3.2,4.0,0.5,0,-pi/2,0)
				CRXRMRC(G2,CRX,env,steps=100)

				G3 = (3.2,3.0,0.5,0,-pi/2,0)
				CRXRMRC(G3,CRX,env,steps=100)

				G4 = (3.2,3.0,0.7,0,-pi/2,0)
				CRXRMRC(G4,CRX,env,steps=100)
				
				Tpose2 = SE3.Trans(3.65,3.0,0.7) * SE3.RPY(0,0,0, order='xyz')
				move_robot(CRX,target_pose=Tpose2,env=env)

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
				state_system = -1
			case _:
				print(f">> Error := Unknown system state <{state_system}>, ensure state_system is set correctly")
				running = False

if __name__ == "__main__":
	main()