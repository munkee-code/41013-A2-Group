
from pathlib import Path
import numpy as np
import swift

from spatialgeometry import Mesh
from spatialmath import SE3
from roboticstoolbox import jtraj

from AR4_MK5.AR4_MK5 import AR4_MK5


# ==========================================
# 1. SWIFT COMPATIBILITY
# ==========================================

# Temporary workaround for Swift 1.1.0
# and SpatialGeometry 1.4.1.
# Visualisation only: collision updates
# through this method are bypassed.

if not hasattr(Mesh, "_update_pyb"):
    Mesh._update_pyb = lambda self: None


# ==========================================
# 2. INITIALISE ROBOT
# ==========================================

robot = AR4_MK5()
robot.q = np.deg2rad([0, 0, -80, 0, 0, 0])
robot.base = SE3()

print(">> AR4-MK5 robot initialised")

# ==========================================
# CHECK DH JOINT AXES
# ==========================================

T = robot.base

for i in range(6):

    print(f"\nJoint {i+1}")

    # Joint axis position
    print("Axis position:", T.t)

    # Joint axis direction
    print("Axis direction:", T.R[:, 2])

    # Move to the next DH frame
    T = T * robot.links[i].A(robot.q[i])


# ==========================================
# 3. INITIALISE SWIFT
# ==========================================

env = swift.Swift()
env.launch(realtime=True)

# Add robot kinematic model
env.add(robot)

print(">> Robot added to Swift")


# ==========================================
# 4. MESH FILE PATHS
# ==========================================

MESH_DIR = (
    Path(__file__).resolve().parent
    / "meshes"
    / "AR4_MK5"
)

mesh_files = [
    "AR4_MK5_Base.stl",
    "AR4_MK5_L1.stl",
    "AR4_MK5_L2.stl",
    "AR4_MK5_L3.stl",
    "AR4_MK5_L4.stl",
    "AR4_MK5_L5.stl",
    "AR4_MK5_L6.stl"
]


# ==========================================
# 5. LOAD MESHES
# ==========================================

meshes = []

for filename in mesh_files:

    filepath = MESH_DIR / filename

    if not filepath.exists():
        raise FileNotFoundError(
            f"Mesh file not found: {filepath}"
        )

    mesh = Mesh(
        filename=str(filepath),
        pose=SE3(),
        color=[0.6, 0.6, 0.6, 1],
        collision=False
    )

    meshes.append(mesh)
    env.add(mesh)

    print(f">> Loaded {filename}")


# Separate stationary base and moving links
base_mesh = meshes[0]
link_meshes = meshes[1:]

import trimesh

print("\n>> STL GEOMETRY INFORMATION")

for filename in mesh_files:

    filepath = MESH_DIR / filename

    geometry = trimesh.load(
        str(filepath),
        force="mesh"
    )

    print(f"\n{filename}")
    print("Bounds:", geometry.bounds)
    print("Dimensions:", geometry.extents)
    print("Bounding-box centre:", geometry.bounds.mean(axis=0))


# ==========================================
# 6. MESH ALIGNMENT OFFSETS
# ==========================================

# These are placeholders, not calibrated
# mesh-to-DH transformations.
#
# Replace them after determining how your
# Blender STL meshes were exported.

base_offset = SE3()

#mesh_offsets = [
#    SE3(0, 0, 0.088),
#    SE3(0.06, 0, 0.18),
#    SE3(0.07, 0, 0.47),
#    SE3(0.068, 0, 0.71),
#    SE3(0.069, 0, 0.833),
#    SE3(0.07, 0, 0.86)
#]
mesh_offsets = [
    SE3(),  # Link 1
    SE3(),  # Link 2
    SE3(),  # Link 3
    SE3(),  # Link 4
    SE3(),  # Link 5
    SE3()   # Link 6
]


# ==========================================

# ==========================================
# 7. UPDATE MESH POSITIONS
# ==========================================

# Save the DH home transformations
q_home = np.zeros(6)

home_transforms = []
T = robot.base

for i in range(6):
    T = T * robot.links[i].A(q_home[i])
    home_transforms.append(T)


def update_meshes():

    base_mesh.T = robot.base.A

    T = robot.base

    for i in range(6):

        T = T * robot.links[i].A(robot.q[i])

        if i >= 3:
            # TEMPORARY TEST:
            # Keep wrist meshes in their exported
            # positions except for Joint 4 rotation.
            pivot = SE3(0.065, 0, 0.475)

            rotation = (
                pivot
                * SE3.Rz(robot.q[3])
                * pivot.inv()
            )

            link_meshes[i].T = rotation.A

        else:
            # Keep existing DH animation for J1-J3
            link_meshes[i].T = (
                T * home_transforms[i].inv()
            ).A


# ==========================================
# 8. INITIALISE MESH POSITIONS
# ==========================================

update_meshes()

env.step(0.05)

print(">> Mesh positions initialised")


# ==========================================
# 9. TEST ROBOT MOVEMENT
# ==========================================

def move_robot(q_target, steps=100):

    trajectory = jtraj(
        robot.q.copy(),
        np.asarray(q_target),
        steps
    ).q

    for q in trajectory:

        robot.q = q

        # Move meshes with the DH robot
        update_meshes()

        # Refresh Swift visualisation
        env.step(0.05)



# ==========================================
# 10. TEST JOINT 4
# ==========================================
print(">> Testing Joint 4")

q_target = robot.q.copy()
q_target[3] = np.deg2rad(20)

move_robot(q_target, steps=100)

print(">> Returning home")
move_robot(np.zeros(6), steps=100)


# ==========================================
# 11. RETURN TO HOME
# ==========================================

print(">> Returning to home")

move_robot(q_home, steps=100)


# ==========================================
# 12. KEEP SIMULATION OPEN
# ==========================================

print(">> Simulation complete")

input(">> Press Enter to exit <<")
