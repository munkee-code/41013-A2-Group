
from pathlib import Path
import swift

from spatialgeometry import Mesh
from spatialmath import SE3

# Temporary compatibility workaround
if not hasattr(Mesh, "_update_pyb"):
    Mesh._update_pyb = lambda self: None

# Start Swift
env = swift.Swift()
env.launch(realtime=True)

# Mesh directory
mesh_dir = (
    Path(__file__).resolve().parent
    / "meshes"
    / "AR4_MK5"
)

# Load Link 1 only
link1 = Mesh(
    filename=str(mesh_dir / "AR4_MK5_L1.stl"),
    pose=SE3.Trans(0, 0, 0.3),
    color=[1, 0, 0, 1],
    collision=False
)

env.add(link1)

print(">> Link 1 added to Swift")

env.step(0.05)

input(">> Press Enter to exit <<")
