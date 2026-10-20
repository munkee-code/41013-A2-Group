import swift
from pathlib import Path
from spatialgeometry import Mesh
from spatialmath import SE3
from math import pi

print(">> Starting simulation", flush=True)

env = swift.Swift()
env.launch(realtime=True)

print(">> Swift launched", flush=True)

current_path = Path(__file__).resolve().parent.parent
factory_path = (
    current_path 
    / "Models" 
    / "Warehouse" 
    #/ "Factory.stl"        # Works but not preferred (no textures)
    #/ "Factory.glb"        # Loads mesh but gets stuck
    / "Warehouse.dae"         # << MUST BE .dae
)

print(">> Factory path:", factory_path, flush=True)

factory_mesh = Mesh(
    filename=str(factory_path),
    pose=SE3.Rx(pi/2),
    collision=False
)

print(">> Factory mesh created", flush=True)

env.add(factory_mesh)

print(">> Factory added to Swift", flush=True)

input(">> Press Enter to exit simulation")