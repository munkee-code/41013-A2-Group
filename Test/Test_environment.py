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
warehouse_path = (current_path / "Models" / "Warehouse" / "Warehouse.dae")
floor_path = (current_path / "Models" / "Floor" / "Floor.dae")   
conveyor_path = (current_path / "Models" / "Conveyor" / "Conveyor.stl")
chassis_path = (current_path / "Models" / "Chassis" / "Chassis.stl")

print(">> Chassis path:", chassis_path, flush=True)
env.add(Mesh(
    filename=str(chassis_path),
    pose=SE3.Trans(0, 0, 0),
    collision=False
))
print(">> Chassis mesh created", flush=True)
print(">> Chassis added to Swift", flush=True)

print(">> Warehouse path:", warehouse_path, flush=True)
warehouse_mesh = Mesh(
    filename=str(warehouse_path),
    pose=SE3.Rx(pi/2),
    collision=False
)
print(">> Warehouse mesh created", flush=True)

print(">> Floor path:", floor_path, flush=True)
env.add(warehouse_mesh)
floor_mesh = Mesh(
    filename=str(floor_path),
    pose=SE3.Trans(2.7, 4.1, 0.01),
    scale=[0.04, 0.04, 1],
    collision=False
)
env.add(floor_mesh)
print(">> Floor mesh created", flush=True)

print(">> Conveyor path:", conveyor_path, flush=True)
env.add(Mesh(
    filename=str(conveyor_path),
    pose=SE3.Trans(0, 0, 0),
    collision=False
))
print(">> Conveyor mesh created", flush=True)

print(">> Chassis path:", chassis_path, flush=True)
env.add(Mesh(
    filename=str(chassis_path),
    pose=SE3.Trans(0, 0, 0),
    collision=False
))
print(">> Chassis mesh created", flush=True)

print(">> Chassis added to Swift", flush=True)
print(">> Warehouse added to Swift", flush=True)
print(">> Floor added to Swift", flush=True)
print(">> Conveyor added to Swift", flush=True)

input(">> Press Enter to exit simulation")