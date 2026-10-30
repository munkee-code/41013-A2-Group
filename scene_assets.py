from pathlib import Path
from math import pi
from spatialgeometry import Mesh
from spatialmath import SE3
from ir_support_extra_parts import *
import numpy as np
from roboticstoolbox import jtraj

def load_scene_assets(env):
    """
    Load the scene assets into the Swift environment
    Returns:
        dict: Dictionary containing all scene meshes.
    """

    cur_path = Path(__file__).resolve().parent

    # asset file paths
    asset_paths = {
        "warehouse": cur_path / "Models" / "Warehouse" / "Warehouse.dae",
        "floor": cur_path / "Models" / "Floor" / "Floor.dae",
        "conveyor": cur_path / "Models" / "Conveyor" / "Conveyor.stl",
        "chassis": cur_path / "Models" / "Chassis" / "Chassis.stl",
        "wheel_FL": cur_path / "Models" / "Components" / "Wheel_1.stl",
        "comp_conv": cur_path / "Models" / "Components" / "Conveyor_Components.stl",
        "work_area": cur_path / "Models" / "Work_Area" / "Workarea.stl"
    }

    for name, path in asset_paths.items():
        print(f">> Asset: '{name}' Path: {path}")

        if not path.exists():
            raise FileNotFoundError(f"Asset '{name}' not found at path: {path}")

    print("All assets verified. Loading into environment...")

    # Scene meshes
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
            pose=SE3.Trans(0, 0, 0),
            collision=False
        ),

        "chassis": Mesh(
            filename=str(asset_paths["chassis"]),
            pose=SE3.Trans(0, 0, 0),
            collision=False
        ),

        "wheel_FL": Mesh(
            filename=str(asset_paths["wheel_FL"]),
            pose=SE3.Trans(0, 0, 0),
            collision=False
        ),

        "comp_conv": Mesh(
            filename=str(asset_paths["comp_conv"]),
            pose=SE3.Trans(0, 0, 0),
            collision=False
        ),

        "work_area": Mesh(
            filename=str(asset_paths["work_area"]),
            pose=SE3.Trans(0, 0, 0),
            collision=False
        ),

        "robot_table": part_mesh(
            "RobotTable",
            pose=SE3.Trans(3.9, 6.9, 0.15 - 0.615)
        ),

        "worker_table": part_mesh(
            "RobotTable",
            pose=SE3.Trans(3.7, 5.85, 0)
        ),

        "stop_button": part_mesh(
            "emergencyStopButton",
            pose=SE3.Trans(3.7, 5.85, 0.3075)
        ),

        "glass_table": part_mesh(
            "RobotTable",
            pose=SE3.Trans(3.8, 4.6, 0.16 - 0.3075)
        ),

        "ur3e_table": part_mesh(
            "RobotTable",
            pose=SE3.Trans(1.9, 1.55, 0.5 - 0.615)
        ),

        "worker": part_mesh(
            "SafetyPerson",
            pose=SE3.Trans(3.7, 5.5, 0) * SE3.Rz(-pi / 2)
        ),

        "shotglass": part_mesh(
            "ShotGlass",
            pose=SE3.Trans(3.5, 4.9, 0.15) * SE3.Rz(pi/2) * SE3.Trans(-0.3, -0.30, 0.05) * SE3.Ry(pi)
        )
    }

    # mesh scaling
    meshes["worker"].scale = [0.5, 0.5, 0.5]
    meshes["worker_table"].scale = [0.5, 0.5, 0.5]
    meshes["stop_button"].scale = [0.25, 0.25, 0.25]
    meshes["glass_table"].scale = [0.5, 0.5, 0.5]

    print(">> Spawning assets into the environment.")

    # Add all meshes to the environment
    for name, mesh in meshes.items():
        env.add(mesh)
        print(f">> Spawned asset: '{name}'", flush=True)

    print(">> All assets spawned successfully.")

    return meshes

def move_asset(env, meshes, assets, translation=(0, 0, 0), rotation=(0, 0, 0), steps=50, increments=0.05):
    """
    Move one or more asset(s) through a smooth trajectory in the swift environment.

    Args:
        env: The simulation environment.
        meshes: Dictionary of mesh objects.
        assets: List of asset names to move.
        translation: Tuple of (x, y, z) translation.
        rotation: Tuple of (roll, pitch, yaw) rotation in radians.
        steps: Number of steps for the trajectory.
        increments: Time increment for each simulation step.

    Returns:
        None
    """

    # convert single asset to list if necessary
    if isinstance(assets, str):
        assets = [assets] 

    # store the starting poses of the assets to be moved
    start_poses = {
        name: SE3(meshes[name].T)
        for name in assets
    }

    translation = np.asarray(translation, dtype=float)
    rotation = np.asarray(rotation, dtype=float)

    start = np.zeros(6)
    target = np.concatenate((translation, rotation))

    trajectory = jtraj(start, target, steps)

    for pose in trajectory.q:
        x, y, z, roll, pitch, yaw = pose

        T_move = SE3.Trans(x, y, z) * SE3.Rz(yaw) * SE3.Ry(pitch) * SE3.Rx(roll)
        for name in assets:
            meshes[name].T = (T_move * start_poses[name]).A

        env.step(increments)

    print(f">> Asset(s) '{', '.join(assets)}' moved by:\n   Translation: {translation}\n   Rotation: {rotation}")