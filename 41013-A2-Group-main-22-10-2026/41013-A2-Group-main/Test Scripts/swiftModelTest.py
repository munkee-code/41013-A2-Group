
from pathlib import Path

import swift
from spatialgeometry import Mesh
from spatialmath import SE3

MODEL_PATH = (
    Path(__file__).resolve().parent
    / "Models"
    / "Blender"
    / "car_chassis_test.dae"
)

if not MODEL_PATH.is_file():
    raise FileNotFoundError(MODEL_PATH)

env = swift.Swift()
env.launch(realtime=True)

try:
    car = Mesh(
        filename=str(MODEL_PATH),
        pose=SE3.Rx(90, unit="deg"),
        scale=[1, 1, 1],
        color=[0.2, 0.3, 0.8, 1.0]
    )

    env.add(car)
    print("Car added successfully.")

    # Keep Swift open without stepping
    env.hold()

except KeyboardInterrupt:
    print("Simulation stopped.")

finally:
    env.close()
