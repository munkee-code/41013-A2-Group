import numpy as np
import swift
from RS003Nv4 import RS003N

# ----------------------------------------------------------------------
# Environment and robot
# ----------------------------------------------------------------------
env = swift.Swift()
env.launch(realtime=True)

robot = RS003N()
robot.q = robot._qtest
robot.add_to_env(env)


# ----------------------------------------------------------------------
# Joint sliders
# ----------------------------------------------------------------------
def set_joint(j, value):
    q = robot.q.copy()
    q[j] = np.deg2rad(float(value))
    robot.q = q


for i, link in enumerate(robot.links):
    env.add(
        swift.Slider(
            lambda value, j=i: set_joint(j, value),
            min=float(np.rad2deg(link.qlim[0])),
            max=float(np.rad2deg(link.qlim[1])),
            step=1,
            value=float(np.rad2deg(robot.q[i])),
            desc=f"Joint {i + 1}",
            unit="&#176;",
        )
    )

# ----------------------------------------------------------------------
# Keep the scene updating so slider changes appear
# ----------------------------------------------------------------------
while True:
    env.step(0.02)