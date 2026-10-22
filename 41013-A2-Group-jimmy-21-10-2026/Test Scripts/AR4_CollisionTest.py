
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from roboticstoolbox import jtraj
from spatialmath import SE3
from math import pi

from Robots.AR4_MK5.AR4_MK5 import AR4_MK5

#---------------------------------------------------------------------------------------------------
# ROBOT
#---------------------------------------------------------------------------------------------------

robot = AR4_MK5()

robot.q = np.zeros(6)
robot.base = SE3(0, 0, 0)

print(">> Robot: AR4-MK5 initialised")

#---------------------------------------------------------------------------------------------------
# COLLISION CUBE
#---------------------------------------------------------------------------------------------------

# Cube position and dimensions (metres)
cube_center = np.array([0.35, 0.0, 0.3])
cube_size = 0.25

# Number of samples per axis
samples = 7

# Generate cube surface points
coordinates = np.linspace(
    -cube_size / 2,
    cube_size / 2,
    samples
)

cube_points = []

for i, x in enumerate(coordinates):
    for j, y in enumerate(coordinates):
        for k, z in enumerate(coordinates):

            # Only include points on the cube surface
            if (
                i == 0 or i == samples - 1
                or j == 0 or j == samples - 1
                or k == 0 or k == samples - 1
            ):
                cube_points.append(
                    cube_center + np.array([x, y, z])
                )

cube_points = np.array(cube_points)

print(">> Cube surface points:", len(cube_points))

#---------------------------------------------------------------------------------------------------
# ELLIPSOID
#---------------------------------------------------------------------------------------------------

# Ellipsoid radii along X, Y and Z
ellipsoid_radii = np.array([0.10, 0.08, 0.12])

# Generate a unit sphere surface
u = np.linspace(0, 2 * pi, 30)
v = np.linspace(0, pi, 20)

u, v = np.meshgrid(u, v)

unit_x = np.cos(u) * np.sin(v)
unit_y = np.sin(u) * np.sin(v)
unit_z = np.cos(v)

#---------------------------------------------------------------------------------------------------
# ROBOT TRAJECTORY
#---------------------------------------------------------------------------------------------------

q_start = np.zeros(6)

# Example target configuration
q_end = np.deg2rad([45, -20, 30, 0, 20, 0])

# Generate trajectory
trajectory = jtraj(q_start, q_end, 150)

print(">> Trajectory generated")

#---------------------------------------------------------------------------------------------------
# MATPLOTLIB 3D PLOT
#---------------------------------------------------------------------------------------------------

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

ax.set_title("AR4-MK5 Collision Visualisation")

ax.set_xlabel("World X (m)")
ax.set_ylabel("World Y (m)")
ax.set_zlabel("World Z (m)")

ax.set_xlim(-0.2, 0.8)
ax.set_ylim(-0.5, 0.5)
ax.set_zlim(0, 0.8)

ax.set_box_aspect([1, 1, 0.8])

# Cube surface samples
cube_scatter = ax.scatter(
    cube_points[:, 0],
    cube_points[:, 1],
    cube_points[:, 2],
    c="blue",
    s=20,
    depthshade=False
)

# Robot skeleton
robot_line, = ax.plot(
    [],
    [],
    [],
    "-o",
    color="black",
    linewidth=3,
    markersize=5
)

# Ellipsoid surface
ellipsoid_surface = None

#---------------------------------------------------------------------------------------------------
# ANIMATION
#---------------------------------------------------------------------------------------------------

def update(frame):

    global ellipsoid_surface

    # Update robot configuration
    q = trajectory.q[frame]
    robot.q = q

    # Get robot joint transformations
    link_poses = robot.fkine_all(q).A

    joint_positions = np.array([
        T[:3, 3] for T in link_poses
    ])

    # Update robot skeleton
    robot_line.set_data(
        joint_positions[:, 0],
        joint_positions[:, 1]
    )

    robot_line.set_3d_properties(
        joint_positions[:, 2]
    )

    # Position ellipsoid at robot end effector
    T_ee = robot.fkine(q)
    ellipsoid_center = T_ee.t

    # Ellipsoid aligned with end-effector orientation
    R = T_ee.R

    unit_points = np.stack(
        [unit_x, unit_y, unit_z],
        axis=-1
    )

    scaled_points = unit_points * ellipsoid_radii

    rotated_points = scaled_points @ R.T
    world_points = rotated_points + ellipsoid_center

    ellipsoid_x = world_points[:, :, 0]
    ellipsoid_y = world_points[:, :, 1]
    ellipsoid_z = world_points[:, :, 2]

    # Remove previous ellipsoid surface
    if ellipsoid_surface is not None:
        ellipsoid_surface.remove()

    ellipsoid_surface = ax.plot_surface(
        ellipsoid_x,
        ellipsoid_y,
        ellipsoid_z,
        color="teal",
        alpha=0.2,
        linewidth=0
    )

    #-----------------------------------------------------------------------------------------------
    # COLLISION CHECKING
    #-----------------------------------------------------------------------------------------------

    # Transform cube points into ellipsoid-local coordinates
    relative_points = cube_points - ellipsoid_center

    local_points = relative_points @ R

    # Ellipsoid equation:
    # (x/a)^2 + (y/b)^2 + (z/c)^2 <= 1

    normalized_distance = np.sum(
        (local_points / ellipsoid_radii) ** 2,
        axis=1
    )

    collisions = normalized_distance <= 1.0

    # Blue = outside ellipsoid
    # Red = inside ellipsoid
    colors = np.where(
        collisions[:, None],
        np.array([1.0, 0.2, 0.1, 1.0]),
        np.array([0.1, 0.4, 1.0, 1.0])
    )

    cube_scatter.set_color(colors)

    ax.set_title(
        f"AR4-MK5 Collision Visualisation | "
        f"Frame {frame + 1}/{len(trajectory.q)} | "
        f"Collisions: {np.count_nonzero(collisions)}"
    )

    return robot_line, cube_scatter, ellipsoid_surface


animation = FuncAnimation(
    fig,
    update,
    frames=len(trajectory.q),
    interval=50,
    blit=False,
    repeat=True
)

#---------------------------------------------------------------------------------------------------
# DISPLAY
#---------------------------------------------------------------------------------------------------

plt.show()

print(">> Simulation completed")
