
import numpy as np
from pathlib import Path
from roboticstoolbox import jtraj
from spatialmath import SE3
import trimesh


# ============================================================
# ATTACHED MESH TRANSFORM
# ============================================================

def get_attached_mesh_pose(robot, q, grasp_transform):
    """
    Calculate the world pose of an object attached to
    the robot end-effector.
    """
    return robot.fkine(q) * grasp_transform


# ============================================================
# LOAD COLLISION GEOMETRY
# ============================================================

def load_collision_meshes():
    """
    Load the original wheel and chassis STL geometry.
    The meshes are not recentered or rescaled.
    """
    root = Path(__file__).resolve().parent

    chassis_path = root / "Models" / "Chassis" / "Chassis.stl"
    wheel_path = root / "Models" / "Components" / "Wheel_1.stl"

    chassis = trimesh.load_mesh(
        str(chassis_path),
        process=False
    )

    wheel = trimesh.load_mesh(
        str(wheel_path),
        process=False
    )

    return chassis, wheel


# ============================================================
# COLLISION CHECKING
# ============================================================

def _points_in_mesh(points, mesh):
    """
    Approximate containment test using ray intersections.

    Returns a boolean array indicating which points
    appear to be inside the mesh.

    This method requires a closed mesh for reliable
    containment results.
    """
    points = np.asarray(points, dtype=float)

    if len(points) == 0:
        return np.zeros(0, dtype=bool)

    # Ray direction is deliberately non-axis-aligned.
    direction = np.array([0.812, 0.371, 0.451])
    direction /= np.linalg.norm(direction)

    inside = np.zeros(len(points), dtype=bool)

    for i, point in enumerate(points):
        # Count intersections along the ray.
        # Odd intersection count indicates containment.
        locations, _, _ = mesh.ray.intersects_location(
            ray_origins=[point],
            ray_directions=[direction],
            multiple_hits=True
        )

        if len(locations) == 0:
            continue

        distances = np.linalg.norm(
            locations - point,
            axis=1
        )

        # Avoid duplicate hits on adjacent triangles.
        distances = np.unique(np.round(distances, 6))

        inside[i] = (len(distances) % 2) == 1

    return inside


def mesh_in_collision(
    robot,
    q,
    grasp_transform,
    mesh_geometry,
    chassis_geometry,
    chassis_pose,
    sample_count=200
):
    """
    Approximate collision check between the attached
    wheel and the chassis.

    Samples wheel surface points and tests whether
    they lie inside the chassis mesh.
    """

    object_pose = get_attached_mesh_pose(
        robot,
        q,
        grasp_transform
    )

    # Copy to avoid modifying original geometry.
    wheel = mesh_geometry.copy()
    chassis = chassis_geometry.copy()

    wheel.apply_transform(object_pose.A)
    chassis.apply_transform(chassis_pose.A)

    # Sample points on the wheel surface.
    points = wheel.sample(sample_count)

    # Broad-phase bounding box rejection.
    bounds = chassis.bounds

    candidate_mask = np.all(
        (points >= bounds[0]) &
        (points <= bounds[1]),
        axis=1
    )

    candidate_points = points[candidate_mask]

    if len(candidate_points) == 0:
        return False

    return bool(
        np.any(
            _points_in_mesh(candidate_points, chassis)
        )
    )


# ============================================================
# JOINT TRAJECTORY INTERPOLATION
# ============================================================

def fine_interpolation(
    q1,
    q2,
    max_step_radians=np.deg2rad(2)
):
    """
    Generate a smooth joint-space trajectory with
    a bounded maximum joint-angle change.
    """

    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)

    steps = max(
        2,
        int(
            np.ceil(
                np.max(np.abs(q2 - q1)) /
                max_step_radians
            )
        ) + 1
    )

    return jtraj(q1, q2, steps).q


def interpolate_waypoints_radians(
    waypoints,
    max_step_radians=np.deg2rad(2)
):
    """
    Interpolate a sequence of joint-space waypoints.
    """

    q_matrix = []

    for i in range(len(waypoints) - 1):
        segment = fine_interpolation(
            waypoints[i],
            waypoints[i + 1],
            max_step_radians
        )

        if i > 0:
            segment = segment[1:]

        q_matrix.extend(segment)

    return np.asarray(q_matrix)


# ============================================================
# TRAJECTORY COLLISION CHECK
# ============================================================

def trajectory_in_collision(
    robot,
    q_matrix,
    grasp_transform,
    wheel_geometry,
    chassis_geometry,
    chassis_pose
):
    """
    Check each configuration in a joint trajectory.
    """

    for q in q_matrix:
        if mesh_in_collision(
            robot,
            q,
            grasp_transform,
            wheel_geometry,
            chassis_geometry,
            chassis_pose
        ):
            return True

    return False


# ============================================================
# RRT PATH PLANNER
# ============================================================

def plan_wheel_path(
    robot,
    q_start,
    q_goal,
    grasp_transform,
    wheel_geometry,
    chassis_geometry,
    chassis_pose,
    max_iterations=2000,
    step_size=np.deg2rad(10)
):
    """
    Plan a collision-free joint-space trajectory using
    a basic Rapidly-exploring Random Tree (RRT).

    Returns:
        np.ndarray: Planned trajectory.
        None: No valid path found.
    """

    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)

    qlim = np.asarray(robot.qlim, dtype=float)

    def is_valid(q1, q2):
        q_matrix = fine_interpolation(q1, q2)

        return not trajectory_in_collision(
            robot,
            q_matrix,
            grasp_transform,
            wheel_geometry,
            chassis_geometry,
            chassis_pose
        )

    # Check start and goal.
    if not is_valid(q_start, q_start):
        print(">> Start configuration is in collision")
        return None

    if not is_valid(q_goal, q_goal):
        print(">> Goal configuration is in collision")
        return None

    # Attempt direct trajectory first.
    if is_valid(q_start, q_goal):
        print(">> Direct trajectory is collision-free")
        return fine_interpolation(q_start, q_goal)

    print(">> Direct trajectory collides; starting RRT")

    nodes = [
        {
            "q": q_start,
            "parent": None
        }
    ]

    rng = np.random.default_rng()

    for iteration in range(max_iterations):

        # Goal-biased random sampling.
        if rng.random() < 0.1:
            q_rand = q_goal
        else:
            q_rand = rng.uniform(
                qlim[0],
                qlim[1]
            )

        # Find nearest node.
        distances = [
            np.linalg.norm(node["q"] - q_rand)
            for node in nodes
        ]

        nearest_index = int(np.argmin(distances))
        q_nearest = nodes[nearest_index]["q"]

        direction = q_rand - q_nearest
        distance = np.linalg.norm(direction)

        if distance < 1e-9:
            continue

        # Extend tree towards sample.
        q_new = q_nearest + (
            direction / distance
        ) * min(step_size, distance)

        # Reject colliding connections.
        if not is_valid(q_nearest, q_new):
            continue

        nodes.append({
            "q": q_new,
            "parent": nearest_index
        })

        new_index = len(nodes) - 1

        # Try connecting to the goal.
        if is_valid(q_new, q_goal):

            waypoints = [q_goal]
            index = new_index

            while index is not None:
                waypoints.append(nodes[index]["q"])
                index = nodes[index]["parent"]

            waypoints.reverse()

            q_matrix = interpolate_waypoints_radians(
                waypoints
            )

            # Final validation.
            if trajectory_in_collision(
                robot,
                q_matrix,
                grasp_transform,
                wheel_geometry,
                chassis_geometry,
                chassis_pose
            ):
                continue

            print(
                f">> RRT path found after "
                f"{iteration + 1} iterations"
            )

            return q_matrix

    print(">> RRT failed to find a path")
    return None


# ============================================================
# EXECUTE PLANNED TRAJECTORY
# ============================================================

def execute_wheel_trajectory(
    env,
    robot,
    q_matrix,
    wheel_mesh,
    grasp_transform,
    increment=0.02
):
    """
    Execute a planned trajectory while updating the
    wheel mesh relative to the end-effector.
    """

    if q_matrix is None or len(q_matrix) == 0:
        print(">> No trajectory available")
        return False

    for q in q_matrix:
        robot.q = np.asarray(q, dtype=float)

        T_wheel = get_attached_mesh_pose(
            robot,
            robot.q,
            grasp_transform
        )

        wheel_mesh.T = T_wheel.A

        env.step(increment)

    print(">> Planned wheel trajectory completed")
    return True