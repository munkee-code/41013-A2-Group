import numpy as np
from pathlib import Path
from roboticstoolbox import jtraj
from spatialmath import SE3
import trimesh
from ir_support import line_plane_intersection

def get_attached_mesh_pose(robot, q, grasp_transform):
    """
    Calculate the world pose of an object attached to the robot end-effector.

    NOTE: lab 5 utilises this for the link poses rather than attached objects.

    Args:
        robot: The robot instance.
        q: Current joint configuration.
        grasp_transform: Transform from end-effector to the attached object.

    Returns:
        SE3: World pose of the attached object.
    """
    return robot.fkine(q) * grasp_transform

def load_collision_meshes():
    """
    Load the original wheel and chassis STL geometry. 
    
    NOTE: The meshes are not recentered or rescaled; used for collision checking only. Lab 5 is models
    a simple prism; using trimesh to load the vertices and faces (triangles).
    """
    root = Path(__file__).resolve().parent

    chassis_path = root / "Models" / "Chassis" / "Chassis.stl"
    wheel_path = root / "Models" / "Components" / "Wheel_1.stl"

    # NOTE: process=False ensures preservation of original mesh geometry
    chassis = trimesh.load_mesh(
        str(chassis_path),
        process=False
    )

    wheel = trimesh.load_mesh(
        str(wheel_path),
        process=False
    )

    return chassis, wheel

def is_intersection_point_inside_triangle(intersect_p, triangle_verts):
    """
    Determine whether a point lies inside a triangle. Ripped straight from Lab 5 solution.

    Args:
        intersect_p: The point to check.
        triangle_verts: 3x3 array of triangle vertices.

    Returns:
        bool: True if the point is inside the triangle, False otherwise.
    """

    u = triangle_verts[1, :] - triangle_verts[0, :]
    v = triangle_verts[2, :] - triangle_verts[0, :]

    uu = np.dot(u, u)
    uv = np.dot(u, v)
    vv = np.dot(v, v)

    w = intersect_p - triangle_verts[0, :]

    wu = np.dot(w, u)
    wv = np.dot(w, v)

    D = uv * uv - uu * vv

    # check for degenerate triangle (area close to zero)
    if abs(D) < 1e-12:
        return False

    s = (uv * wv - vv * wu) / D
    t = (uv * wu - uu * wv) / D

    return s >= 0.0 and t >= 0.0 and s + t <= 1.0

def get_wheel_segments(wheel_geometry, sample_count=60):
    """
    Represent the wheel mesh as a cuboid.

    NOTE: using the cuboids edges rather than faces for collision approximation.

    Args:
        wheel_geometry: The trimesh geometry of the wheel.
        sample_count: Number of samples along each edge of the cuboid.

    Returns:
        np.ndarray: Array of line segments representing the cuboid edges.
    """
    # extract the bounds of the wheel mesh
    bounds = wheel_geometry.bounds

    # get the minimum and maximum coordinates of the bounding box
    minimum = bounds[0]
    maximum = bounds[1]

    # extract coordinates of min / max points
    x0, y0, z0 = minimum
    x1, y1, z1 = maximum

    # create vertices of the cuboid based on min / max coordinates
    vertices = np.array([
        [x0, y0, z0],
        [x1, y0, z0],
        [x1, y1, z0],
        [x0, y1, z0],
        [x0, y0, z1],
        [x1, y0, z1],
        [x1, y1, z1],
        [x0, y1, z1]
    ])

    # define the edges of the cuboid using vertex indices
    edges = np.array([
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ])

    return vertices[edges]

def mesh_in_collision(robot, q, grasp_transform, mesh_geometry, chassis_geometry, chassis_pose, sample_count=15):
    """
    Check whether wheel line segments intersect with chassis triangles.

    Args:
        robot: The robot model.
        q: The joint configuration.
        grasp_transform: The transform of the grasped wheel.
        mesh_geometry: The geometry of the wheel mesh.
        chassis_geometry: The geometry of the chassis mesh.
        chassis_pose: The pose of the chassis.
        sample_count: Number of samples along each edge of the cuboid.

    Returns:
        bool: True if a collision is detected, False otherwise.
    """

    # calculate attached wheel pose
    T_wheel = get_attached_mesh_pose(robot, q, grasp_transform)

    # copy meshes to avoid modifying originals
    wheel = mesh_geometry.copy()
    chassis = chassis_geometry.copy()

    # apply transforms of the wheel and chassis to their respective meshes
    wheel.apply_transform(T_wheel.A)
    chassis.apply_transform(chassis_pose.A)

    # approximate wheel using line segments
    segments = get_wheel_segments(wheel, sample_count)
    chassis_triangles = np.asarray(chassis.triangles, dtype=float)

    # precompute chassis triangle normals
    edge1 = chassis_triangles[:, 1] - chassis_triangles[:, 0]
    edge2 = chassis_triangles[:, 2] - chassis_triangles[:, 0]
    normals = np.cross(edge1, edge2)

    # check wheel segments against chassis triangles
    for segment in segments:
        # start / end points of the wheel segment
        start = segment[0]
        end = segment[1]

        # for each chassis triangle...
        for j, triangle in enumerate(chassis_triangles):
            # compute intersection of the wheel segment with the plane of the triangle
            intersection, check = line_plane_intersection(normals[j], triangle[0], start, end)

            # if the segment does not intersect the plane, skip this triangle
            if check != 1:
                continue

            # check if the intersection point lies inside the triangle
            if is_intersection_point_inside_triangle(intersection, triangle):
                return True
            
    return False

def fine_interpolation(q1, q2, max_step_radians=np.deg2rad(2)):
    """
    Generate a smooth joint-space trajectory.
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

def interpolate_waypoints_radians(waypoints, max_step_radians=np.deg2rad(2)):
    """
    Interpolate a sequence of joint-space waypoints.

    NOTE: ripped straight from lab 5.

    Args:
        waypoints: A list of joint-space waypoints.
        max_step_radians: Maximum allowed change in joint angles between consecutive steps.

    Returns:
        np.ndarray: Interpolated joint configurations forming a smooth trajectory.
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

def trajectory_in_collision(robot, q_matrix, grasp_transform, wheel_geometry, chassis_geometry, chassis_pose):
    """
    Check each configuration in a joint trajectory.

    NOTE: adapted is_collision() from lab 5. Lab 5 iterates through each joint config and checks if
    the robot itself intersects the obstacle, whereas here i just check if the mesh of the grasped 
    object collides with the environment. Literally, just simpler version of is_collision().

    Args:
        robot: The robot model.
        q_matrix: A sequence of joint configurations.
        grasp_transform: The transform of the grasped object.
        wheel_geometry: Geometry of the wheels for collision checking.
        chassis_geometry: Geometry of the chassis for collision checking.
        chassis_pose: Pose of the chassis in the world frame.

    Returns:
        bool: True if any configuration in the trajectory is in collision, False otherwise.
    """

    for q in q_matrix:
        if mesh_in_collision(robot, q, grasp_transform, wheel_geometry, chassis_geometry, chassis_pose):
            return True

    return False

def plan_wheel_path(robot, q_start, q_goal, grasp_transform, wheel_geometry, chassis_geometry, chassis_pose, max_iterations=500, step_size=np.deg2rad(10)):
    """
    Plan a collision-free joint-space trajectory using a basic Rapidly-exploring Random Tree (RRT).
    
    Args:
        robot: The robot model.
        q_start: Start joint configuration.
        q_goal: Goal joint configuration.
        grasp_transform: The transform of the grasped object.
        wheel_geometry: Geometry of the wheels for collision checking.
        chassis_geometry: Geometry of the chassis for collision checking.
        chassis_pose: Pose of the chassis in the world frame.
        max_iterations: Maximum number of RRT iterations.
        step_size: Maximum allowed change in joint angles between consecutive steps.

    Returns:
        np.ndarray: Planned trajectory.
        None: No valid path found.
    """

    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)

    # IMPORTANT: ensure joint limits are in float for consistent calculations
    qlim = np.asarray(robot.qlim, dtype=float)

    # helper function to check if the path between two configurations is collision-free
    def is_valid(q1, q2):
        q_matrix = fine_interpolation(q1, q2)

        return not trajectory_in_collision(robot, q_matrix, grasp_transform, wheel_geometry, chassis_geometry, chassis_pose)

    # check start and goal.
    if not is_valid(q_start, q_start):
        print(">> Start configuration is in collision")
        return None

    if not is_valid(q_goal, q_goal):
        print(">> Goal configuration is in collision")
        return None

    # attempt direct trajectory first.
    if is_valid(q_start, q_goal):
        print(">> Direct trajectory is collision-free")
        return fine_interpolation(q_start, q_goal)

    print(">> Direct trajectory collides; starting RRT")

    # using nodes to build the RRT tree
    nodes = [{"q": q_start, "parent": None}]

    rng = np.random.default_rng()               # random number generator for sampling in RRT

    # for each RRT iteration...
    for iteration in range(max_iterations):
        # progress messages
        if iteration % 25 == 0:
            print(
                f">> RRT iteration {iteration}/{max_iterations} "
                f"| Tree nodes: {len(nodes)}",
                flush=True
            )

        # goal-biased random sampling
        # if rng is less than 0.1...
        if rng.random() < 0.1:
            q_rand = q_goal     # sample the goal diractly 
        else:
            # sample randomly
            q_rand = rng.uniform(qlim[0], qlim[1])

        # find nearest node
        distances = [np.linalg.norm(node["q"] - q_rand) for node in nodes]
        nearest_index = int(np.argmin(distances))
        q_nearest = nodes[nearest_index]["q"]
        direction = q_rand - q_nearest
        distance = np.linalg.norm(direction)

        # check if distance is too small
        if distance < 1e-9:
            continue

        # extend tree towards sample
        q_new = q_nearest + (direction / distance) * min(step_size, distance)

        # reject colliding connections.
        if not is_valid(q_nearest, q_new):
            continue

        nodes.append({"q": q_new, "parent": nearest_index})
        new_index = len(nodes) - 1

        # try connecting to the goal.
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

            # final validation...
            if trajectory_in_collision(robot, q_matrix, grasp_transform, wheel_geometry, chassis_geometry, chassis_pose):
                continue

            print(
                f">> RRT path found after "
                f"{iteration + 1} iterations"
            )

            return q_matrix

    print(">> RRT failed to find a path")
    return None

def execute_wheel_trajectory(env, robot, q_matrix, wheel_mesh, grasp_transform, increment=0.02):
    """
    Execute a planned trajectory while updating the wheel mesh relative to the end-effector.

    Args:
        env: The simulation environment.
        robot: The robot object.
        q_matrix: The planned trajectory as a list of joint configurations.
        wheel_mesh: The mesh representing the wheel.
        grasp_transform: The transformation from the end-effector to the wheel.
        increment: The simulation time step increment.

    Returns:
        True if the trajectory was executed successfully, False otherwise.
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