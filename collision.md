# Collision Avoidance
Before I begin pretty much my code goes through the following movements appraoch, contact, lift, align and then place. I have placed the collision avoidance between the lift and align waypoints.

## How it works
Pretty much I utilised a bounding box around the wheel mesh to avoid the chassis mesh because it (the wheel) slightly clips the chassis without using any collision avoidance.

1. Save robots q value configuration
2. Solve the inverse kinematics from start location to finish
3. load collision meshes (NOTE: using `Trimesh` library for the chassis to generate triangles around the mesh)
4. Path planner
    1. Check if start / goal config is valid
    2. Attempt direct trajectory first (used as a baseline / test to see if collision avoidance is actually needed)
    3. If direct trajectory collides -> build RRT
    4. Generate random joint configurations
    5. Find the nearest existing node
    6. Extends tree to random configuration
    7. Check if proposed movmenet for any wheel-to-chassis intersection(s)
    8. Add new node if movement passes collision check
    9. Attempts to connect a new node to goal
    10. Reconstructs and interpolates the path when a valid connection is found.

    TLDR; random configurations are generated think of these as nodes. A start node is made and 'branches' out to random configurations to see if they pass the collision checks until it find the best path.

It takes about 200-300 iterations to find a good path (max is set to 500, any more is unecessary I tested this)

## Function Overview
Most of the following functions are ripped directly from Lab 5, slightly altered to fit the needs of this.

| Function | Purpose |
|---|---|
| `get_attached_mesh_pose()` | Calculates the wheel's position based on the robot's joint configuration and grasp transformation. |
| `load_collision_meshes()` | Loads the wheel and chassis STL files for collision checking. |
| `is_intersection_point_inside_triangle()` | Checks whether an intersection point lies inside a chassis triangle. |
| `get_wheel_segments()` | Approximates the wheel using the 12 edges of a bounding box. |
| `mesh_in_collision()` | Checks whether the wheel's bounding-box edges intersect the chassis at a given robot configuration. |
| `fine_interpolation()` | Generates intermediate joint configurations between two positions using `jtraj()`. |
| `interpolate_waypoints_radians()` | Combines multiple RRT waypoints into a smooth joint-space trajectory. |
| `trajectory_in_collision()` | Checks each sampled configuration in a trajectory for collisions. |
| `plan_wheel_path()` | Runs the direct-path check and RRT algorithm to find a valid route. |
| `execute_wheel_trajectory()` | Executes the planned trajectory in Swift while updating the attached wheel's position. |
