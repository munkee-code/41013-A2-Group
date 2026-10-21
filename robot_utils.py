from roboticstoolbox import jtraj

"""
Move a robot to a target pose using inverse kinematics and 
joint-space interpolation.

Returns True if IK succeeds, otherwise False.
"""
def move_robot(robot, target_pose, env, steps=100, increments=0.05):
    ik = robot.ikine_LM(
        target_pose,
        q0=robot.q,
        joint_limits=True
    )

    if not ik.success:
        print(f">> IK solution not found for target pose")
        print(f">> Reason: {ik.reason}")
        return False

    traj = jtraj(robot.q, ik.q, steps)

    for q in traj.q:
        robot.q = q
        env.step(increments)

    return True