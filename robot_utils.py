from roboticstoolbox import jtraj
from spatialmath import SE3

"""
Move a robot to a target pose using inverse kinematics and 
joint-space interpolation.

Returns True if IK succeeds, otherwise False.
"""
def move_robot(robot, target_pose, env, steps=100, increments=0.05, attached_mesh=None, grasp_transform=None):
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

        # update the attached object's position
        if attached_mesh is not None and grasp_transform is not None:
            T_ee = robot.fkine(robot.q)
            attached_mesh.T = (T_ee * grasp_transform).A

        env.step(increments)

    return True