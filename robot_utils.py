from roboticstoolbox import jtraj
from spatialmath import SE3

def move_robot(robot, target_pose, env, steps=100, increments=0.05, attached_mesh=None, grasp_transform=None, gui=None):
    """
    Move the robot to the target pose using inverse kinematics and trajectory planning.

    Args:
        robot: The robot instance.
        target_pose: The desired end-effector pose (SE3).
        env: The simulation environment.
        steps: Number of steps for the trajectory.
        increments: Time increment for each simulation step.
        attached_mesh: Optional mesh attached to the end-effector.
        grasp_transform: Transform from end-effector to attached mesh.
        gui: Optional GUI instance.

    Returns:
        bool: True if the move succeeds, False otherwise.
    """

    # compute inverse kinematics for the target pose
    ik = robot.ikine_LM(
        target_pose,
        q0=robot.q,
        joint_limits=True
    )

    # handler for inverse kinematics failure
    if not ik.success:
        print(f">> IK solution not found for target pose")
        print(f">> Reason: {ik.reason}")
        return False

    def update_attached(q):
        """
        Update the attached mesh's position based on the current end-effector pose.

        Args:
            q: Current joint configuration of the robot.
        """
        # update the attached object's position
        if attached_mesh is not None and grasp_transform is not None:
            T_ee = robot.fkine(q)
            attached_mesh.T = (T_ee * grasp_transform).A

    # store previous GUI move callback to restore later
    prev_move_cb = None
    if gui is not None:
        # while this move runs, a held mesh follows slider moves made during an e-stop
        prev_move_cb = gui._moveCallbacks.get(id(robot))
        gui.setMoveCallback(robot, update_attached)
        gui.motionRunning = True        # block slider moves while this robot is driven

    # plan and execute the trajectory to the target pose
    # NOTE: using try-finally to ensure GUI state is restored even if an error occurs
    try:
        # compute the joint trajectory
        traj = jtraj(robot.q, ik.q, steps).q

        # execute trajectory
        i = 0
        while i < len(traj):
            # E-stop handler
            if gui is not None and gui.estopActive:
                gui.movedByHand = False
                gui.waitWhileEstopped()

                if gui.movedByHand:
                    print(">> Joints were moved during the e-stop - re-planning this move")
                    traj = jtraj(robot.q, ik.q, max(len(traj) - i, 2)).q
                    i = 0

            q = traj[i]                 # get current joint configuration
            robot.q = q                 # set robot to current joint configuration
            update_attached(q)          # update attached mesh position

            # keep the GUI sliders showing the current joint angles
            if gui is not None:
                gui.syncSliders()

            # environment step (animation)
            env.step(increments)
            i += 1
    finally:
        if gui is not None:
            gui.motionRunning = False
            gui.setMoveCallback(robot, prev_move_cb)

    return True