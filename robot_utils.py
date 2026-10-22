from roboticstoolbox import jtraj
from spatialmath import SE3

def move_robot(robot, target_pose, env, steps=100, increments=0.05, attached_mesh=None, grasp_transform=None, gui=None):
    ik = robot.ikine_LM(
        target_pose,
        q0=robot.q,
        joint_limits=True
    )

    if not ik.success:
        print(f">> IK solution not found for target pose")
        print(f">> Reason: {ik.reason}")
        return False

    def update_attached(q):
        # update the attached object's position
        if attached_mesh is not None and grasp_transform is not None:
            T_ee = robot.fkine(q)
            attached_mesh.T = (T_ee * grasp_transform).A

    prev_move_cb = None
    if gui is not None:
        # while this move runs, a held mesh follows slider moves made during an e-stop
        prev_move_cb = gui._moveCallbacks.get(id(robot))
        gui.setMoveCallback(robot, update_attached)
        gui.motionRunning = True        # block slider moves while this robot is driven

    try:
        traj = jtraj(robot.q, ik.q, steps).q
        i = 0
        while i < len(traj):
            # E-STOP: hold in place until reset; re-plan if the joints were moved meanwhile
            if gui is not None and gui.estopActive:

                gui.movedByHand = False
                gui.waitWhileEstopped()

                if gui.movedByHand:
                    print(">> Joints were moved during the e-stop - re-planning this move")
                    traj = jtraj(robot.q, ik.q, max(len(traj) - i, 2)).q
                    i = 0

            q = traj[i]
            robot.q = q
            update_attached(q)

            # keep the GUI sliders showing the current joint angles
            if gui is not None:
                gui.syncSliders()

            env.step(increments)
            i += 1
    finally:
        if gui is not None:
            gui.motionRunning = False
            gui.setMoveCallback(robot, prev_move_cb)

    return True