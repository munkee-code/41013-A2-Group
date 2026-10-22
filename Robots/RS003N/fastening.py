"""
fastening.py - RS003N pick -> carry -> slide in -> screw routine (State 2).
Location: Robots/RS003N/fastening.py

Motion and task logic only. The GUI (buttons, sliders, e-stop) lives in
GUI.py and the parts are spawned in main.py.

All positions/orientations here are in the RS003N's BASE frame and are
converted to world coordinates with robot.base.

Usage (in main.py):
    from Robots.RS003N.GUI import RobotGui
    from Robots.RS003N.fastening import RS003NScrewing

    gui = RobotGui(env, {"RS003N": RS003, "AR4_MK5": robot_AR4})   # state 0
    fastening = RS003NScrewing(env, RS003, gui, [shotglass])       # state 2
    fastening.run()
"""
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# ----------------------------------------------------------------------
# Settings - tune these
# ----------------------------------------------------------------------
APPROACH_HEIGHT = 0.10      # m above a part before descending onto it
GRASP_OFFSET = 0.0          # m above the part origin when gripping
LIFT_HEIGHT = 0.10          # part height above the robot base after pick-up (m)
SLIDE_DISTANCE = 0.10       # m backed off along the tool axis before sliding in

J6_CARRY = -2 * np.pi       # joint 6 angle while picking and carrying (rad)
J6_SCREW = 2 * np.pi        # joint 6 angle at the end of the "screw" (rad)
SCREW_STEPS = 200           # animation steps for the screwing rotation

PAUSE_BEFORE_SCREW = False  # True = wait for Enter before each screw
WAIT_FOR_START = True       # True = wait for the 'Start RS003N' button

# Destination(s) (RS003N base frame) - part origin at the end of the slide-in.
# One entry per part, in the same order as the parts passed in from main.py.
# Tuned: X = 0.02, Y = 0.265, Z = 0.255, Rx = 90 deg.
# NOTE: if IK fails on the pre-destination pose, move the RS003N base +x in
# main.py and add the same amount to Y here (part stays at the same world spot).
DESTINATIONS = [
    np.array([0.02, 0.265, 0.255]),
]

# Tool orientation at the destination. Part = tool * Rz(pi) (top-down grasp of a
# part spawned with Ry(pi)), so a part at Rx(90 deg) needs the tool at Rx(-90 deg).
# Tool z-axis -> base +y, so the part slides in along +y.
DEST_ROT = SE3.Rx(-np.pi / 2)


class RS003NScrewing:
    def __init__(self, env, robot, gui, parts, destinations=None):
        """
        env          : swift.Swift environment
        robot        : the RS003N (base already set)
        gui          : RobotGui (from GUI.py)
        parts        : list of meshes to pick and screw in (spawned in main.py)
        destinations : list of base-frame positions (defaults to DESTINATIONS)
        """
        self.env = env
        self.robot = robot
        self.gui = gui
        self.parts = list(parts)
        self.destinations = DESTINATIONS if destinations is None else destinations
        self.base = SE3(robot.base)

        self.qHome = np.array(robot.q, dtype=float)
        self.qHomeCarry = self.qHome.copy()
        self.qHomeCarry[5] = J6_CARRY

        self.attached = []      # [(mesh, T_ee_obj)]
        self.gui.setMoveCallback(robot, self.updateAttached)     # held part follows slider moves

        if len(self.parts) != len(self.destinations):
            print(f"!! Warning: {len(self.parts)} part(s) but {len(self.destinations)} "
                  f"destination(s) - only the first {min(len(self.parts), len(self.destinations))} will be used")

    # ------------------------------------------------------------------
    # Frame helpers
    # ------------------------------------------------------------------
    def toWorld(self, tLocal):
        """Base-frame pose -> world pose."""
        return self.base * tLocal

    def toLocal(self, T):
        """World pose (SE3 or 4x4) -> base-frame pose."""
        return self.base.inv() * SE3(T)

    # ------------------------------------------------------------------
    # Attach / detach
    # ------------------------------------------------------------------
    def attach(self, obj):
        tEeObj = self.robot.fkine(self.robot.q).inv() * SE3(obj.T)
        self.attached.append((obj, tEeObj))
        print("Object attached")

    def detach(self, obj):
        self.attached = [(o, t) for o, t in self.attached if o is not obj]
        print("Object detached")

    def updateAttached(self, q):
        tEe = self.robot.fkine(q)
        for obj, tEeObj in self.attached:
            obj.T = (tEe * tEeObj).A

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def animate(self, qGoal, steps):
        """Smooth joint trajectory to qGoal; freezes on e-stop, re-plans if movedByHand."""
        gui = self.gui
        gui.motionRunning = True
        traj = rtb.jtraj(self.robot.q, qGoal, steps).q
        i = 0
        while i < len(traj):
            if gui.estopActive:
                gui.movedByHand = False
                gui.waitWhileEstopped()
                if gui.movedByHand:
                    print("Joints were moved during the e-stop - re-planning this move")
                    traj = rtb.jtraj(self.robot.q, qGoal, max(len(traj) - i, 2)).q
                    i = 0
            q = traj[i]
            self.robot.q = q
            self.updateAttached(q)
            gui.syncSliders()
            self.env.step(0.02)
            i += 1
        gui.motionRunning = False

    def inLimits(self, idx, angle):
        lo, hi = self.robot.qlim[:, idx]
        if not (lo - 1e-9 <= angle <= hi + 1e-9):
            print(f"Joint {idx + 1} target {np.degrees(angle):.1f} deg is outside "
                  f"its limits [{np.degrees(lo):.1f}, {np.degrees(hi):.1f}] deg")
            return False
        return True

    def solveIK(self, tGoal):
        """Try IK from several seeds; return the solution closest to the current q, or None."""
        qNow = np.array(self.robot.q, dtype=float)
        qJ6Zero = qNow.copy()
        qJ6Zero[5] = 0.0                    # same pose, J6 off its limit
        seeds = [qNow, self.qHomeCarry, self.qHome, qJ6Zero]

        best = None
        for q0 in seeds:
            sol = self.robot.ikine_LM(tGoal, q0=q0, joint_limits=True, ilimit=100, slimit=300)
            if sol.success:
                q = np.array(sol.q, dtype=float)
                cost = np.linalg.norm(q[:5] - qNow[:5])     # ignore J6, it gets overridden anyway
                if best is None or cost < best[0]:
                    best = (cost, q)
        return None if best is None else best[1]

    def moveTo(self, tGoal, steps=60, jointOverrides=None):
        """IK to a WORLD pose, then animate. jointOverrides: {joint_idx: angle_rad}."""
        qGoal = self.solveIK(tGoal)
        if qGoal is None:
            local = self.toLocal(tGoal).t
            print(f"IK failed. Target in RS003N base frame: {np.round(local, 3)} "
                  f"(horizontal reach {np.hypot(local[0], local[1]):.3f} m)")
            return False

        if jointOverrides:
            for idx, angle in jointOverrides.items():
                if not self.inLimits(idx, angle):
                    return False
                qGoal[idx] = angle

        self.animate(qGoal, steps)
        err = np.linalg.norm(self.robot.fkine(self.robot.q).t - tGoal.t)
        print(f"Reached target, position error = {err * 1000:.2f} mm")
        return True

    def rotateJoint(self, idx, angle, steps=100):
        if not self.inLimits(idx, angle):
            return False
        qGoal = np.array(self.robot.q, dtype=float)
        qGoal[idx] = angle
        self.animate(qGoal, steps)
        print(f"Joint {idx + 1} now at {np.degrees(self.robot.q[idx]):.1f} deg")
        return True

    def moveToJoints(self, qGoal, steps=80):
        self.animate(np.array(qGoal, dtype=float), steps)
        print("Reached joint configuration")
        return True

    # ------------------------------------------------------------------
    # Pose builders (built in the base frame, returned in WORLD frame)
    # ------------------------------------------------------------------
    def pickPoses(self, part):
        p = self.toLocal(part.T).t
        tApproach = self.toWorld(SE3.Trans(p[0], p[1], p[2] + APPROACH_HEIGHT) * SE3.Rx(np.pi))
        tGrasp = self.toWorld(SE3.Trans(p[0], p[1], p[2] + GRASP_OFFSET) * SE3.Rx(np.pi))
        return tApproach, tGrasp

    def placePoses(self, dest):
        tDest = self.toWorld(SE3.Trans(*dest) * DEST_ROT)
        tPreDest = tDest * SE3.Tz(-SLIDE_DISTANCE)
        return tPreDest, tDest

    # ------------------------------------------------------------------
    # One cycle: pick -> carry -> slide in -> screw -> release -> back off
    # ------------------------------------------------------------------
    def pickAndScrew(self, part, dest, n):
        print(f"\n=== Part {n}: to {dest} (base frame) ===")
        hold = {5: J6_CARRY}
        tApproach, tGrasp = self.pickPoses(part)
        tPreDest, tDest = self.placePoses(dest)

        # 1. Approach from above, descend, grab
        if not (self.moveTo(tApproach, jointOverrides=hold)
                and self.moveTo(tGrasp, steps=30, jointOverrides=hold)):
            return False
        self.attach(part)

        # 2. Lift straight up (along base z) until the part is LIFT_HEIGHT above the base
        dz = LIFT_HEIGHT - self.toLocal(part.T).t[2]
        tLift = self.base * SE3.Tz(dz) * self.base.inv() * self.robot.fkine(self.robot.q)
        if not self.moveTo(tLift, steps=30, jointOverrides=hold):
            return False

        # 3. Carry to pre-destination, then slide straight in
        if not (self.moveTo(tPreDest, steps=80, jointOverrides=hold)
                and self.moveTo(tDest, steps=40, jointOverrides=hold)):
            return False

        # 4. Screw it in
        if PAUSE_BEFORE_SCREW:
            input(f"Part {n} in position. Press Enter to screw it in...")
        if not self.rotateJoint(5, J6_SCREW, steps=SCREW_STEPS):
            return False

        # 5. Release
        self.detach(part)

        # 6. Back out along the tool axis
        tRetreat = self.robot.fkine(self.robot.q) * SE3.Tz(-SLIDE_DISTANCE)
        self.moveTo(tRetreat, steps=40)

        # 7. Home, unwinding joint 6 to the carry angle
        self.moveToJoints(self.qHomeCarry, steps=100)
        return True

    # ------------------------------------------------------------------
    # Full sequence
    # ------------------------------------------------------------------
    def run(self):
        """Run the whole fastening sequence. Returns True if every part was placed."""
        if WAIT_FOR_START:
            self.gui.waitForStart()

        success = False
        if self.rotateJoint(5, J6_CARRY, steps=100):
            success = True
            for i, (part, dest) in enumerate(zip(self.parts, self.destinations), start=1):
                if not self.pickAndScrew(part, dest, i):
                    print(f"Stopped: part {i} failed")
                    success = False
                    break
            else:
                print("\nAll parts placed")

            self.moveToJoints(self.qHome, steps=120)

        return success