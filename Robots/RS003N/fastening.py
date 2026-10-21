import numpy as np
import swift
import roboticstoolbox as rtb
from spatialmath import SE3
from ir_support_extra_parts.parts import part_mesh

# ----------------------------------------------------------------------
# Settings - tune these
# ----------------------------------------------------------------------
APPROACH_HEIGHT = 0.10      # m above a glass before descending onto it
GRASP_OFFSET = 0.0          # m above the glass origin when gripping
LIFT_HEIGHT = 0.10          # glass height above the robot base after pick-up (m)
SLIDE_DISTANCE = 0.10       # m backed off along the tool axis before sliding in

J6_CARRY = -2 * np.pi       # joint 6 angle while picking and carrying (rad)
J6_SCREW = 2 * np.pi        # joint 6 angle at the end of the "screw" (rad)
SCREW_STEPS = 200           # animation steps for the screwing rotation

PAUSE_BEFORE_SCREW = False  # True = wait for Enter before each screw
WAIT_FOR_START = True       # True = wait for the 'Start RS003N' button

# Pick-up positions (RS003N base frame)
GLASS_STARTS = [
    (-0.3, -0.30, 0.05),
]

# Destinations (RS003N base frame): cross pattern in the base x-z plane at
# y = 0.35, every glass slides in along base +y.
DESTINATIONS = [
    np.array([0.02, 0.265, 0.255]),
]

# Tool orientation at a destination: tool z-axis -> base +y
DEST_ROT = SE3.Rx(-np.pi / 2)


class RS003NScrewing:
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot
        self.base = SE3(robot.base)

        self.qHome = np.array(robot.q, dtype=float)
        self.qHomeCarry = self.qHome.copy()
        self.qHomeCarry[5] = J6_CARRY

        self.startPressed = False
        self.estopActive = False
        self.motionRunning = False
        self.jogged = False
        self.continuePressed = False

        self.attached = []      # [(mesh, T_ee_obj)]
        self.jointSliders = []
        self.glasses = []

        self._addButtons()
        self._addSliders()
        self._spawnGlasses()

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
    # Scene setup
    # ------------------------------------------------------------------
    def _spawnGlasses(self):
        for x, y, z in GLASS_STARTS:
            pose = self.toWorld(SE3.Trans(x, y, z) * SE3.Ry(np.pi))
            g = part_mesh("ShotGlass", pose=pose)
            self.env.add(g)
            self.glasses.append(g)

    def _addButtons(self):
        def onStart(_=None):
            self.startPressed = True

        def onEstop(_=None):
            if not self.estopActive:
                self.estopActive = True
                print("\n*** E-STOP PRESSED - RS003N halted. Press 'Reset E-stop' to resume. ***")

        def onReset(_=None):
            if self.estopActive:
                self.estopActive = False
                print("E-stop reset - resuming")

        self.env.add(swift.Button(onStart, desc="Start RS003N"))
        self.env.add(swift.Button(onEstop, desc="E-STOP"))
        self.env.add(swift.Button(onReset, desc="Reset E-stop"))

    def _jogAllowed(self):
        return (not self.motionRunning) or self.estopActive

    def _makeJointSlider(self, j):
        def cb(value):
            if not self._jogAllowed():
                return
            deg = float(value)
            if abs(np.degrees(self.robot.q[j]) - deg) < 0.05:   # echo of syncSliders()
                return
            q = np.array(self.robot.q, dtype=float)
            q[j] = np.radians(deg)
            self.robot.q = q
            self.updateAttached(q)
            self.jogged = True
        return cb

    def _addSliders(self):
        for j in range(6):
            lo, hi = np.degrees(self.robot.qlim[:, j])
            lo = lo if np.isfinite(lo) else -360.0
            hi = hi if np.isfinite(hi) else 360.0
            s = swift.Slider(self._makeJointSlider(j),
                             min=round(float(lo), 1), max=round(float(hi), 1),
                             step=0.5, value=round(float(np.degrees(self.robot.q[j])), 1),
                             desc=f"RS003N Joint {j + 1}", unit="&#176;")
            self.env.add(s)
            self.jointSliders.append(s)

    def syncSliders(self):
        for j, s in enumerate(self.jointSliders):
            deg = round(float(np.degrees(self.robot.q[j])), 1)
            if abs(float(s.value) - deg) > 0.25:
                s.value = deg

    def waitWhileEstopped(self):
        while self.estopActive:
            self.env.step(0.05)

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
        """Smooth joint trajectory to qGoal; freezes on e-stop, re-plans if jogged."""
        self.motionRunning = True
        traj = rtb.jtraj(self.robot.q, qGoal, steps).q
        i = 0
        while i < len(traj):
            if self.estopActive:
                self.jogged = False
                self.waitWhileEstopped()
                if self.jogged:
                    print("Joints were moved during the e-stop - re-planning this move")
                    traj = rtb.jtraj(self.robot.q, qGoal, max(len(traj) - i, 2)).q
                    i = 0
            q = traj[i]
            self.robot.q = q
            self.updateAttached(q)
            self.syncSliders()
            self.env.step(0.02)
            i += 1
        self.motionRunning = False

    def inLimits(self, idx, angle):
        lo, hi = self.robot.qlim[:, idx]
        if not (lo - 1e-9 <= angle <= hi + 1e-9):
            print(f"Joint {idx + 1} target {np.degrees(angle):.1f} deg is outside "
                  f"its limits [{np.degrees(lo):.1f}, {np.degrees(hi):.1f}] deg")
            return False
        return True

    def moveTo(self, tGoal, steps=60, jointOverrides=None):
        """IK to a WORLD pose, then animate. jointOverrides: {joint_idx: angle_rad}."""
        sol = self.robot.ikine_LM(tGoal, q0=self.robot.q, joint_limits=True)
        if not sol.success:
            print(f"IK failed for target:\n{tGoal}")
            return False

        qGoal = np.array(sol.q, dtype=float)
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
    def pickPoses(self, glass):
        p = self.toLocal(glass.T).t
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
    def pickAndScrew(self, glass, dest, n):
        print(f"\n=== Glass {n}: to {dest} (base frame) ===")
        hold = {5: J6_CARRY}
        tApproach, tGrasp = self.pickPoses(glass)
        tPreDest, tDest = self.placePoses(dest)

        # 1. Approach from above, descend, grab
        if not (self.moveTo(tApproach, jointOverrides=hold)
                and self.moveTo(tGrasp, steps=30, jointOverrides=hold)):
            return False
        self.attach(glass)

        # 2. Lift straight up (along base z) until the glass is LIFT_HEIGHT above the base
        dz = LIFT_HEIGHT - self.toLocal(glass.T).t[2]
        tLift = self.base * SE3.Tz(dz) * self.base.inv() * self.robot.fkine(self.robot.q)
        if not self.moveTo(tLift, steps=30, jointOverrides=hold):
            return False

        # 3. Carry to pre-destination, then slide straight in
        if not (self.moveTo(tPreDest, steps=80, jointOverrides=hold)
                and self.moveTo(tDest, steps=40, jointOverrides=hold)):
            return False

        # 4. Screw it in
        if PAUSE_BEFORE_SCREW:
            input(f"Glass {n} in position. Press Enter to screw it in...")
        if not self.rotateJoint(5, J6_SCREW, steps=SCREW_STEPS):
            return False

        # 5. Release
        self.detach(glass)

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
        """Run the whole fastening sequence. Returns True if every glass was placed."""
        if WAIT_FOR_START:
            print(">> Press 'Start RS003N' in Swift to begin fastening...")
            while not self.startPressed or self.estopActive:
                self.env.step(0.05)

        success = False
        if self.rotateJoint(5, J6_CARRY, steps=100):
            success = True
            for i, (glass, dest) in enumerate(zip(self.glasses, DESTINATIONS), start=1):
                if not self.pickAndScrew(glass, dest, i):
                    print(f"Stopped: glass {i} failed")
                    success = False
                    break
            else:
                print("\nAll glasses placed")

            self.moveToJoints(self.qHome, steps=120)

        return success

    def holdForJog(self, label="Continue to next state"):
        """Keep Swift responsive (sliders work) until the button is pressed."""
        self.continuePressed = False

        def onContinue(_=None):
            self.continuePressed = True

        self.env.add(swift.Button(onContinue, desc=label))
        print(f">> Sliders active - press '{label}' in Swift to continue")
        while not self.continuePressed:
            self.env.step(0.05)