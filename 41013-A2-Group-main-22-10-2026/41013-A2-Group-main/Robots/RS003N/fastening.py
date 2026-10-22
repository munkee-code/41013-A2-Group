# numpy: arrays and maths (angles, norms, vectors)
import numpy as np
# roboticstoolbox: used here for rtb.jtraj, the smooth joint-space trajectory generator
import roboticstoolbox as rtb
# SE3: a 4x4 pose (position + orientation). "A * B" chains poses: apply B in A's frame
from spatialmath import SE3

# ----------------------------------------------------------------------
# Settings - tune these
# ----------------------------------------------------------------------
# Hover this far above a part before going down onto it
APPROACH_HEIGHT = 0.10      # m above a part before descending onto it
# Extra height above the part's origin where the gripper closes (0 = right at the origin)
GRASP_OFFSET = 0.0          # m above the part origin when gripping
# After picking up, raise the part until it is this high above the robot base frame
LIFT_HEIGHT = 0.10          # part height above the robot base after pick-up (m)
# Distance to back off along the tool's z-axis, so the part can slide in straight
SLIDE_DISTANCE = 0.10       # m backed off along the tool axis before sliding in

# Joint 6 angle while picking and carrying: -360 deg (a full turn, at its lower limit)
J6_CARRY = -2 * np.pi       # joint 6 angle while picking and carrying (rad)
# Joint 6 angle at the end of the screw: +360 deg. Going from -2pi to +2pi is two full turns
J6_SCREW = 2 * np.pi        # joint 6 angle at the end of the "screw" (rad)
# How many animation frames the screw rotation takes (more = slower and smoother)
SCREW_STEPS = 200           # animation steps for the screwing rotation

# If True, the program stops and waits for Enter before the screwing motion
PAUSE_BEFORE_SCREW = False  # True = wait for Enter before each screw
# If True, run() waits for the Start button in the GUI before moving anything
WAIT_FOR_START = True       # True = wait for the 'Start RS003N' button

# Destination(s) (RS003N base frame) - part origin at the end of the slide-in.
# One entry per part, in the same order as the parts passed in from main.py.
# Tuned: X = 0.02, Y = 0.265, Z = 0.255, Rx = 90 deg.
# NOTE: if IK fails on the pre-destination pose, move the RS003N base +x in
# main.py and add the same amount to Y here (part stays at the same world spot).
# List of target positions, one [x, y, z] per part, in metres, in the robot's base frame
DESTINATIONS = [
    np.array([0.02, 0.265, 0.255]),
]

# Tool orientation at the destination. Part = tool * Rz(pi) (top-down grasp of a
# part spawned with Ry(pi)), so a part at Rx(90 deg) needs the tool at Rx(-90 deg).
# Tool z-axis -> base +y, so the part slides in along +y.
# A pure rotation of -90 deg about x: turns the tool so its z-axis points along base +y
DEST_ROT = SE3.Rx(-np.pi / 2)


class RS003NScrewing:
    # env = the Swift scene, robot = the RS003N object, gui = your slider/e-stop/start
    # GUI object, parts = the meshes to pick up, destinations = optional override list
    def __init__(self, env, robot, gui, parts, destinations=None):
        # Keep references so the other methods can use them
        self.env = env
        self.robot = robot
        self.gui = gui
        # Copy the parts into a list (so any iterable works)
        self.parts = list(parts)
        # Use the file-level DESTINATIONS unless the caller supplied their own
        self.destinations = DESTINATIONS if destinations is None else destinations
        # The robot base's pose in the world. Used to convert between world and base frames
        self.base = SE3(robot.base)

        # Remember the starting joint angles as the "home" configuration
        self.qHome = np.array(robot.q, dtype=float)
        # Same as home, but with joint 6 at the carry angle (-2pi)
        self.qHomeCarry = self.qHome.copy()
        self.qHomeCarry[5] = J6_CARRY

        # Held objects live here as (mesh, pose of the mesh relative to the end-effector)
        self.attached = []      # [(mesh, T_ee_obj)]
        # Ask the GUI to call updateAttached whenever a slider moves the robot, so a
        # held part follows slider moves too (setMoveCallback is defined in your GUI class)
        self.gui.setMoveCallback(robot, self.updateAttached)     # held part follows slider moves

        # Warn if the number of parts and destinations differ. zip() later stops at the shorter
        if len(self.parts) != len(self.destinations):
            print(f"!! Warning: {len(self.parts)} part(s) but {len(self.destinations)} "
                  f"destination(s) - only the first {min(len(self.parts), len(self.destinations))} will be used")

    # ------------------------------------------------------------------
    # Frame helpers
    # ------------------------------------------------------------------
    def toWorld(self, tLocal): # Convert a pose in the robot's base frame to a world frame pose
        """Base-frame pose -> world pose."""
        # World pose = base pose, then the local pose applied in the base's frame
        return self.base * tLocal

    def toLocal(self, T): # Convert a pose in the world frame to a pose in the robot's base frame
        """World pose (SE3 or 4x4) -> base-frame pose."""
        # Undo the base transform. SE3(T) lets this accept either an SE3 or a raw 4x4 array
        return self.base.inv() * SE3(T)

    # ------------------------------------------------------------------
    # Attach / detach
    # ------------------------------------------------------------------
    def attach(self, obj): # Attach an object to the robot
        # Pose of the object RELATIVE to the end-effector right now:
        # inverse(end-effector pose in world) * (object pose in world)
        tEeObj = self.robot.fkine(self.robot.q).inv() * SE3(obj.T)
        # Store it. From now on the object keeps this fixed offset from the gripper
        self.attached.append((obj, tEeObj))
        print("Object attached")

    def detach(self, obj): # Detach an object from the robot
        # Rebuild the list without this object (the "is not" test compares identity)
        self.attached = [(o, t) for o, t in self.attached if o is not obj]
        print("Object detached")

    def updateAttached(self, q): # Update the pose of any attached objects to follow the robot's end-effector
        # Where the end-effector is for joint angles q
        tEe = self.robot.fkine(q)
        # For every held object: new world pose = end-effector pose * stored offset
        for obj, tEeObj in self.attached:
            obj.T = (tEe * tEeObj).A      # .A gives the raw 4x4 array the mesh expects

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def animate(self, qGoal, steps):
        """Smooth joint trajectory to qGoal; freezes on e-stop, re-plans if movedByHand."""
        gui = self.gui
        # Tell the GUI a move is in progress (your GUI uses this flag; defined in its class)
        gui.motionRunning = True
        # Smooth path from the current joints to qGoal, `steps` rows, one per frame
        traj = rtb.jtraj(self.robot.q, qGoal, steps).q
        # Frame index. A while loop is used (not for) so we can reset it when re-planning
        i = 0
        while i < len(traj):
            # If the emergency stop is pressed, freeze here
            if gui.estopActive:
                # Clear the flag, then block until the e-stop is released
                gui.movedByHand = False
                gui.waitWhileEstopped()
                # If joints were moved with the sliders while stopped, the old path is wrong
                if gui.movedByHand:
                    print("Joints were moved during the e-stop - re-planning this move")
                    # New path from wherever the joints are now, over the remaining frames (min 2)
                    traj = rtb.jtraj(self.robot.q, qGoal, max(len(traj) - i, 2)).q
                    i = 0
            # Joint angles for this frame
            q = traj[i]
            # Apply them to the robot
            self.robot.q = q
            # Move any held part with the gripper
            self.updateAttached(q)
            # Update the GUI sliders to show the new angles
            gui.syncSliders()
            # Advance the Swift scene by 0.02 s, which draws the frame
            self.env.step(0.02)
            i += 1
        # Move finished
        gui.motionRunning = False

    def inLimits(self, idx, angle):
        # robot.qlim is a 2 x n array (row 0 = lower limits, row 1 = upper). Take joint idx's pair
        lo, hi = self.robot.qlim[:, idx]
        # Allow a tiny tolerance (1e-9 rad) so a value exactly on the limit passes
        if not (lo - 1e-9 <= angle <= hi + 1e-9):
            print(f"Joint {idx + 1} target {np.degrees(angle):.1f} deg is outside "
                  f"its limits [{np.degrees(lo):.1f}, {np.degrees(hi):.1f}] deg")
            return False
        return True

    def solveIK(self, tGoal):
        """Try IK from several seeds; return the solution closest to the current q, or None."""
        # Current joint angles
        qNow = np.array(self.robot.q, dtype=float)
        # Copy of the current pose with joint 6 set to 0, in case J6 sitting on its limit hurts IK
        qJ6Zero = qNow.copy()
        qJ6Zero[5] = 0.0                    # same pose, J6 off its limit
        # IK is sensitive to its starting guess, so try several different starts
        seeds = [qNow, self.qHomeCarry, self.qHome, qJ6Zero]

        # Best (cost, q) found so far
        best = None
        for q0 in seeds:
            # Levenberg-Marquardt IK, keeping joints within limits.
            # ilimit = max iterations per attempt, slimit = max restarts with random seeds
            sol = self.robot.ikine_LM(tGoal, q0=q0, joint_limits=True, ilimit=100, slimit=300)
            if sol.success:
                q = np.array(sol.q, dtype=float)
                # Distance from the current pose over joints 1-5 (indices 0-4).
                # Joint 6 is left out because the caller overrides it anyway
                cost = np.linalg.norm(q[:5] - qNow[:5])     # ignore J6, it gets overridden anyway
                # Keep the solution that needs the least joint movement
                if best is None or cost < best[0]:
                    best = (cost, q)
        # No seed worked -> None, otherwise the best joint vector
        return None if best is None else best[1]

    def moveTo(self, tGoal, steps=60, jointOverrides=None):
        """IK to a WORLD pose, then animate. jointOverrides: {joint_idx: angle_rad}."""
        # Solve for joint angles that put the end-effector at tGoal
        qGoal = self.solveIK(tGoal)
        if qGoal is None:
            # Report the target in the base frame, which is how DESTINATIONS is written
            local = self.toLocal(tGoal).t
            print(f"IK failed. Target in RS003N base frame: {np.round(local, 3)} "
                  f"(horizontal reach {np.hypot(local[0], local[1]):.3f} m)")
            return False

        # Force chosen joints to fixed angles (e.g. {5: J6_CARRY} pins joint 6)
        if jointOverrides:
            for idx, angle in jointOverrides.items():
                # Refuse if the forced angle is outside that joint's limits
                if not self.inLimits(idx, angle):
                    return False
                qGoal[idx] = angle

        # Play the move
        self.animate(qGoal, steps)
        # How far the end-effector ended up from the target, in mm
        err = np.linalg.norm(self.robot.fkine(self.robot.q).t - tGoal.t)
        print(f"Reached target, position error = {err * 1000:.2f} mm")
        return True

    def rotateJoint(self, idx, angle, steps=100):
        # Check the target is within limits first
        if not self.inLimits(idx, angle):
            return False
        # Start from the current joints and change only joint idx
        qGoal = np.array(self.robot.q, dtype=float)
        qGoal[idx] = angle
        self.animate(qGoal, steps)
        print(f"Joint {idx + 1} now at {np.degrees(self.robot.q[idx]):.1f} deg")
        return True

    def moveToJoints(self, qGoal, steps=80):
        # Straight joint-space move to a given set of joint angles (no IK needed)
        self.animate(np.array(qGoal, dtype=float), steps)
        print("Reached joint configuration")
        return True

    # ------------------------------------------------------------------
    # Pose builders (built in the base frame, returned in WORLD frame)
    # ------------------------------------------------------------------
    def pickPoses(self, part):
        # Part's position in the base frame ([x, y, z])
        p = self.toLocal(part.T).t
        # Pose above the part: same x, y, raised by APPROACH_HEIGHT. Rx(pi) flips the tool
        # 180 deg about x so its z-axis points down at the part. Converted back to world frame
        tApproach = self.toWorld(SE3.Trans(p[0], p[1], p[2] + APPROACH_HEIGHT) * SE3.Rx(np.pi))
        # Same, but at grip height (part z + GRASP_OFFSET)
        tGrasp = self.toWorld(SE3.Trans(p[0], p[1], p[2] + GRASP_OFFSET) * SE3.Rx(np.pi))
        return tApproach, tGrasp

    def placePoses(self, dest):
        # Final tool pose: position `dest` (unpacked with *), orientation DEST_ROT, in world frame
        tDest = self.toWorld(SE3.Trans(*dest) * DEST_ROT)
        # Pre-destination: back off SLIDE_DISTANCE along the TOOL's own z-axis
        # (right-multiplying moves in the tool frame, so this is the straight line to slide in on)
        tPreDest = tDest * SE3.Tz(-SLIDE_DISTANCE)
        return tPreDest, tDest

    # ------------------------------------------------------------------
    # One cycle: pick -> carry -> slide in -> screw -> release -> back off
    # ------------------------------------------------------------------
    def pickAndScrew(self, part, dest, n):
        print(f"\n=== Part {n}: to {dest} (base frame) ===")
        # Joint 6 is pinned to the carry angle during every IK move in this cycle
        hold = {5: J6_CARRY}
        # Build the poses for picking this part and placing it
        tApproach, tGrasp = self.pickPoses(part)
        tPreDest, tDest = self.placePoses(dest)

        # 1. Approach from above, descend, grab
        # `and` short-circuits: the descent only runs if the approach succeeded
        if not (self.moveTo(tApproach, jointOverrides=hold)
                and self.moveTo(tGrasp, steps=30, jointOverrides=hold)):
            return False
        # Gripper is at the part: lock the part to the end-effector
        self.attach(part)

        # 2. Lift straight up (along base z) until the part is LIFT_HEIGHT above the base
        # How far to raise: target height minus the part's current height in the base frame
        dz = LIFT_HEIGHT - self.toLocal(part.T).t[2]
        # Current end-effector pose, shifted by dz along the BASE z-axis:
        # go to base frame, shift, go back to world, then apply to the current pose
        tLift = self.base * SE3.Tz(dz) * self.base.inv() * self.robot.fkine(self.robot.q)
        if not self.moveTo(tLift, steps=30, jointOverrides=hold):
            return False

        # 3. Carry to pre-destination, then slide straight in
        if not (self.moveTo(tPreDest, steps=80, jointOverrides=hold)
                and self.moveTo(tDest, steps=40, jointOverrides=hold)):
            return False

        # 4. Screw it in
        # Optional manual pause (blocks on the console until Enter is pressed)
        if PAUSE_BEFORE_SCREW:
            input(f"Part {n} in position. Press Enter to screw it in...")
        # Turn joint 6 from -2pi to +2pi (two full turns). The tool tip stays put while it spins
        if not self.rotateJoint(5, J6_SCREW, steps=SCREW_STEPS):
            return False

        # 5. Release
        # The part stays where it is and stops following the gripper
        self.detach(part)

        # 6. Back out along the tool axis
        # Current end-effector pose, moved back along the tool's z-axis by SLIDE_DISTANCE
        tRetreat = self.robot.fkine(self.robot.q) * SE3.Tz(-SLIDE_DISTANCE)
        # No jointOverrides here, so joint 6 is free. The return value isn't checked
        self.moveTo(tRetreat, steps=40)

        # 7. Home, unwinding joint 6 to the carry angle
        # Joint-space move back to home, with joint 6 set to the carry angle ready for the next part
        self.moveToJoints(self.qHomeCarry, steps=100)
        return True

    # ------------------------------------------------------------------
    # Full sequence
    # ------------------------------------------------------------------
    def run(self):
        """Run the whole fastening sequence. Returns True if every part was placed."""
        # Block until the Start button in the GUI is pressed
        if WAIT_FOR_START:
            self.gui.waitForStart()

        success = False
        # First wind joint 6 to the carry angle. Only continue if that worked
        if self.rotateJoint(5, J6_CARRY, steps=100):
            success = True
            # Pair each part with its destination; numbering starts at 1 for the printouts
            for i, (part, dest) in enumerate(zip(self.parts, self.destinations), start=1):
                if not self.pickAndScrew(part, dest, i):
                    print(f"Stopped: part {i} failed")
                    success = False
                    break
            # for/else: the else block runs only if the loop finished WITHOUT hitting `break`
            else:
                print("\nAll parts placed")

            # Whether it succeeded or stopped early, go back to the original home pose
            self.moveToJoints(self.qHome, steps=120)

        return success