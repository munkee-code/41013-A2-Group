# numpy: angle conversion (degrees <-> radians), clipping, array handling
import numpy as np
# swift: the simulator. Its Label, Select, Slider and Button widgets make up the side panel
import swift
# SE3 is imported but not used anywhere in this file
from spatialmath import SE3


class RobotGui:
    # env = the Swift scene, robots = dict of {display name: robot object},
    # startLabel = text shown on the Start button
    def __init__(self, env, robots, startLabel="Start RS003N"):
        """
        env        : swift.Swift environment
        robots     : dict {display name: robot}; the first one is selected at start
        startLabel : text on the Start button (read by waitForStart())
        """
        self.env = env
        # Copy the dict so later changes by the caller don't affect the GUI
        self.robots = dict(robots)
        # Display names in order (these fill the dropdown)
        self.names = list(self.robots)
        # The first robot in the dict is the one the sliders control at start
        self.selected = self.names[0]
        self.startLabel = startLabel

        # Shared state - read/written by motion code (e.g. fastening.py)
        # Set True by the Start button; waitForStart() watches it
        self.startPressed = False
        # True while the E-STOP is active; the motion code's animate loop checks it every frame
        self.estopActive = False
        self.motionRunning = False   # True while code is driving a robot - slider moves are blocked
        self.movedByHand = False          # set when a slider moves a robot by hand
        # Set True by the "Next state" button; waitForNext() watches it
        self.nextPressed = False

        # Maps id(robot) -> function to call after a slider moves that robot
        self._moveCallbacks = {}      # id(robot) -> fn(q), e.g. so a held part follows a slider move
        # The slider widgets, one per joint (filled in by _addJointSliders)
        self.jointSliders = []

        # State display - added first so it sits at the top of the panel
        self.stateText = "State: -"
        # Widgets appear in the panel in the order they're added to the env
        self.stateLabel = swift.Label(self.stateText)
        self.env.add(self.stateLabel)

        # E-stop display - right under the state
        self.estopLabel = swift.Label(self._estopText())
        self.env.add(self.estopLabel)

        # Build the rest of the panel: dropdown, then sliders, then buttons
        self._addSelector()
        self._addJointSliders()
        self._addButtons()
        print(f">> GUI ready - moving {self.selected}")

    # Makes `gui.robot` read like an attribute (no brackets) while always returning the current choice
    @property
    def robot(self):
        """The robot currently selected in the dropdown."""
        return self.robots[self.selected]

    def setMoveCallback(self, robot, fn):
        """Call fn(q) whenever `robot` is moved by hand with the sliders."""
        # Keyed by id(robot) so each robot can have its own callback (e.g. RS003N's held-part update)
        self._moveCallbacks[id(robot)] = fn

    # ------------------------------------------------------------------
    # State display
    # ------------------------------------------------------------------
    def setState(self, number, name):
        """Show the current system state at the top of the GUI, e.g. setState(1, "Pick & Place")."""
        self.stateText = f"State {number}: {name}"
        self._setText(self.stateLabel, self.stateText)
        self.env.step(0.01)                                    # push the new text to Swift now

    def _estopText(self):
        # Text for the e-stop label, depending on whether it's active
        return "E-STOP: ACTIVE - press 'Reset E-stop'" if self.estopActive else "E-STOP: off"

    def _updateEstopLabel(self):
        self._setText(self.estopLabel, self._estopText())

    # ------------------------------------------------------------------
    # Robot dropdown
    # ------------------------------------------------------------------
    def _resolveName(self, value):
        # Swift may hand back the option text...
        if value in self.robots:
            return value
        # ...or its position in the list, so try treating it as an index
        try:
            return self.names[int(value)]
        # Not a number, out of range, or None: no robot matches
        except (ValueError, IndexError, TypeError):
            return None

    def _addSelector(self):
        # Called by Swift whenever the dropdown changes
        def cb(value):
            name = self._resolveName(value)
            # Ignore unknown values and re-selecting the robot that's already selected
            if name is None or name == self.selected:
                return
            self.selected = name
            # Re-label the sliders and give them the new robot's limits and angles
            self._applyRobotToSliders()
            # Build a text list of joint limits like "J1 -160..160, J2 -60..150, ..."
            lims = ", ".join(f"J{j + 1} {lo:.0f}..{hi:.0f}"
                             for j, (lo, hi) in ((j, self._limitsDeg(self.robot, j))
                                                 for j in range(self.robot.n)))
            print(f">> GUI now moving {name} (deg: {lims})")

        # The dropdown itself. value=0 selects the first option
        self.env.add(swift.Select(cb, desc="Robot to move", options=self.names, value=0))

    # ------------------------------------------------------------------
    # Joint sliders (one shared set, bound to the selected robot)
    # ------------------------------------------------------------------
    def moveAllowed(self):
        # Sliders work when no program motion is running. During an e-stop they work too,
        # so you can adjust joints by hand while stopped
        return (not self.motionRunning) or self.estopActive

    # Builds the callback for slider j. A separate method so each callback remembers its own j
    def _makeJointSlider(self, j):
        def cb(value):
            # Ignore slider moves while the robot is being driven by code
            if not self.moveAllowed():
                return
            robot = self.robot
            if j >= robot.n:                                   # robot has fewer joints
                return
            deg = float(value)
            # syncSliders() moves the sliders to match the robot, which triggers this callback.
            # If the slider already equals the robot's angle, it's that echo, so do nothing
            if abs(np.degrees(robot.q[j]) - deg) < 0.05:       # echo of syncSliders()
                return

            # This robot's lower and upper limit for joint j, in radians
            lo, hi = robot.qlim[:, j]
            # Convert the slider's degrees to radians and clamp to the limits
            rad = float(np.clip(np.radians(deg), lo, hi))      # respect this robot's limits
            # Copy the joint vector, change joint j, and assign it back (this updates the robot's pose)
            q = np.array(robot.q, dtype=float)
            q[j] = rad
            robot.q = q

            # If a callback is registered for this robot (e.g. to move a held part), run it
            fn = self._moveCallbacks.get(id(robot))
            if fn is not None:
                fn(q)
            # Record that the user moved joints by hand (animate() uses this to re-plan after an e-stop)
            self.movedByHand = True

            # If clamping changed the value, move the slider back to the limit
            if abs(np.degrees(rad) - deg) > 0.05:              # snap slider back inside limits
                self.jointSliders[j].value = round(float(np.degrees(rad)), 1)
        return cb

    # A static method needs no `self`; it can be called as RobotGui._limitsDeg(robot, j)
    @staticmethod
    def _limitsDeg(robot, j):
        """Joint j limits in degrees, read from the robot's own qlim (its DH links)."""
        lo, hi = np.degrees(robot.qlim[:, j])
        # A joint with no limit shows as infinity; use +/-360 deg instead so the slider has a range
        lo = lo if np.isfinite(lo) else -360.0
        hi = hi if np.isfinite(hi) else 360.0
        return round(float(lo), 1), round(float(hi), 1)

    @staticmethod
    def _setText(s, text):
        """Change the text of a slider or label (newer Swift uses .label, older uses .desc)."""
        try:
            # Check the class for a `label` property; if it has one, use it, otherwise use `desc`
            if hasattr(type(s), "label"):
                s.label = text
            else:
                s.desc = text
        except AttributeError:
            pass                                               # this Swift can't change element text

    def _addJointSliders(self):
        # One shared set of sliders, sized for the robot with the most joints
        nJoints = max(r.n for r in self.robots.values())
        robot = self.robot
        for j in range(nJoints):
            # Start with the selected robot's own limits; _applyRobotToSliders() swaps them on select
            # (if the robot has no joint j, fall back to +/-360 deg)
            lo, hi = self._limitsDeg(robot, j) if j < robot.n else (-360.0, 360.0)
            # Start at the robot's current angle, or 0 for a joint it doesn't have
            value = np.degrees(robot.q[j]) if j < robot.n else 0.0
            # Create the slider. The callback comes from _makeJointSlider(j); step is half a degree;
            # "&#176;" is the HTML code for the degree sign
            s = swift.Slider(self._makeJointSlider(j),
                             min=lo, max=hi,
                             step=0.5, value=round(float(value), 1),
                             desc=f"{self.selected} J{j + 1}", unit="&#176;")
            self.env.add(s)
            # Keep a handle so we can change its range, label and value later
            self.jointSliders.append(s)

    def _applyRobotToSliders(self):
        """Give every slider the selected robot's joint limits, label and current angle."""
        robot = self.robot
        for j, s in enumerate(self.jointSliders):
            # Sliders beyond this robot's joint count are marked unused
            if j >= robot.n:
                self._setText(s, f"(unused) J{j + 1}")
                continue
            lo, hi = self._limitsDeg(robot, j)
            s.min = lo                                         # set range first, then the value inside it
            s.max = hi
            s.value = round(float(np.degrees(robot.q[j])), 1)
            self._setText(s, f"{self.selected} J{j + 1}")

    def syncSliders(self, force=False):
        """Push the selected robot's joint angles to the sliders."""
        robot = self.robot
        for j, s in enumerate(self.jointSliders):
            if j >= robot.n:
                continue
            # The robot's joint angle in degrees, rounded to the slider's precision
            deg = round(float(np.degrees(robot.q[j])), 1)
            # Only update a slider if it's off by more than 0.25 deg (or force=True).
            # This avoids needless updates during animation
            if force or abs(float(s.value) - deg) > 0.25:
                s.value = deg

    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------
    def _addButtons(self):
        # Each handler takes an optional argument because Swift may pass one; it's ignored
        def onStart(_=None):
            self.startPressed = True

        def onEstop(_=None):
            # Only act on the first press, so repeated clicks don't print repeatedly
            if not self.estopActive:
                self.estopActive = True
                self._updateEstopLabel()
                print("\n*** E-STOP PRESSED - motion halted. Press 'Reset E-stop' to resume. ***")

        def onReset(_=None):
            # Only does something if the e-stop is currently active
            if self.estopActive:
                self.estopActive = False
                self._updateEstopLabel()
                print("E-stop reset - resuming")

        def onNext(_=None):
            self.nextPressed = True

        # The buttons, in panel order. The handlers only set flags; the code waiting on
        # them (below) does the actual work
        self.env.add(swift.Button(onStart, desc=self.startLabel))
        self.env.add(swift.Button(onEstop, desc="E-STOP"))
        self.env.add(swift.Button(onReset, desc="Reset E-stop"))
        self.env.add(swift.Button(onNext, desc="Next state"))

    # ------------------------------------------------------------------
    # Waiting helpers (all keep Swift responsive)
    # ------------------------------------------------------------------
    def waitForStart(self):
        # Clear any earlier press so an old click doesn't skip the wait
        self.startPressed = False
        print(f">> Press '{self.startLabel}' in Swift to begin...")
        # Loop until Start is pressed and the e-stop is off. Each env.step() lets Swift
        # process button clicks and redraw, so the GUI stays alive while waiting
        while not self.startPressed or self.estopActive:
            self.env.step(0.05)

    def waitWhileEstopped(self):
        """Hold everything exactly where it is while the e-stop is active."""
        # Nothing moves in this loop; it just keeps Swift running until Reset is pressed
        while self.estopActive:
            self.env.step(0.05)

    def waitForNext(self, message="Press 'Next state' in Swift to continue"):
        """Use instead of input(): sliders and dropdown stay live until 'Next state' is clicked."""
        # Clear any earlier press, then wait for a new one
        self.nextPressed = False
        print(f">> {message}")
        while not self.nextPressed:
            self.env.step(0.05)