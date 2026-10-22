import numpy as np
import swift
from spatialmath import SE3
 
 
class RobotGui:
    def __init__(self, env, robots, startLabel="Start RS003N"):
        """
        env        : swift.Swift environment
        robots     : dict {display name: robot}; the first one is selected at start
        startLabel : text on the Start button (read by waitForStart())
        """
        self.env = env
        self.robots = dict(robots)
        self.names = list(self.robots)
        self.selected = self.names[0]
        self.startLabel = startLabel
 
        # Shared state - read/written by motion code (e.g. fastening.py)
        self.startPressed = False
        self.estopActive = False
        self.motionRunning = False   # True while code is driving a robot - slider moves are blocked
        self.movedByHand = False          # set when a slider moves a robot by hand
        self.nextPressed = False
 
        self._moveCallbacks = {}      # id(robot) -> fn(q), e.g. so a held part follows a slider move
        self.jointSliders = []
 
        # State display - added first so it sits at the top of the panel
        self.stateText = "State: -"
        self.stateLabel = swift.Label(self.stateText)
        self.env.add(self.stateLabel)
 
        # E-stop display - right under the state
        self.estopLabel = swift.Label(self._estopText())
        self.env.add(self.estopLabel)
 
        self._addSelector()
        self._addJointSliders()
        self._addButtons()
        print(f">> GUI ready - moving {self.selected}")
 
    @property
    def robot(self):
        """The robot currently selected in the dropdown."""
        return self.robots[self.selected]
 
    def setMoveCallback(self, robot, fn):
        """Call fn(q) whenever `robot` is moved by hand with the sliders."""
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
        return "E-STOP: ACTIVE - press 'Reset E-stop'" if self.estopActive else "E-STOP: off"
 
    def _updateEstopLabel(self):
        self._setText(self.estopLabel, self._estopText())
 
    # ------------------------------------------------------------------
    # Robot dropdown
    # ------------------------------------------------------------------
    def _resolveName(self, value):
        if value in self.robots:
            return value
        try:
            return self.names[int(value)]
        except (ValueError, IndexError, TypeError):
            return None
 
    def _addSelector(self):
        def cb(value):
            name = self._resolveName(value)
            if name is None or name == self.selected:
                return
            self.selected = name
            self._applyRobotToSliders()
            lims = ", ".join(f"J{j + 1} {lo:.0f}..{hi:.0f}"
                             for j, (lo, hi) in ((j, self._limitsDeg(self.robot, j))
                                                 for j in range(self.robot.n)))
            print(f">> GUI now moving {name} (deg: {lims})")
 
        self.env.add(swift.Select(cb, desc="Robot to move", options=self.names, value=0))
 
    # ------------------------------------------------------------------
    # Joint sliders (one shared set, bound to the selected robot)
    # ------------------------------------------------------------------
    def moveAllowed(self):
        return (not self.motionRunning) or self.estopActive
 
    def _makeJointSlider(self, j):
        def cb(value):
            if not self.moveAllowed():
                return
            robot = self.robot
            if j >= robot.n:                                   # robot has fewer joints
                return
            deg = float(value)
            if abs(np.degrees(robot.q[j]) - deg) < 0.05:       # echo of syncSliders()
                return
 
            lo, hi = robot.qlim[:, j]
            rad = float(np.clip(np.radians(deg), lo, hi))      # respect this robot's limits
            q = np.array(robot.q, dtype=float)
            q[j] = rad
            robot.q = q
 
            fn = self._moveCallbacks.get(id(robot))
            if fn is not None:
                fn(q)
            self.movedByHand = True
 
            if abs(np.degrees(rad) - deg) > 0.05:              # snap slider back inside limits
                self.jointSliders[j].value = round(float(np.degrees(rad)), 1)
        return cb
 
    @staticmethod
    def _limitsDeg(robot, j):
        """Joint j limits in degrees, read from the robot's own qlim (its DH links)."""
        lo, hi = np.degrees(robot.qlim[:, j])
        lo = lo if np.isfinite(lo) else -360.0
        hi = hi if np.isfinite(hi) else 360.0
        return round(float(lo), 1), round(float(hi), 1)
 
    @staticmethod
    def _setText(s, text):
        """Change the text of a slider or label (newer Swift uses .label, older uses .desc)."""
        try:
            if hasattr(type(s), "label"):
                s.label = text
            else:
                s.desc = text
        except AttributeError:
            pass                                               # this Swift can't change element text
 
    def _addJointSliders(self):
        nJoints = max(r.n for r in self.robots.values())
        robot = self.robot
        for j in range(nJoints):
            # Start with the selected robot's own limits; _applyRobotToSliders() swaps them on select
            lo, hi = self._limitsDeg(robot, j) if j < robot.n else (-360.0, 360.0)
            value = np.degrees(robot.q[j]) if j < robot.n else 0.0
            s = swift.Slider(self._makeJointSlider(j),
                             min=lo, max=hi,
                             step=0.5, value=round(float(value), 1),
                             desc=f"{self.selected} J{j + 1}", unit="&#176;")
            self.env.add(s)
            self.jointSliders.append(s)
 
    def _applyRobotToSliders(self):
        """Give every slider the selected robot's joint limits, label and current angle."""
        robot = self.robot
        for j, s in enumerate(self.jointSliders):
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
            deg = round(float(np.degrees(robot.q[j])), 1)
            if force or abs(float(s.value) - deg) > 0.25:
                s.value = deg
 
    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------
    def _addButtons(self):
        def onStart(_=None):
            self.startPressed = True
 
        def onEstop(_=None):
            if not self.estopActive:
                self.estopActive = True
                self._updateEstopLabel()
                print("\n*** E-STOP PRESSED - motion halted. Press 'Reset E-stop' to resume. ***")
 
        def onReset(_=None):
            if self.estopActive:
                self.estopActive = False
                self._updateEstopLabel()
                print("E-stop reset - resuming")
 
        def onNext(_=None):
            self.nextPressed = True
 
        self.env.add(swift.Button(onStart, desc=self.startLabel))
        self.env.add(swift.Button(onEstop, desc="E-STOP"))
        self.env.add(swift.Button(onReset, desc="Reset E-stop"))
        self.env.add(swift.Button(onNext, desc="Next state"))
 
    # ------------------------------------------------------------------
    # Waiting helpers (all keep Swift responsive)
    # ------------------------------------------------------------------
    def waitForStart(self):
        self.startPressed = False
        print(f">> Press '{self.startLabel}' in Swift to begin...")
        while not self.startPressed or self.estopActive:
            self.env.step(0.05)
 
    def waitWhileEstopped(self):
        """Hold everything exactly where it is while the e-stop is active."""
        while self.estopActive:
            self.env.step(0.05)

    def waitForNext(self, message="Press 'Next state' in Swift to continue"):
        """Use instead of input(): sliders and dropdown stay live until 'Next state' is clicked."""
        self.nextPressed = False
        print(f">> {message}")
        while not self.nextPressed:
            self.env.step(0.05)
