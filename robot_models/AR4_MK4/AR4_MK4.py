from math import pi
import roboticstoolbox as rtb
import swift
from ir_support import CylindricalDHRobotPlot
import numpy as np

"""
AR4-MK4 6-DOF Robot Model

Using Denavit-Hartenberg convention (DH Parameters)

DH Parameters:

Joint    theta offset    d (m)       a (m)       alpha      qlim
J1       0               0.16977     0.06420     -pi/2      -170 to +170
J2       -pi/2           0           0.30500      0         -42 to +90
J3       pi              0           0            pi/2      -89 to +52
J4       0               0.22263     0           -pi/2      -165 to +165
J5       pi/2            0           0            pi/2      -105 to +105
J6       0               0.04100     0            0         -155 to +155

NOTE: all values obtained from the manufacturer specifications at the official AR4-MK4 datasheet.

Link: https://anninrobotics.com/downloads/

Excel spreadsheet containing the DH parameters and joint limits can be found within the folder.
"""

class AR4_MK4(rtb.DHRobot):
    def __init__(self, base=None):
        links = self._create_DH()

        super().__init__(
            links = links,
            name = "AR4-MK4",
            base = base,
        )

    def _create_DH(self):
        return [
            # Joint 1
            rtb.RevoluteDH(
                d=0.16977, 
                a=0.06420, 
                alpha=-pi/2, 
                offset=0,
                qlim=[-170 * pi/180, 170 * pi/180]
            ),

            # Joint 2
            rtb.RevoluteDH(
                d=0, 
                a=0.30500, 
                alpha=0, 
                offset=-pi/2,
                qlim=[-42 * pi/180, 90 * pi/180]
            ),

            # Joint 3
            rtb.RevoluteDH(
                d=0, 
                a=0, 
                alpha=pi/2, 
                offset=pi,
                qlim=[-89 * pi/180, 52 * pi/180]
            ),

            # Joint 4
            rtb.RevoluteDH(
                d=0.22263, 
                a=0, 
                alpha=-pi/2, 
                offset=0,
                qlim=[-165 * pi/180, 165 * pi/180]
            ),

            # Joint 5
            rtb.RevoluteDH(
                d=0, 
                a=0, 
                alpha=pi/2, 
                offset=pi/2,
                qlim=[-105 * pi/180, 105 * pi/180]
            ),

            # Joint 6
            rtb.RevoluteDH(
                d=0.04100, 
                a=0, 
                alpha=0, 
                offset=0,
                qlim=[-155 * pi/180, 155 * pi/180]
            )
        ]

# for testing / visualisation
"""
Select visualisation mode:
- 1: Swift simulation
- 2: Matplotlib 3D plot
- 3: Robotics toolbox teach window
"""
if __name__ == "__main__":
    mode = 1

    robot = AR4_MK4()
    robot.q = np.zeros(6)

    if mode == 1:
        env = swift.Swift()
        env.launch(realtime=True)
        robot_visual = CylindricalDHRobotPlot(
            robot,
            cylinder_radius=0.025,
            color = "blue"
        )
        robot_visual.create_cylinders()
        env.add(robot)
        print(robot)
        robot.test()
        env.hold()
    elif mode == 2:
        robot.plot(
            robot.q,
            backend="pyplot",
            block=True
        )
    elif mode == 3:
        robot.teach(
            robot.q,
            backend="pyplot",
            block=True
        )
    else:
        print("Invalid mode selected.")