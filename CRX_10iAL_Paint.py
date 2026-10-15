## CRX-10iA/L Paint Robot

#(abbreviated in future to CRXPaint)

import os
from math import pi
import roboticstoolbox as rtb
from ir_support import CylindricalDHRobotPlot
import spatialmath.base as spb

class CRXPaint(rtb.DHRobot):
    def __init__(self, base=None):
        links = self._create_DH()
        super().__init__(
            links=links,
            mesh_stem="CRXMESH",
            mesh_dir=os.path.abspath(os.path.dirname(__file__)),
            name="CRXPaint",
            home_q=[0, 0, 0, 0, 0, 0],
            link3d_names=dict(link0 = 'CRXBase',
                              link1 = 'CRXJ1',
                              link2 = 'CRXJ2',
                              link3 = 'CRXJ3',
                              link4 = 'CRXJ4',
                              link5 = 'CRXJ5',
                              link6 = 'CRXJ6'),
            base=base,
            #qtest_transforms = [spb.transl(0,0,0),
                                #spb.transl(0,0,0.4) @ spb.trotz(0),
                                #spb.transl(0,-0.1,0.54) @ spb.rpy2tr(0,0,pi/2, order = 'xyz'),
                                #spb.transl(0,0.24,1.15) @ spb.rpy2tr(0,0,pi/2, order = 'xyz'),
                                #spb.transl(0.1,-0.22,1.54) @ spb.rpy2tr(0,-pi/2,0, order = 'xyz'),
                                #spb.transl(0.15,-0.12,1.54) @ spb.rpy2tr(0,0,pi/2, order= 'xyz'),
                                #spb.transl(0.15,-0.3,1.54) @ spb.rpy2tr(0,0,pi/2, order= 'xyz')]
        )

    def _create_DH(self):
        return [
            #Joint 1
            rtb.RevoluteDH(d=0.1330, a=0, alpha=pi / 2),
            #Joint 2
            rtb.RevoluteDH(d=0, a=-0.3100, alpha=0),
            #Joint 3
            rtb.RevoluteDH(d=0, a=-0.1800, alpha=0),
            #Joint 4
            rtb.RevoluteDH(d=0.0950, a=0, alpha=pi / 2),
            #Joint 5
            rtb.RevoluteDH(d=0.0940, a=0, alpha=-pi / 2),
            #Joint 6
            rtb.RevoluteDH(d=0.0890, a=0, alpha=0)
        ]