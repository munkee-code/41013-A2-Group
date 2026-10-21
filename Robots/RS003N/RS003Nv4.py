import os
import time
import numpy as np
import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from ir_support.robots.UTSMeshRobot import UTSMeshRobot


class RS003N(UTSMeshRobot):
    """Kawasaki RS003N: standard DH model with 3D STL meshes."""

    # The DH base frame is at shoulder height, so lift the robot by this much
    # to put the bottom of the base mesh on the ground (z = 0).
    SHOULDER_HEIGHT = 0.25

    def __init__(self, base=None):
        if base is None:
            base = SE3(0, 0, self.SHOULDER_HEIGHT)

        # DH links
        links = self._create_DH()

        # Names of the mesh files (base + 6 moving links)
        link3D_names = dict(link0="Base",
                            link1="J1",
                            link2="J2",
                            link3="J3",
                            link4="J4",
                            link5="J5",
                            link6="J6")

        # Joint config at which the meshes were exported (CAD zero pose)
        # CAD arm is upright, which is the DH zero pose.
        qtest = [0, 0, 0, 0, 0, 0]

        # Where the DH base frame sits in the CAD (STL) coordinates, measured from
        # the mesh bounding boxes:
        #   x, y = vertical axis of joint 1, z = shoulder (joint 2) height.
        # Rz(pi) is needed because the CAD arm sits at +0.12 m in y from the joint 1
        # axis, while the DH model puts it at -0.12 m.
        CAD_X, CAD_Y, CAD_Z = 0.1071, 0.1723, self.SHOULDER_HEIGHT
        dh_in_cad = spb.transl(CAD_X, CAD_Y, CAD_Z) @ spb.trotz(np.pi)
        cad_in_dh = np.linalg.inv(dh_in_cad)

        # Pose of each mesh's origin in the DH world (all STLs share the CAD origin).
        qtest_transforms = [cad_in_dh for _ in range(7)]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(
            links=links,
            mesh_stem="RS003N",
            mesh_dir=current_path,
            name="RS003N",
            home_q=qtest,
            base=base,
            link3d_names=link3D_names,
            qtest_transforms=qtest_transforms,
        )

    # ------------------------------------------------------------------
    def _create_DH(self):
        """Standard DH model (equivalent to the original MDH parameters)."""
        a = [0.120, 0.250, 0, 0, 0, 0]
        d = [0, 0, 0, 0.250, 0, 0.080]
        alpha = [-np.pi / 2, np.pi, -np.pi / 2, np.pi / 2, -np.pi / 2, 0]
        offset = [-np.pi / 2, -np.pi / 2, -np.pi / 2, 0, 0, -np.pi / 2]
        qlim = [np.deg2rad([-160, 160]),
                np.deg2rad([-60, 150]),
                np.deg2rad([-150, 120]),
                np.deg2rad([-360, 360]),
                np.deg2rad([-135, 135]),
                np.deg2rad([-360, 360])]

        links = []
        for i in range(6):
            links.append(rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i],
                                        offset=offset[i], qlim=qlim[i]))
        return links