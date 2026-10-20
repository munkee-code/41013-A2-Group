## @file AR4_MK5.py
## @brief AR4-MK5 Robot defined by standard DH parameters with 3D model. NOTE: structure of this 
##        file is based on UR3.py provided in ir_support libraries.
## @author Samuel Vella
import swift
import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3
from spatialgeometry import Mesh
from ir_support.robots.UTSMeshRobot import UTSMeshRobot
import time
import os
from math import pi

# fix for swift incompatibility with Mesh class
if not hasattr(Mesh, "_update_pyb"):
    Mesh._update_pyb = lambda self: None

class AR4_MK5(UTSMeshRobot):
    manufacturer_url = "https://anninrobotics.com/product-page/ar4-mk5-robot-combo-kit/?srsltid=AU7gw4XWyoF9mWPhT7ZzsrwE_njY0NjbGfbI_gKxAWrZkEmwAb9Mhief"

    # dowloads for specifications sheet: https://anninrobotics.com/downloads?srsltid=AU7gw4V2RoZVggSfibZHk0Cru2HA9CryeRJT9tTb1HQv4JYjgruDK-0g
    # 3d models obtained from: https://github.com/Annin-Robotics/ar4_ros_driver/tree/main/annin_ar4_description/meshes/ar4_mk5

    #-----------------------------------------------------------------------------------------------
    ## @brief Initialize the AR4-MK5 robot with its DH parameters and 3D model
    ## @param base The base transformation of the robot (optional)
    ## @return An instance of the AR4-MK5 robot
    def __init__(self, base=None):
        # DH Links
        links = self._create_DH()

        # names of the robot link files in the directory
        link_names = dict(
            link0 = "link0",        # base
            link1 = "link1",        # link 1
            link2 = "link2",        # link 2
            link3 = "link3",        # link 3
            link4 = "link4",        # link 4
            link5 = "link5",        # link 5
            link6 = "link6"         # end effector
        )

        qtest = [0, 0, 0, 0, 0, 0]
        qtest_transforms = [
            spb.transl(0, 0, 0),        # link 0 (base)
            spb.transl(0, 0, 0),        # link 1
            spb.transl(0, 0, 0),        # link 2
            spb.transl(0, 0, 0),        # link 3
            spb.transl(0, 0, 0),        # link 4
            spb.transl(0, 0, 0),        # link 5
            spb.transl(0, 0, 0)         # link 6 (end effector)
        ]

        current_path = os.path.abspath(os.path.dirname(__file__))
        print("Mesh directory:", current_path)
        for i in range(7):
            filepath = os.path.join(current_path, f"link{i}.stl")
            print(f"Link {i}: {filepath} -> {os.path.isfile(filepath)}")

        super().__init__(
            links=links,
            mesh_stem="AR4_MK5",
            mesh_dir=current_path,
            name="AR4_MK5",
            home_q=qtest,
            base=base,
            link3d_names=link_names,
            qtest_transforms=qtest_transforms,
        )

    #-----------------------------------------------------------------------------------------------
    ## @brief Create the DH parameters for the AR4-MK5 robot
    ## @return List of RevoluteDH links based on the standard DH parameters
    def _create_DH(self):
        # a: link lengths along the x-axis of the previous frame
        a = [
            0.06420, 
            0.30500, 
            0, 
            0, 
            0, 
            0
        ]

        # d: link offsets along the z-axis of the previous frame
        d = [
            0.16977, 
            0, 
            0, 
            0.22263, 
            0, 
            0.03630
        ]

        # alpha: link twists around the x-axis of the previous frame
        alpha = [
            -pi/2, 
            0, 
            pi/2, 
            -pi/2, 
            pi/2, 
            0
        ]

        # offset: joint angle offsets for each revolute joint
        offset = [
            0,
            -pi/2,
            pi,
            0,
            0,
            0
        ]

        # qlim: joint limits for each revolute joint
        qlim = [
            [-170 * pi/180, 170 * pi/180],
            [-42 * pi/180, 90 * pi/180],
            [-89 * pi/180, 52 * pi/180],
            [-165 * pi/180, 165 * pi/180],
            [-105 * pi/180, 105 * pi/180],
            [-155 * pi/180, 155 * pi/180]
        ]

        # create the list of revolute DH links
        links = []

        # iterate over each joint and create the corresponding revolute DH link
        for i in range(6):
            link = rtb.RevoluteDH(a=a[i], d=d[i], alpha=alpha[i], qlim=qlim[i], offset=offset[i])
            links.append(link)

        return links

    #-----------------------------------------------------------------------------------------------
    ## @brief Test the AR4-MK5 robot by moving it from its initial configuration to a goal configuration
    ## @details The robot is added to a Swift environment and a joint trajectory is executed
    ## @return None
    def test(self):
        # initialise swift environment
        env = swift.Swift()
        env.launch(realTime=True)

        # set the initial joint configuration and base pose
        self.q = self._qtest
        self.base = SE3(0.5, 0.5, 0)

        # add the robot to the Swift environment
        self.add_to_env(env)

        # define the goal joint configuration and generate a joint trajectory
        q_home = self.q.copy()
        
        for i in range(self.n):
            q_goal = q_home.copy()
            q_goal[i] += pi/6

            qtraj = rtb.jtraj(self.q, q_goal, 50).q

            # execute the joint trajectory in the Swift environment
            #fig = self.plot(self.q)
            for q in qtraj:
                self.q = q
                env.step(0.02)
                #fig.step(0.01)

            time.sleep(3)
            input("Press Enter to continue...")

        # execute the joint trajectory in the Swift environment
        #fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            #fig.step(0.01)

        time.sleep(3)
        input("Press Enter to continue...")
        #env.hold()

#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    robot = AR4_MK5()
    robot.test()

