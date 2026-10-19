# A2 Group Project | README 
## Robots
These implementations use the Robotics Toolbox for Python to model the robots' kinematics using standard Denavit-Hartenberg (DH) parameters. The robots are visualised and tested in the Swift simulator.

### AR4-MK5
#### Overview

<img src="AR4_MK5/ref_image.jfif" alt="AR4-MK5 Robot" width="250" align="right">

The **AR4-MK5** is a six-degree-of-freedom (6-DOF) robotic arm developed by [Annin Robotics](https://anninrobotics.com/).

This implementation uses the **Robotics Toolbox for Python** to model the robot's kinematics using standard Denavit–Hartenberg (DH) parameters.

The robot is visualised using 3D STL meshes in the **Swift simulator**.

<br clear="all">

#### Specifications
| Property | Description |
|:---|:---|
| Manufacturer | Annin Robotics |
| Model | AR4-MK5 |
| Degrees of Freedom | 6 |
| Joint Type | Revolute |
| Kinematic Model | Standard DH (URDF can be used) |
| Simulation | Swift |
| 3D Models | STL |

#### DH Parameters
The following standard DH parameters are used in the robot model. Distances are in meters and angles are in degrees.

| Joint | θ Offset | d (m) | a (m) | α |
|---|---|---|---|---|
| J1 | 0° | 0.16977 | 0.06420 | -90° |
| J2 | -90° | 0 | 0.30500 | 0° |
| J3 | 180° | 0 | 0 | 90° |
| J4 | 0° | 0.22263 | 0 | -90° |
| J5 | 0° | 0 | 0 | 90° |
| J6 | 0° | 0.03630 | 0 | 0° |

#### Joint Limits
| Joint | Minimum | Maximum |
|---|---|---|
| J1 | -170° | 170° |
| J2 | -42° | 90° |
| J3 | -89° | 52° |
| J4 | -165° | 165° |
| J5 | -105° | 105° |
| J6 | -155° | 155° |

#### Project Structure

```text
AR4_MK5/
├── AR4_MK5.py
├── README.md
├── link0.stl
├── link1.stl
├── link2.stl
├── link3.stl
├── link4.stl
├── link5.stl
└── link6.stl
```

#### Running the Simulation

From the repository root, run:

```bash
python AR4_MK5/AR4_MK5.py
```

The simulation launches in Swift and tests the movement of all six joints individually.

#### References

- [Annin Robotics](https://anninrobotics.com/)
- [AR4-MK5 Specifications](https://anninrobotics.com/downloads)
- [AR4-MK5 STL Models](https://github.com/Annin-Robotics/ar4_ros_driver/tree/main/annin_ar4_description/meshes/ar4_mk5)
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
- [Swift Simulator](https://github.com/jhavl/swift)

<!-- Reference image of AR4-MK5-->
![AR4 MK5 image](AR4_MK5/ref_image.jfif)