# 41013 | A2 Group Project 
## Overview
This repository contains the development and simulation of an automated vehicle assembly line for the 41013 Industrial Robotics group assignment.

The project uses multiple industrial robotic arms to simulate automotive manufacturing operations, including picking & place, fastening, and spray painting.

The simulation is developed in Python using `ir_support` package.

## Repository Structure
```Text
41013-A2-Group/
│
├── Models/
│   ├── Warehouse/
│   ├── Floor/
│   ├── Conveyor/
│   └── Chassis/
│
├── Robots/
│   ├── AR4_MK5/
│   └── ...
│
├── Test Scripts/
│
├── main.py
├── README.md
└── .gitignore
```

| Directory / File | Description |
|---|---|
| `main.py` | Main entry point for the complete assembly-line simulation |
| `Models/` | Environmental assets and vehicle components used in Swift |
| `Robots/` | Robot implementations, kinematic models, and robot-specific 3D meshes |
| `Test Scripts/` | Standalone scripts for testing individual features and functionality |
| `README.md` | Project documentation |
| `.gitignore` | Specifies files and directories excluded from Git |

**Note:** Robot-specific STL files should be stored alongside their respective robot implementations in `Robots/`, rather than in the general `Models/` directory.

## Main Simulation
### Running the Simulation
To launch the main simulation, run the following command from the repository root:

```text
python main.py
```

Ensure the required Python dependencies are installed before running the simulation (refer to Canvas).

### Design
The main simulation uses a state machine to organise the different stages of the assembly process.

Each state is responsible for a particular operation, allowing the simulation to progress through environment setup, robot initialisation and automated assembly tasks.

#### STATE | 0 := Configuration & Setup
Initialises the simulation environment and loads the required assets into swift.

Assets include:
- Warehouse environment
- Factory floor
- Conveyor system
- Vehicle chassis
- Robot arms
    - AR4-MK5
    - Kawasaki ...
    - FANUC ...
- Vehicle components, fasteners, and other assembly items

Asset paths and mesh objects are managed using Python dictionaries to simplify loading and accessing objects throughout the simulation.

#### STATE | 1 := Pick & Place
TODO

#### STATE | 2 := Fastening
TODO

#### STATE | 3 := Spray Painting
TODO

## Models
The `Models/` directory contains 3D assets used to construct the Swift simulation environment.

These assets are seperate from the robot implementations and generally represent static environmental geometry or objects manipulated during the assembly process.

Assets may use formats such as `.stl` and / or `.dae`, depending on their requirements and compatability with Swift.

### Car & Vehicle Components
Contains the vehicle components used during assembly.

Examples include:
- Vehicle chassis
- Wheel(s)
- Fasteners

Components that need to move independently should maintain as separate mesh objects.

### Environment
Contains the models used to construct the 'warehouse' environment.

Current assets include:

| Directory | Description |
|---|---|
| `Warehouse/` | Main warehouse structure and surrounding factory environment |
| `Floor/` | Factory flooring |
| `Conveyor/` | Conveyor equipment used to transport components |
| `Chassis/` | Vehicle chassis used in the assembly simulation |

### Safety Equipment
TODO

## Robots
The `Robots/` directory contains the implementations of the industrial robotic arms used in the simulation.

Each robot should have its own subdirectory containing:
- Python robot implementation
- Kinematic parameters and joint limits 
- Robot-specific 3D meshes 
- Supporting files and documentation

Custom robot implementations use the Robotic Toolbox for Python to model robot kinematics using Denavit-Hartenberg parameters. The robots are visualised and tested in the Swift simulator.

### AR4-MK5
#### Overview

<img src="Robots\AR4_MK5\ref_image.jfif" alt="AR4-MK5 Robot" width="250" align="right">

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
![AR4 MK5 image](Robots\AR4_MK5\ref_image.jfif)

## Test Scripts
The `Test Scripts/` directory contains standalone Python scripts used to develop, test, and validate individual components of the project. 

These scripts are separate from the main simulation and may be used to experiment with functionality before integrating it into main.py.