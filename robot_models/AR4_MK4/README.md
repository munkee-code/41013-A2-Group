# AR4-MK4 Robot Model

This folder contains a six-degree-of-freedom Denavit-Hartenberg (DH) model of the Annin Robotics AR4-MK4 robot. The model is implemented as a `roboticstoolbox` `DHRobot` and can be used for forward kinematics, joint-limit checks, and basic visualization.

## Files

- `AR4_MK4.py` - Python robot model and visualization entry point.
- `AR4-MK3 kinematic model (standard DH).xlsm` - spreadsheet containing the DH parameter and joint-limit data used by the model.

## Running / Testing the model

From the repository root, run:

```powershell
python robot_models\AR4_MK4\AR4_MK4.py
```

The `mode` value in the `__main__` block selects the visualization:

| Mode | Description |
| ---: | --- |
| `1` | Swift real-time simulation with cylindrical links. |
| `2` | Robotics Toolbox Matplotlib 3D plot. |
| `3` | Robotics Toolbox teach window for interactively changing joint positions. |

The default mode is `1`. The model starts with all six joint angles set to zero. The script also calls the Robotics Toolbox robot test method before holding the Swift simulation open.

## DH parameters and joint limits

Distances are in metres and angles are in radians inside the Python model. The limits below are shown in degrees for readability.

| Joint | Offset | $d$ (m) | $a$ (m) | $\alpha$ | Joint limits (degrees) |
| --- | ---: | ---: | ---: | ---: | ---: |
| J1 | 0 | 0.16977 | 0.06420 | $-\pi/2$ | -170 to 170 |
| J2 | $-\pi/2$ | 0 | 0.30500 | 0 | -42 to 90 |
| J3 | $\pi$ | 0 | 0 | $\pi/2$ | -89 to 52 |
| J4 | 0 | 0.22263 | 0 | $-\pi/2$ | -165 to 165 |
| J5 | $\pi/2$ | 0 | 0 | $\pi/2$ | -105 to 105 |
| J6 | 0 | 0.04100 | 0 | 0 | -155 to 155 |

The DH values and limits are taken from the manufacturer specifications referenced in the Python source. For the official datasheets, see the [Annin Robotics downloads page](https://anninrobotics.com/downloads/).

## Using the class in another script

```python
import numpy as np

from robot_models.AR4_MK4.AR4_MK4 import AR4_MK4

robot = AR4_MK4()
robot.q = np.zeros(6)
print(robot.fkine(robot.q))
```

The `AR4_MK4` constructor accepts an optional `base` transform, which is passed to the Robotics Toolbox `DHRobot` base property.
