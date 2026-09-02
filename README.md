# OMY-L100 → MuJoCo FR3 Teleoperation

Physical-leader-to-simulated-follower teleoperation for Cartesian manipulation research.

## Overview

This repository implements a teleoperation pipeline from a **physical OMY-L100 leader** to a **Franka FR3 simulated in MuJoCo**.

The OMY-L100 motion is mapped into Cartesian commands and executed on the redundant 7-DoF FR3 using **velocity-level damped least-squares inverse kinematics**, with optional **null-space posture control**.

The current validated scope is:

> **Physical OMY-L100 → MuJoCo Franka FR3 free-space teleoperation**

Physical FR3 deployment is not claimed.

## Key Features

- Cartesian motion retargeting from OMY-L100 to Franka FR3
- Position-only, orientation-only, and full-pose teleoperation
- Velocity-level DLS IK for a 6D task on the redundant 7-DoF FR3
- Null-space posture regulation
- Runtime clutch anchoring for continuous relative motion
- Target velocity and acceleration conditioning
- Joint-velocity limiting
- Target / actual end-effector pose and IK diagnostic logging

## System Architecture

```text
OMY-L100 hardware
    ↓ ROS 2 /leader/joint_states
OMY forward kinematics
    ↓
Runtime clutch anchor
    ↓
Relative Cartesian command
    ↓
Position / orientation retargeting
    ↓
Target conditioning
    ↓
Velocity-level DLS IK
for a 6D task on the redundant 7-DoF FR3
    +
optional null-space posture control
    ↓
MuJoCo Franka FR3
```

## Verified Scope

The following behaviors have been tested in the current simulation-stage setup:

- Position-only teleoperation
- Orientation-only teleoperation
- Full-pose teleoperation
- 6D Cartesian tracking using FR3 position and rotational Jacobians
- Runtime clutch anchoring and command continuity
- Target velocity / acceleration conditioning
- Joint-velocity limiting
- Per-run logging of target pose, actual pose, tracking error, and IK diagnostics

The controller uses a nominal:

```text
CONTROL_HZ = 1000
```

simulation/control target.

This is a **configured target rate**, not a measured hard-real-time guarantee.

No quantitative claim is made for:

- physical Franka FR3 hardware performance
- randomized-task generalization
- production real-time operation

## Teleoperation Modes

Three control modes are supported.

| Mode | Behavior |
|---|---|
| `position_only` | OMY position is retargeted while FR3 orientation remains anchored |
| `orientation_only` | OMY orientation is retargeted while FR3 position remains anchored |
| `full_pose` | Position and orientation are retargeted together |

The current default mode is:

```python
TELEOP_MODE = "full_pose"
```

## Quick Start

### Environment

Tested with:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- MuJoCo
- Physical OMY-L100 leader

This repository also depends on local robot models and ROS workspaces that are **not included in the public repository**.

Set the repository location:

```bash
export TELEOP_ROOT=/path/to/OMY_FRANKA_TELEOP
cd "$TELEOP_ROOT"
```

### Integrated teleoperation

After the required local dependencies are configured:

```bash
OMY_PORT=/dev/ttyUSB0 ./simul
```

### OMY leader only

```bash
./teleop /dev/ttyUSB0
```

### Direct bridge execution

For development or debugging:

```bash
source /opt/ros/humble/setup.bash
source "$TELEOP_ROOT/open_manipulator_omy/install/setup.bash"

/usr/bin/python3 launch/FR3_omy_bridge.py
```

Some external model and ROS-workspace paths are currently configured locally, so a clean clone may require path configuration before the wrappers can be used.

## Main Bridge

The main teleoperation bridge is:

```text
launch/FR3_omy_bridge.py
```

Its data flow is:

```text
/leader/joint_states
    ↓
OMY MuJoCo forward kinematics
    ↓
Runtime clutch anchor
    ↓
Position / orientation mapping
    ↓
Cartesian target conditioning
    ↓
Velocity-level DLS IK
    ↓
Null-space posture control
    ↓
FR3 MuJoCo actuator command
```

The OMY model is used for forward kinematics, while the FR3 model is used for target tracking and simulated actuator dynamics.

## Repository Structure

```text
OMY_FRANKA_TELEOP/
├── launch/
│   ├── FR3_omy_bridge.py
│   ├── fr3_omy_sync.py
│   ├── FR3_EEposes.py
│   └── omy_EEposes.py
├── scripts/
│   ├── omy_sim_bridge.py
│   ├── plot_orientation_log.py
│   └── test_fr3_joint_isolation.py
├── config/
├── src/
├── tests/
├── docs/
├── simul
├── teleop
└── README.md
```

External robot models and local ROS workspaces are intentionally not shown as part of the tracked repository tree.

## Diagnostics

When logging is enabled, teleoperation runs can record:

- target end-effector position
- actual end-effector position
- rotational pose representation
- tracking error
- target dynamics
- IK / joint-velocity diagnostics

Logs can be visualized using:

```bash
/usr/bin/python3 scripts/plot_orientation_log.py
```

## Related Robot-Learning Work

Teleoperation demonstration collection and manipulation-policy experiments are maintained separately in:

### [dp-act-policy-study](https://github.com/jack2148/dp-act-policy-study)

That repository contains:

- teleoperation demonstration datasets
- LeRobot-based manipulation pipelines
- ACT experiments
- Diffusion Policy experiments
- Push-T policy analysis
- MuJoCo FR3 manipulation-policy evaluations

The two repositories represent different layers of the current research workflow:

```text
OMY_FRANKA_TELEOP
    ↓
robot interaction / teleoperation / kinematics
    ↓
demonstration collection
    ↓
dp-act-policy-study
    ↓
imitation-learning policy training and analysis
```

## Documentation

More detailed development and debugging notes are available under:

- [`docs/`](docs/)
- [Position / full-pose debugging](docs/position_mode_debugging.md)
- [Development log](docs/development.md)
- [OMY-L100 teleoperation notes](docs/omy_l100_teleop.md)

## Current Limitations

- The follower robot is currently evaluated in **MuJoCo**, not on physical FR3 hardware.
- External robot models and ROS workspaces require local configuration.
- The current Python control loop is not a hard-real-time controller.
- Physical-robot safety, collision handling, watchdogs, and hardware validation are outside the current public scope.
