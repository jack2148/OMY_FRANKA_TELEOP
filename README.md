# OMY-L100 → MuJoCo FR3 Teleoperation

Physical-leader-to-simulated-follower teleoperation for Cartesian manipulation research.

## Overview

This repository reads a physical OMY-L100 leader through ROS 2 Humble, computes its Cartesian pose with OMY forward kinematics, and retargets that motion to a Franka FR3 model simulated in MuJoCo. The controller supports position-only, orientation-only, and full-pose operation through target conditioning, velocity-level damped least-squares inverse kinematics, and optional null-space posture control.

The validated scope is MuJoCo free-space teleoperation with a physical OMY-L100 leader. Physical Franka FR3 deployment, contact manipulation, gripper control, and production real-time guarantees are outside the current claim.

## Key Contributions

- Cartesian retargeting from OMY-L100 leader motion to FR3 end-effector targets.
- Runtime clutch anchoring and held-command continuity for continuous relative motion.
- Velocity-level damped least-squares IK for a 6D task on the redundant 7-DoF FR3.
- Null-space posture regulation for the redundant FR3 configuration.
- Independent position/orientation mappings with selectable teleoperation modes.
- Target velocity/acceleration conditioning, joint-speed limiting, and runtime diagnostics.

## Results

### Verified behavior

- Position-only, orientation-only, and full-pose teleoperation modes are implemented.
- Full-pose control combines position and rotational Jacobians into a 6D task-space DLS system for the 7-DoF FR3.
- Runtime clutch anchoring and command continuity are implemented; the bridge records command and target state for MuJoCo-run inspection.
- Target and actual end-effector pose, tracking error, target dynamics, and IK/qdot diagnostics can be written to per-run CSV logs and visualized with repository plotting tools.
- The module documents a nominal 1 kHz simulation/control target. This is a configured target rate, not a measured hard-real-time guarantee.
- Quantitative hardware-FR3 performance and randomized/generalization benchmarks have not been evaluated.

These verified behaviors characterize the current OMY-L100 → MuJoCo FR3 free-space setup and should not be interpreted as physical-FR3 hardware performance.

## System Architecture

```text
OMY-L100 hardware
    ↓ ROS 2 /leader/joint_states
OMY forward kinematics
    ↓
Runtime clutch anchor and relative command
    ↓
Position / orientation retargeting
    ↓
Target conditioning
    ↓
Velocity-level DLS IK for a 6D task on the redundant 7-DoF FR3
    + optional null-space posture control
    ↓
MuJoCo Franka FR3
```

---

## Setup & Operation Guide (Korean)

실제 OMY-L100 leader의 ROS 2 joint state를 받아 MuJoCo OMY FK로 Cartesian
pose를 계산하고, MuJoCo FR3에 position/orientation target을 전달하는
teleoperation 저장소다.

현재 검증 대상은 실제 OMY-L100 → MuJoCo FR3의 단방향 free-space
teleoperation이다. 실제 FR3 제어, PushT, gripper, dataset 수집 pipeline은
아직 이 README의 실행 대상이 아니다.

Demonstration recording and policy-learning experiments are maintained separately
in [dp-act-policy-study](https://github.com/jack2148/dp-act-policy-study). 그 repo와의
inter-repo bridge에서 ZeroMQ를 사용할 수 있지만, 이 repository의 core
teleoperation architecture는 ROS 2 기반이다.

## 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10 계열
- MuJoCo Python package
- 실제 leader 사용 시 OMY-L100 및 USB serial connection

README의 shell 예제에서만 checkout 위치를 가리키기 위해 다음 변수를 사용한다.
이 변수 자체를 `simul`이나 `teleop`이 읽는 것은 아니다.

```bash
export TELEOP_ROOT=/path/to/OMY_FRANKA_TELEOP
cd "$TELEOP_ROOT"
```

## 외부 의존성

아래 항목은 이 repository의 public main에 tracked되어 있지 않다. clean clone에는
자동으로 포함되지 않으므로, 각 경로를 local machine에 준비해야 한다.

- **MuJoCo Menagerie Franka FR3 model** — `mujoco_menagerie/franka_fr3/scene.xml`.
  External dependency — path must be configured locally. 이 repository에는 해당
  model의 clone URL이나 설치 명령이 정의되어 있지 않다.
- **ROBOTIS OMY MuJoCo model** — `robotis_mujoco_menagerie/robotis_omy/scene.xml`.
  External dependency — path must be configured locally. 이 repository에는 해당
  model의 clone URL이나 설치 명령이 정의되어 있지 않다.
- **ROBOTIS OMY ROS 2 workspace** — `open_manipulator_omy/` 아래에 준비된
  `open_manipulator_bringup` 및 leader launch 파일. 기존 문서에 기록된 준비 방법은
  다음과 같다.

  ```bash
  cd "$TELEOP_ROOT"
  git clone -b feature-omy-humble \
    https://github.com/ROBOTIS-GIT/open_manipulator.git \
    open_manipulator_omy
  cd open_manipulator_omy
  source /opt/ros/humble/setup.bash
  rosdep install --from-paths . --ignore-src --rosdistro humble -r -y
  colcon build --symlink-install --event-handlers console_direct+
  ```

- **ROS 2 Humble**와 ROS package `rclpy`, `sensor_msgs`, `std_msgs`, `launch`,
  `ament_index_python` — `simul`, `teleop`, 통합 launch, bridge가 사용한다.
- **`franka_ros2_ws` / `franka_msgs`** — `simul`이
  `$HOME/franka_ros2_ws/install/setup.bash`가 있을 때 source한다. External
  dependency — path must be configured locally. 이 repository에는 clone URL이나
  설치 명령이 정의되어 있지 않다.
- **Python packages** — `mujoco`, `numpy`가 bridge와 model 점검에 필요하고,
  `matplotlib`은 CSV plotting에 필요하다. `requirements.txt`는 제공되지 않는다.

필수 model 파일과 ROS workspace가 준비되지 않은 상태에서 `simul` 또는
`launch/FR3_omy_bridge.py`를 실행하면 clean clone만으로는 재현되지 않는다.

## 사용자-facing entrypoint

- `./simul` — OMY leader bringup과 현재 FR3 MuJoCo teleoperation을 통합 실행한다.
  `OMY_PORT` 환경변수로 serial port를 지정하며 기본값은 `/dev/ttyUSB0`이다.
  `SIMULATED_FRANKA_STATE_TOPIC`, `SIMULATED_FRANKA_STATE_HZ`도 script가 읽으며
  기본값은 각각 `/fr3/simulated_robot_state`, `100`이다. 이 script는 ROS Humble setup을
  source하고, 존재하면 `franka_ros2_ws`와 `open_manipulator_omy`의 setup도
  source한다. `SIMULATED_FRANKA_STATE=1`을 설정하고
  `FRANKA_ROBOT_STATE_TOPIC`을 `SIMULATED_FRANKA_STATE_TOPIC`과 같게 만든 뒤
  `/usr/bin/python3 launch/fr3_omy_sync.py`를 호출한다.
- `./teleop [PORT]` — OMY-L100 leader bringup만 실행한다. `PORT`의 기본값은
  `/dev/ttyUSB0`이며 별도 user-configurable environment variable은 없다. ROS
  Humble과 `open_manipulator_bringup`을 source한 뒤
  `ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py`를 호출한다.
- 개발/디버그 모드에서는 leader를 별도 실행한 뒤
  `/usr/bin/python3 launch/FR3_omy_bridge.py`를 직접 실행할 수 있다.

### 빠른 시작

외부 model과 ROS workspace를 준비한 뒤 통합 teleoperation은 다음처럼 실행한다.

```bash
cd "$TELEOP_ROOT"
OMY_PORT=/dev/ttyUSB0 ./simul
```

leader bringup만 별도 실행하려면 다음을 사용한다.

```bash
cd "$TELEOP_ROOT"
./teleop /dev/ttyUSB0
```

위 wrapper의 checkout-path 제한이 적용되므로, 임의 경로의 clean clone에서는
아래 direct/debug 실행 경로를 사용하기 전에 관련 source 경로를 확인해야 한다.

`simul`이 호출하는 `launch/fr3_omy_sync.py`와 root `teleop` script에는 현재
OMY workspace/checkout 위치가 source에 고정된 부분이 있다. 따라서 임의의 clone
경로에서 두 wrapper가 모두 즉시 재현되는지는 보장되지 않는다. 이 P0에서는
code logic을 바꾸지 않고 이 제한만 명시한다.

## REVIEW_NOTE

- External dependency — path must be configured locally: MuJoCo Menagerie FR3,
  ROBOTIS OMY MuJoCo model, `open_manipulator_omy`, and `franka_ros2_ws` /
  `franka_msgs`의 local path는 이 repository가 자동으로 준비하지 않는다.
- `simul`과 `teleop`이 호출하는 integration path에는 현재 fixed local path가
  남아 있다. 이 P0에서는 code logic을 변경하지 않았으므로, arbitrary checkout
  경로에서 wrapper가 바로 동작하는 문제는 후속 작업으로 남긴다.

## 전체 구조

```text
omy_franka_teleop/
├── launch/
│   ├── FR3_omy_bridge.py       # 현재 메인 OMY → FR3 teleoperation bridge
│   ├── fr3_omy_sync.py         # leader bringup + FR3 bridge 통합 실행
│   ├── omy_sim_sync.py         # legacy OMY simulation sync 실행 파일
│   ├── FR3_EEposes.py          # FR3 초기 EE pose 확인용
│   └── omy_EEposes.py          # OMY EE pose 확인용 legacy script
├── scripts/
│   ├── omy_sim_bridge.py       # OMY joint state → OMY MuJoCo sync legacy bridge
│   ├── plot_orientation_log.py # teleoperation CSV diagnostics plot 생성
│   └── test_fr3_joint_isolation.py # FR3 단일 joint 시각 점검
├── config/
│   └── omy_sim_sync.yaml       # OMY simulation sync 설정 참고 파일
├── logs/                        # tracked reference figures; runtime CSV도 생성됨
├── src/                         # collision feedback 및 관련 source
├── tests/                       # tracked tests
├── docs/                        # tracked 개발/디버깅 문서와 이미지
├── simul                       # OMY + FR3 통합 실행 wrapper
├── teleop                      # OMY leader bringup wrapper
├── README.md
├── MUJOCO_LOG.TXT
└── Adobe Express - IMG_8690.gif
```

`robotis_mujoco_menagerie/`, `mujoco_menagerie/`, `open_manipulator_omy/`는
`.gitignore` 대상인 external/local dependency이므로 위 tracked tree에 포함하지
않는다. `results/`와 branch-specific research WIP도 public main tree의 일부로
표시하지 않는다.

### 현재 main bridge의 데이터 흐름

```text
OMY-L100 hardware
    ↓
/leader/joint_states
    ↓
launch/FR3_omy_bridge.py
    ├─ OMY MuJoCo FK
    ├─ runtime clutch anchor
    ├─ position/orientation mapping
    ├─ Cartesian target conditioning
    ├─ velocity-level DLS IK for a 6D task on the redundant 7-DoF FR3
    └─ FR3 MuJoCo actuator command
```

`FR3_omy_bridge.py`는 OMY model을 FK 용도로 사용하고, FR3 model은 target
tracking과 actuator dynamics에 사용한다. OMY ROS topic은
`/leader/joint_states`이며, trigger는 `rh_r1_joint`다.

실행을 종료하면 `logs/refactored_teleop_YYYYMMDD_HHMMSS.csv`가 생성된다.
CSV에는 `fr3_target_position_*`, `fr3_actual_position_*`와 각 pose의
회전 벡터가 100 Hz로 저장된다.

## 1. ROS workspace 준비

저장소 루트에서 ROS 2와 OMY workspace를 source한다.

```bash
cd "$TELEOP_ROOT"
source /opt/ros/humble/setup.bash
source open_manipulator_omy/install/setup.bash
```

처음 clone하는 경우 OMY package workspace를 준비한다.

```bash
cd "$TELEOP_ROOT"
git clone -b feature-omy-humble \
  https://github.com/ROBOTIS-GIT/open_manipulator.git \
  open_manipulator_omy
```

그 다음 workspace를 빌드한다.

```bash
cd "$TELEOP_ROOT/open_manipulator_omy"
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

새 terminal을 열 때마다 다음 두 환경을 source한다.

```bash
source /opt/ros/humble/setup.bash
source "$TELEOP_ROOT/open_manipulator_omy/install/setup.bash"
```

필요한 주요 Ubuntu/ROS package가 없다면 다음을 설치한다.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  libboost-all-dev \
  libserial-dev \
  ros-humble-control-toolbox \
  ros-humble-controller-interface \
  ros-humble-hardware-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-tf-transformations
```

Python dependency 확인:

```bash
/usr/bin/python3 -c "import mujoco, numpy, rclpy; print('dependencies OK')"
```

plot을 만들려면 matplotlib도 필요하다.

```bash
/usr/bin/python3 -c "import matplotlib; print('matplotlib OK')"
```

## 2. 실제 OMY-L100 leader 실행

별도 terminal에서 ROS 환경을 source한 뒤 leader bringup을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$TELEOP_ROOT/open_manipulator_omy/install/setup.bash"

ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \
  port_name:=/dev/ttyUSB0 \
  use_sim:=false \
  use_mock_hardware:=false
```

serial device가 다르면 `port_name`을 바꾼다. leader가 정상적으로 올라왔는지
다른 terminal에서 확인한다.

```bash
source /opt/ros/humble/setup.bash
source "$TELEOP_ROOT/open_manipulator_omy/install/setup.bash"

ros2 topic echo /leader/joint_states
ros2 topic hz /leader/joint_states
```

bridge가 기대하는 주요 joint 이름은 다음과 같다.

```text
joint1, joint2, joint3, joint4, joint5, joint6, rh_r1_joint
```

현재 확인된 leader controller update rate와 bridge control rate는 서로 다른
개념이다. bridge source의 nominal control rate는 `CONTROL_HZ=1000`이고,
실제 ROS message rate는 `ros2 topic hz` 결과로 별도 확인한다.

## 3. 현재 OMY → FR3 teleoperation 실행

leader를 실행한 terminal은 유지하고, 새 terminal에서 다음을 실행한다.

```bash
cd "$TELEOP_ROOT"
source /opt/ros/humble/setup.bash
source open_manipulator_omy/install/setup.bash

/usr/bin/python3 launch/FR3_omy_bridge.py
```

bridge는 시작 시 OMY/FR3 model과 EE site를 읽고 base-frame 정보를 출력한
뒤 viewer를 연다. 이후 `/leader/joint_states`를 기다린다.

정상 실행 중에는 다음 종류의 상태가 출력된다.

```text
Teleoperation mode: full_pose
Waiting for /leader/joint_states...
pos=... mm | rot=... deg | mode=full_pose | qdot=... rad/s ...
measured loop=... Hz | deadline misses=...
```

viewer를 닫거나 `Ctrl-C`로 종료한다. `ENABLE_LOGGING=True`인 경우 실행별
CSV가 `logs/refactored_teleop_YYYYMMDD_HHMMSS.csv`로 생성된다.

### 통합 실행

leader bringup과 현재 FR3 bridge를 한 번에 실행하려면 다음을 사용한다.

```bash
cd "$TELEOP_ROOT"
source /opt/ros/humble/setup.bash
source open_manipulator_omy/install/setup.bash

/usr/bin/python3 launch/fr3_omy_sync.py port_name:=/dev/ttyUSB0
```

이 파일은 현재 ROS package 안에 설치된 launch entry가 아니므로 `ros2 launch`
대신 `/usr/bin/python3`로 실행한다.

## 4. Teleoperation mode 변경

mode는 `launch/FR3_omy_bridge.py` 상단의 `TELEOP_MODE`를 수정한다.

```python
TELEOP_MODE = "full_pose"
```

지원 값은 다음 세 가지다.

| Mode | 동작 |
|---|---|
| `position_only` | OMY position만 command에 반영하고 FR3 rotation은 clutch anchor에 hold |
| `orientation_only` | OMY orientation만 command에 반영하고 FR3 position은 clutch anchor에 hold |
| `full_pose` | position과 orientation을 함께 retarget |

현재 source의 기본 mode는 `full_pose`다. mode를 바꾼 뒤에는 syntax를 먼저
확인한다.

```bash
python3 -m py_compile launch/FR3_omy_bridge.py
```

주요 조정값은 같은 파일 상단에 있다.

```python
POSITION_SCALE = 0.4
ORIENTATION_SCALE = 0.3
MAX_TARGET_LINEAR_SPEED = 0.09
MAX_TARGET_LINEAR_ACCEL = 2.0
MAX_TARGET_ANGULAR_SPEED = 2.0
MAX_TARGET_ANGULAR_ACCEL = 4.0
IK_DAMPING = 0.05
MAX_JOINT_SPEED = 0.80
ENABLE_NULLSPACE_POSTURE = True
```

axis mapping은 자동으로 바뀌지 않는다. position과 orientation mapping은
`POSITION_AXIS_MAP`, `ORIENTATION_AXIS_MAP`에 독립적으로 정의되어 있다.

## 5. CSV 확인과 plot 생성

가장 최근 CSV를 자동 선택하려면:

```bash
cd "$TELEOP_ROOT"
/usr/bin/python3 scripts/plot_orientation_log.py
```

특정 CSV를 지정하려면:

```bash
/usr/bin/python3 scripts/plot_orientation_log.py \
  /path/to/generated_teleop_log.csv
```

plot은 CSV 옆에 생성되며, position/orientation signed diagnostics와 summary
plot을 포함할 수 있다. CSV header가 오래된 실행 파일이면 일부 diagnostic
figure가 생략될 수 있다. 이 경우 CSV header를 먼저 확인한다.

```bash
head -n 2 /path/to/generated_teleop_log.csv
```

## 6. FR3 model 단독 점검

teleoperation을 실행하지 않고 FR3 관절 하나의 방향과 joint axis를 확인한다.

```bash
cd "$TELEOP_ROOT"
/usr/bin/python3 scripts/test_fr3_joint_isolation.py \
  --joint 6 \
  --amplitude 0.25 \
  --frequency 0.20
```

`--joint`는 1부터 7까지 사용할 수 있다. 이 스크립트는 선택한 관절만
sinusoidal command로 움직이고 나머지 관절은 초기 command에 hold한다.
실제 teleoperation controller의 IK나 actuator 설정을 변경하지 않는다.

FR3 초기 EE pose만 출력하려면:

```bash
/usr/bin/python3 launch/FR3_EEposes.py
```

## 7. 종료와 기본 점검

실행 전 syntax와 diff whitespace를 확인한다.

```bash
python3 -m py_compile launch/FR3_omy_bridge.py
python3 -m py_compile scripts/plot_orientation_log.py
git diff --check
```

leader topic이 보이지 않으면 다음을 순서대로 확인한다.

```bash
ros2 topic list | grep leader
ros2 topic echo /leader/joint_states
ros2 topic hz /leader/joint_states
```

USB serial timeout이 반복되면 device와 permission을 확인한다.

```bash
ls -l /dev/ttyUSB0
groups
```

OMY controller의 USB latency를 사용하는 환경에서는 다음 설정이 도움될 수
있지만, hardware 환경에 맞는지 확인한 뒤 적용한다.

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

## 8. 관련 문서

- [Position/full-pose debugging](docs/position_mode_debugging.md)
- [Development log](docs/development.md)
- [Archived full debugging log](docs/archive/debugging_full_log_20260724_20260730.md)
- [OMY-L100 teleoperation notes](docs/omy_l100_teleop.md)

## 주의사항

현재 bridge는 MuJoCo 검증용 Python loop다. `CONTROL_HZ=1000`은 nominal
simulation/control setting이며 Python process가 hard real-time이라는 뜻이
아니다. 실제 FR3에 연결하기 전에는 별도의 robot interface, safety limit,
watchdog, collision policy 및 hardware validation이 필요하다.
