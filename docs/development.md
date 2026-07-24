# FR3–OMY Cartesian Teleoperation 개발 기록

최초 작성: 2026-07-23<br>
최종 수정: 2026-07-24

## 1. 목표 및 현재 결과

이 시스템의 목표는 실제 OMY-L100 리더암의 ROS joint state를 MuJoCo OMY 모델에 적용하고, OMY EE의 Cartesian position 변화를 FR3 MuJoCo의 target position으로 전달하는 것이다.

현재 다음 기능을 구현하였다.

- 실제 OMY-L100의 `/leader/joint_states` 수신
- OMY MuJoCo 모델의 joint qpos 실시간 동기화
- `mujoco.mj_forward()` 기반 OMY EE pose 계산
- Trigger ON 시 OMY·FR3 runtime initial pose 캡처
- OMY EE position displacement의 FR3 target position 변환
- translational DLS IK를 이용한 FR3 MuJoCo EE target 추종
- FR3 position actuator command 적용
- 전후·좌우·상하 position teleoperation 동작 확인

현재 실제 OMY-L100의 전후·좌우·상하 움직임을 FR3 MuJoCo EE position으로 전달하는 position-only Cartesian teleoperation MVP를 구현하였다.

현재 IK objective에는 EE position error만 포함하며, OMY orientation retargeting과 rotational IK는 구현하지 않았다.

## 2. 시스템 구성과 EE 기준

전체 파이프라인:

```text
실제 OMY-L100
→ /leader/joint_states
→ OMY MuJoCo qpos
→ mujoco.mj_forward()
→ omy_ee_site pose
→ runtime-relative displacement
→ operator-frame position mapping
→ FR3 target position
→ translational DLS IK
→ FR3 position actuator
```

| Robot | MuJoCo model | EE site |
|---|---|---|
| OMY-L100 | OMY MJCF | `omy_ee_site` |
| FR3 | MuJoCo Menagerie FR3 | `attachment_site` |

OMY EE site:

```xml
<site
    name="omy_ee_site"
    pos="0 -0.109 0"
    size="0.01"
    rgba="0 1 0 1"
/>
```

FR3 EE site:

```xml
<site
    name="attachment_site"
    pos="0 0 0.107"
/>
```

MuJoCo site pose 조회:

```python
position = data.site_xpos[site_id].copy()
rotation = data.site_xmat[site_id].reshape(3, 3).copy()
```

현재 IK objective에는 `position`만 사용한다. `rotation`은 pose 확인을 위해 조회할 수 있지만 현재 DLS 오차항에는 포함하지 않는다.

## 3. 실제 OMY joint state 기반 실시간 FK

```text
OMY-L100 실제 리더암
    ↓
/leader/joint_states
    ↓
Joint1~Joint6 이름 기준 추출
    ↓
OMY MuJoCo qpos 갱신
    ↓
mujoco.mj_forward()
    ↓
omy_ee_site current pose 계산
```

실시간으로 계산하는 값:

- `p_omy_current`
- `R_omy_current`

![Real-time OMY-L100 Synchronization](real-time-omy-l100-synchronization.gif)

실제 OMY joint state가 OMY MuJoCo 모델에 반영되고, 실제 arm motion에 따라 MuJoCo joint state와 EE pose가 갱신되는 것을 확인하였다.

위 영상은 최종 FR3 teleoperation 결과가 아니라 실제 OMY joint state 수신, MuJoCo synchronization, FK 및 EE pose 계산을 검증한 결과이다.

### Joint6 회전 해석

OMY Joint6의 local rotation axis는 Y축이다.

```xml
<default class="Joint6">
    <joint axis="0 1 0"/>
</default>
```

`omy_ee_site`가 Joint6 회전축 위에 위치하면 Joint6 회전 시 EE rotation은 변하지만 EE position 변화는 작을 수 있다. 이는 FK 오류가 아니라 EE site와 회전축 사이의 기하학적 관계 때문이다. 회전축에서 벗어난 별도의 debug site를 사용하면 위치 변화를 시각적으로 확인할 수 있다.

## 4. Runtime-relative Cartesian position mapping

### XML home pose

XML `home` keyframe은 모델 검증, EE site 확인, 시뮬레이션 초기화 및 IK 초기 자세 설정에 사용한다. Teleoperation 증분 계산의 기준으로는 사용하지 않는다.

### Runtime initial pose

Trigger ON 시 실제 OMY joint state를 MuJoCo에 적용하고 FK를 수행한 뒤, 해당 시점의 OMY와 FR3 EE position을 runtime initial position으로 저장한다. Rotation은 현재 position-only 구현에서 저장하거나 IK objective에 사용하지 않는다.

```python
p_omy_initial = p_omy_current.copy()
p_fr3_initial = p_fr3_current.copy()
```

현재 position displacement:

```python
delta_position_omy = (
    p_omy_current
    - p_omy_initial
)
```

operator-frame position mapping:

```python
AXIS_MAP = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])
```

```text
FR3 X =  OMY Y
FR3 Y = -OMY X
FR3 Z =  OMY Z
```

target 계산은 실제 구현과 동일하게 `+` 부호를 사용한다.

```python
delta_position_fr3 = (
    POSITION_SCALE
    * (
        AXIS_MAP
        @ delta_position_omy
    )
)

fr3_target_position = (
    p_fr3_initial
    + delta_position_fr3
)
```

Trigger 동작:

- Trigger ON rising edge에서 OMY와 FR3 runtime initial position을 저장한다.
- Trigger ON 동안 runtime initial pose 기준 target position을 갱신한다.
- Trigger OFF 시 OMY pose를 더 이상 반영하지 않고 마지막 target 및 command를 유지한다.
- Trigger ON 직후 OMY delta position은 0에 가까워야 한다.

`AXIS_MAP`은 조작자 기준의 전후·좌우·상하 움직임을 FR3 task frame에 대응시키기 위한 position mapping이다. 이 행렬은 position 단축 실험을 통해 확정한 mapping이다. 향후 orientation retargeting에서는 OMY와 FR3 EE frame 및 회전 방향을 별도로 검증한 뒤 재사용 여부를 결정한다.

## 5. Position-only DLS IK

현재 IK objective에는 position error 3개만 포함한다.

```python
position_error = (
    fr3_target_position
    - fr3_current_position
)
```

MuJoCo의 translational Jacobian을 사용한다.

```python
jacp = np.zeros((3, fr3_model.nv))
jacr = np.zeros((3, fr3_model.nv))

mujoco.mj_jacSite(
    fr3_model,
    fr3_data,
    jacp,
    jacr,
    fr3_ee_site_id,
)

J_position = jacp[:, fr3_dof_indices]
```

index 역할:

- `qpos address`: 관절 상태 접근
- `dof address`: Jacobian column 선택

DLS 계산:

```python
dq = (
    J_position.T
    @ np.linalg.solve(
        J_position @ J_position.T
        + (IK_DAMPING ** 2) * np.eye(3),
        position_error,
    )
)

dq = IK_GAIN * dq
dq = np.clip(dq, -MAX_DQ, MAX_DQ)
```

실제 관절 상태와 actuator command는 분리한다.

```python
q_current = fr3_data.qpos[fr3_qpos_indices].copy()

fr3_q_command = np.clip(
    fr3_q_command + dq,
    fr3_joint_lower,
    fr3_joint_upper,
)

fr3_data.ctrl[fr3_actuator_indices] = fr3_q_command
mujoco.mj_step(fr3_model, fr3_data)
```

```text
q_current
= MuJoCo에서 측정한 실제 FR3 관절 상태
= 현재 EE pose와 Jacobian 계산에 사용

fr3_q_command
= position actuator에 전달하는 목표 관절 상태
= 이전 command에 dq를 연속적으로 누적

dq
= 현재 cycle에서 적용할 관절 command 수정량
```

현재 IK objective에는 orientation error를 포함하지 않는다. 따라서 OMY orientation을 FR3가 추종하지 않으며, position-only IK 동작 중 FR3 EE orientation은 관절 해에 따라 일부 변할 수 있다.

## 6. 주요 디버깅 결과

### 6.1 XML home pose와 runtime initial pose 혼용

Trigger ON 직후 OMY delta가 0이 아니고 FR3 target이 jump하며 일부 IK joint가 limit에 포화될 수 있었다. 원인은 XML home pose를 teleoperation 기준으로 사용한 것이다.

```python
delta_p_omy = p_omy_current - p_omy_home
```

실제 joint state 수신 후 Trigger ON 시 runtime initial pose를 캡처하도록 수정하였다.

```python
delta_p_omy = p_omy_current - p_omy_initial
```

### 6.2 좌표축 매핑

초기 `AXIS_MAP = I`에서는 조작자 기준 전후·좌우 움직임과 FR3 task direction이 일치하지 않았다. 단축 이동 실험으로 현재 `AXIS_MAP`을 확정하였다.

### 6.3 Trigger ON 시 FR3 하강 및 진동

Trigger ON 시 FR3가 아래로 내려앉거나 진동하는 현상이 발생하였다. 실제 qpos 기반 command 갱신 구조, DLS 수치 안정성, gain 및 cycle당 joint update를 함께 점검하였다. 이후 `q_current`와 지속 actuator command를 분리하고, DLS damping과 IK gain 및 joint-step limit을 보수적으로 설정하여 안정화하였다.

상세 raw 디버깅 기록: [debugging.md](debugging.md)

## 7. 검증 결과

현재 다음 position-only 동작을 확인하였다.

- Trigger ON 직후 target jump 없음
- OMY 전후 이동에 따른 FR3 전후 추종
- OMY 좌우 이동에 따른 FR3 좌우 추종
- OMY 상하 이동에 따른 FR3 상하 추종
- 정지 시 target과 FR3 EE 수렴
- joint-limit saturation 없음
- 눈에 띄는 발산과 진동 없음
- OMY를 initial pose 근처로 복귀시켰을 때 FR3도 initial position 근처로 복귀

대표 로그:

```text
tracking error: 0.04 mm
return error: 2.48 mm
max joint step: 0.000002 rad
```

위 수치는 안정화 이후 특정 cycle에서 기록한 대표 로그이며, 여러 trial의 평균 또는 최대 성능을 의미하지 않는다. return error에는 사용자가 OMY를 runtime initial pose에 정확히 되돌리지 못한 입력 오차가 포함될 수 있다.

현재 코드에서 평가하는 지표:

```python
tracking_error = np.linalg.norm(
    fr3_target_position - fr3_current_position
)

return_error = np.linalg.norm(
    fr3_current_position - fr3_initial_position
)

max_joint_step = np.max(np.abs(dq))
```

**Position-only Cartesian teleoperation MVP completed.**

현재 OMY orientation retargeting과 rotational IK는 포함하지 않는다.

## 8. 현재 파라미터

| Parameter | Current value |
|---|---:|
| `POSITION_SCALE` | `0.2` |
| `AXIS_MAP` | `[[0,1,0],[-1,0,0],[0,0,1]]` |
| `IK_DAMPING` | `0.05` |
| `IK_GAIN` | `0.05` |
| `MAX_DQ` | `0.002 rad/cycle` |
| configured control period | `0.002 s` |
| nominal control frequency | `500 Hz` |
| measured control frequency | not yet evaluated |

`500 Hz`는 configured loop period에서 계산한 nominal frequency이며, 실제 control frequency는 MuJoCo step 시간과 시스템 부하에 따라 달라질 수 있다.
