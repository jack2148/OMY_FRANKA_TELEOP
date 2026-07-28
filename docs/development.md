# FR3–OMY Cartesian Teleoperation Development Log

최초 작성: 2026-07-23<br>
최종 수정: 2026-07-28

> Current status: 6D position-orientation teleoperation and target-based clutch re-anchoring are operational in MuJoCo, while standardized quantitative evaluation remains incomplete.

## Phase 1 — Position-only Cartesian Teleoperation

Status: Completed on 2026-07-24

![Position-only Cartesian teleoperation](images/teleop/development/position_only_cartesian_teleoperation.gif)

위 GIF는 orientation retargeting과 rotational IK를 연결하기 전, 현재 기준으로 보존한 position-only Cartesian teleoperation 동작을 보여준다.

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

## Phase 2 — 6D Cartesian Teleoperation

Status: In progress on 2026-07-28

## 1. 작업 목표

2026-07-27 작업에서는 실제 OMY-L100의 EE motion을 MuJoCo FR3에 retargeting하는 기존 position-only 구조를 position + orientation 6D pose 제어로 확장하였다. Trigger 기반 clutch를 사용해 제한된 leader workspace에서도 반복 조작할 수 있도록 runtime relative pose를 사용하고, OMY FK부터 FR3 joint position command까지의 전체 흐름을 단계별로 검증하는 것을 목표로 했다.

## 2. 시작 상태

작업 시작 전에는 다음 기능이 이미 동작했다.

- `/leader/joint_states`를 MuJoCo OMY joint qpos에 반영
- `omy_ee_site`의 EE FK 및 runtime position delta 계산
- FR3 position target 생성
- translational DLS IK와 FR3 position actuator command

반면 orientation retargeting, rotational IK, 그리고 반복 Trigger 입력에서 OMY와 FR3의 기준 pose를 일관되게 갱신하는 clutch re-anchoring은 완전히 검증되지 않은 상태였다. XML home pose, runtime initial pose, clutch anchor의 역할도 position-only 코드 안에서 혼재할 여지가 있었다.

## 3. 구현한 시스템 구조

```text
Real OMY joint state
    → MuJoCo OMY FK
    → OMY current EE pose (p, R)
    → clutch runtime reference 기준 relative motion
    → OMY-to-FR3 axis mapping
    → FR3 target pose (p_target, R_target)
    → 6D task error
    → Damped Least Squares IK
    → FR3 joint position command
```

### 3.1 Relative position

현재 코드의 position target은 Trigger ON 시 저장한 OMY runtime position을 기준으로 계산한다.

```python
delta_position_omy = omy_current_position - omy_initial_position
delta_position_fr3 = POSITION_SCALE * (AXIS_MAP @ delta_position_omy)
fr3_target_position = fr3_initial_position + delta_position_fr3
```

현재 position mapping은 다음과 같다.

```python
AXIS_MAP = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])
```

현재는 재클러치 후에도 기존에 검증한 고정축 position mapping을 유지한다. clutch 시점 OMY local frame을 이용하는 방식은 방향 반전 문제가 확인되어 baseline에서 제외했다.

### 3.2 Relative orientation

Euler angle을 단순히 빼지 않고 rotation matrix의 상대변환을 계산한 뒤 rotation vector와 angle로 검증한다.

```python
R_omy_rel = R_omy_anchor.T @ R_omy_current
```

orientation mapping 후보는 position mapping과 분리된 변수로 유지한다.

```python
R_FR3_FROM_OMY_ORIENTATION = AXIS_MAP.copy()
R_rel_mapped = (
    R_FR3_FROM_OMY_ORIENTATION
    @ R_omy_delta_rotation_for_target
    @ R_FR3_FROM_OMY_ORIENTATION.T
)
R_fr3_target = R_fr3_anchor @ R_rel_mapped
```

현재 코드에서는 좌표계 방향을 확인하기 위해 OMY relative rotation을 RPY로 분해하고 `ORIENTATION_RPY_SIGN = [1, -1, -1]`을 적용한다. 이는 Euler subtraction을 통한 target 생성이 아니라, relative rotation을 진단한 뒤 rotation matrix로 재구성하는 orientation direction adjustment이다.

### 3.3 6D DLS IK

현재 baseline parameter는 실제 코드에서 다음과 같다.

| Parameter | Current value |
|---|---:|
| `POSITION_SCALE` | `0.7` |
| `IK_DAMPING` | `0.05` |
| `IK_GAIN` | `0.05` |
| `ROTATION_IK_WEIGHT` | `0.1` |
| `MAX_DQ` | `0.004 rad/cycle` |

position Jacobian `J_pos`와 angular Jacobian `J_rot`를 결합하고, rotation error에 `ROTATION_IK_WEIGHT`를 곱한다.

```python
J_task = np.vstack((J_pos, ROTATION_IK_WEIGHT * J_rot))
e_task = np.concatenate((position_error,
                         ROTATION_IK_WEIGHT * rotation_error))

dq_raw = J_task.T @ np.linalg.solve(
    J_task @ J_task.T + IK_DAMPING**2 * np.eye(6),
    e_task,
)
dq_raw = IK_GAIN * dq_raw
dq_cmd = np.clip(dq_raw, -MAX_DQ, MAX_DQ)
```

`dq_raw`는 clipping 전 DLS 결과이고 `dq_cmd`는 joint-step limit을 적용한 command이다. 현재 CSV logging은 `ENABLE_CSV_LOGGING = True`이며 실행별 `logs/target_anchor_TIMESTAMP.csv`에 기록한다.

## 4. Clutch re-anchoring 구현 상태

재클러치에서 최초 runtime pose를 계속 사용하면 OMY와 FR3가 이미 이동한 뒤 Trigger를 다시 눌렀을 때 target discontinuity가 발생할 수 있다. OMY anchor만 새로 저장하고 FR3 anchor를 갱신하지 않는 경우 position/orientation error spike와 joint-step saturation으로 이어질 수 있다.

현재 Trigger rising-edge 코드는 OMY current pose와 당시 유지 중인 FR3 target pose를 하나의 clutch anchor pair로 저장한다.

```python
omy_anchor_position = omy_current_position.copy()
omy_anchor_rotation = omy_current_rotation.copy()
fr3_anchor_position = fr3_target_position.copy()
fr3_anchor_rotation = fr3_target_rotation.copy()
```

이후 position과 orientation target은 모두 이 anchor pair 기준으로 계산한다. Trigger OFF 동안에는 마지막 FR3 target과 command를 유지하며, rising edge에서 target anchor jump를 mm/deg로 출력한다. `fr3_q_command`는 재클러치 시 reset하지 않고 기존 command state를 유지한다.

상세한 원인 분석과 권장 anchor pair 구조는 [debugging.md](debugging.md)에 분리해 기록한다.

## 5. 최초 결과

![Initial 6D teleoperation tracking](images/teleop/development/initial_6d_tracking.png)

초기 plot에는 다음 네 값이 표시된다.

- position tracking error
- OMY / FR3 target / FR3 actual orientation
- orientation tracking error
- maximum joint step

초기 CSV(`logs/orientation_teleop.csv`)에서 확인된 최대값은 position error 약 `756.5 mm`, orientation tracking error 약 `90.0 deg`, maximum joint step `0.004 rad`이다. 이 로그는 여러 동작 구간과 timestamp gap을 한 그래프에 연결한 기록이므로, 구간 사이의 직선은 실제 연속 동작이 아닌 plot artifact일 수 있다. 재클러치 기준 불일치와 target discontinuity 가능성이 포함된 초기 결과로 해석한다.

## 6. 최종 결과

![Final 6D teleoperation tracking](images/teleop/development/final_6d_tracking.png)

`logs/MAX_DQ_0.004_v2.csv` 기준으로 확인된 값은 position error peak 약 `7.78 mm`, orientation tracking error peak 약 `10.02 deg`, maximum joint step `0.004 rad`이다. 그래프에서는 OMY와 FR3 target orientation이 대체로 같은 방향으로 생성되고, FR3 actual orientation은 빠른 구간에서 lag를 보인 뒤 target을 따라간다.

이 결과는 MuJoCo의 특정 로그 세션에 대한 관찰이며, 동일 입력 궤적을 사용한 엄밀한 benchmark는 아니다. 최신 코드에서는 CSV를 실행별 `logs/target_anchor_TIMESTAMP.csv`로 기록한다.

## 7. 최초 결과와 최종 결과 비교

| 항목 | 최초 상태 | 최종 관찰 | 해석 |
|---|---|---|---|
| Pose control | position 중심, orientation 미검증 | position + orientation 6D DLS | 기능 확장 |
| Orientation target mapping | 검증 전 | OMY relative와 FR3 target이 대체로 일치 | 기본 mapping 확인 |
| Re-clutch continuity | 큰 target jump 가능 | target-based re-anchoring은 아직 미반영 | 추가 구현 필요 |
| Peak position error | 약 756.5 mm | 약 7.78 mm | 로그 세션 기준 개선 |
| Peak orientation error | 약 90.0 deg | 약 10.02 deg | 로그 세션 기준 개선 |
| Joint-step limit | `0.004 rad`까지 관찰 | `0.004 rad` baseline | raw saturation ratio는 새 logging 재활성화 후 계산 |
| Final convergence | 분석 곤란 | 저속 구간에서 target을 추종 | 빠른 동작과 재클러치 추가 검증 필요 |

초기와 최종 실험의 입력 궤적 및 로그 세션이 완전히 동일하지 않으므로, 위 비교는 정량 benchmark가 아닌 개발 전후의 qualitative comparison이다.

## 8. 현재 baseline

완료 또는 확인된 항목:

- runtime OMY FK
- relative position retargeting
- relative orientation retargeting
- OMY–FR3 axis mapping 후보
- 6D DLS IK
- orientation marker visualization
- target/actual/orientation error debug 출력
- basic plot visualization

남은 항목:

- FR3 target 기반 clutch anchor pair 적용
- 동일 궤적 기반 정량 비교
- RMSE, peak error, settling time, saturation ratio 자동 계산
- `MAX_DQ`를 `q_dot_max * dt` 방식으로 전환
- translation-only / rotation-only / combined 표준 실험
- joint-limit 및 singularity 처리
- actual FR3 연결 전 safety layer
- target velocity 및 acceleration conditioning 검토

## 9. 다음 작업

1. `fr3_target` 기반 clutch anchor pair 구현 및 재클러치 연속성 검증
2. CSV logging 재활성화 후 `raw_max_dq`, `cmd_max_dq`, `dq_saturated`, `dt` 기록
3. 느린 단축 왕복, 빠른 단축 왕복, position + rotation 동시 움직임 실험
4. saturation ratio와 RMSE, peak error, settling time 계산
5. 결과 plot과 재현 절차 문서화

## 10. 결론

현재 MuJoCo 검증 단계에서는 OMY-L100 → FR3 position + orientation 6D Cartesian teleoperation의 기본 pipeline과 rotational DLS IK가 동작한다. `MAX_DQ = 0.004` baseline에서 기록된 한 로그는 position error 약 `7.78 mm`, orientation error 약 `10.02 deg`를 보였다. 다만 재클러치 anchor continuity와 빠른 동작 안정성은 추가 검증이 필요하므로, 실로봇 적용 준비가 완료되었다고 판단하지 않는다.
