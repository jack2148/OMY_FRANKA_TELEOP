# FR3–OMY EE Pose 디버깅 기록

작성일: 2026-07-24

개발 문서: [developt.md](developt.md)

## 문제 상황

Bridge 실행 직후 OMY를 거의 움직이지 않았는데도 큰 position delta와 rotation delta가 발생했다.

```text
OMY delta position ≠ [0, 0, 0]
OMY delta rotation ≠ Identity
```

이로 인해 FR3 target pose가 시작부터 크게 이동했고, FR3 IK 결과에서 여러 joint가 limit에 포화되었다.

## 원인 가설

OMY의 실제 teleoperation 시작 pose가 아니라 MuJoCo XML의 home pose를 기준으로 증분을 계산했을 가능성이 높다.

기존 계산은 다음과 같다.

```python
delta_p_omy = p_omy_current - p_omy_home
R_omy_delta = R_omy_home.T @ R_omy_current
```

실제 OMY 시작 자세가 XML home 자세와 다르면, Bridge 실행 직후부터 큰 delta가 발생할 수 있다.

## 해결 방법

정상적인 `/leader/joint_states`를 수신하고 OMY MuJoCo FK를 수행한 뒤, Bridge가 활성화되는 시점의 current EE pose를 runtime initial pose로 저장한다.

```python
p_omy_initial = p_omy_current.copy()
R_omy_initial = R_omy_current.copy()

p_fr3_initial = p_fr3_current.copy()
R_fr3_initial = R_fr3_current.copy()
```

이후 증분은 XML home pose가 아니라 runtime initial pose를 기준으로 계산한다.

```python
delta_p_omy = p_omy_current - p_omy_initial

R_omy_delta = (
    R_omy_initial.T
    @ R_omy_current
)
```

FR3 target position은 OMY와 FR3의 축 방향 및 스케일을 적용한 뒤 계산한다.

```python
p_fr3_target = (
    p_fr3_initial
    + scale_matrix
    @ axis_map
    @ delta_p_omy
)
```

실제 좌표계 방향에 따라 `axis_map`의 부호와 `scale_matrix`의 값은 별도로 조정해야 한다.


## 성공 조건

Bridge 시작 직후 다음 조건을 만족해야 한다.

```text
OMY delta position ≈ [0, 0, 0]

OMY delta rotation ≈
[[1, 0, 0],
 [0, 1, 0],
 [0, 0, 1]]

FR3 target position ≈ FR3 initial position
```

수치 기준:

```python
np.linalg.norm(delta_p_omy) < 1e-3
np.linalg.norm(R_omy_delta - np.eye(3)) < tolerance
```

또한 Bridge 시작 직후 FR3 IK joint가 joint limit에 포화되지 않아야 한다.


## 핵심 결론

XML의 home pose는 MuJoCo 모델 초기화용 기준이고, teleoperation 증분 계산의 기준은 실제 teleoperation 시작 시점에 수신한 OMY EE pose로 설정해야 한다.

```text
OMY runtime initial pose
    ↓
OMY current pose
    ↓
OMY delta pose
    ↓
FR3 initial pose에 적용
    ↓
FR3 target pose
```

## 2. OMY–FR3 수평축 mapping 문제

초기 position-only marker 실험 결과는 다음과 같았다.

| OMY 이동 | FR3 marker 이동 |
|---|---|
| 앞으로 | 오른쪽 |
| 뒤로 | 왼쪽 |
| 오른쪽 | 앞으로 |
| 왼쪽 | 뒤로 |

수평축의 방향 부호는 유지되지만 X축과 Y축의 역할이 서로 바뀐 상태였다.

현재 position-only mapping:

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

이 mapping은 position-only 검증용이다. position mapping 행렬을 orientation mapping에 그대로 사용하지 않는다.

## 3. FR3 target 로그 순서 확인

서로 다른 FR3 target이 출력되어 current delta와 target이 맞지 않는 것처럼 보이는 현상이 있었다.

현재 코드는 delta 계산 후 target을 계산하고, 그 다음 같은 출력 블록에서 target을 출력한다. 따라서 target을 먼저 출력하는 구조는 아니다.

출력 주기가 `0.2 s`이고 ROS launch의 여러 프로세스가 동시에 출력할 수 있어 서로 다른 cycle의 로그가 섞여 보일 수 있다.

## 4. `q_current + dq`로 인한 FR3 처짐

실제 qpos는 중력과 actuator tracking 오차로 command보다 약간 처질 수 있다. 이 값을 매 cycle 새로운 command 기준으로 사용하면 command가 처진 자세를 따라갈 수 있다.

기존 구조:

```python
fr3_q_target = q_current + dq
```

수정 구조:

```text
q_current = 실제 MuJoCo 관절 상태
fr3_q_command = actuator 목표 관절 상태
```

현재는 `fr3_q_command`를 이전 command에 누적한다. Trigger ON 순간에도 실제 qpos가 아니라 직전 actuator command를 유지한다.

## 5. command 누적 이후 진동

`fr3_q_command`를 분리한 뒤에도 진동이 관찰될 수 있었다. FR3가 command를 따라가는 동안 command에 `dq`가 계속 누적되면 목표를 앞질러 갈 수 있기 때문이다.

현재 안정화 파라미터:

```python
IK_DAMPING = 0.05
IK_GAIN = 0.05
MAX_DQ = 0.002
```

진동이 다시 발생하면 먼저 `MAX_DQ`를 낮춰 command 변화 속도를 줄인다. `IK_GAIN`과 `MAX_DQ`를 동시에 크게 변경하지 않는다.

## 6. 왕복 오차 검증

현재 로그는 다음 값을 출력한다.

```text
tracking error: xx.xx mm
return error: xx.xx mm
max joint step: x.xxxxxx rad
```

왕복은 Trigger ON 상태에서 OMY 이동 후 원위치 복귀까지 진행한다. `return_error`를 기록한 뒤 Trigger OFF로 전환한다. 별도의 복귀 오차용 trigger는 없다.

## 7. 현재 디버깅 결론

- XML home pose와 runtime initial pose를 분리한다.
- OMY–FR3 position mapping은 한 축씩 실험해 확정한다.
- position mapping 행렬을 orientation mapping에 그대로 사용하지 않는다.
- `q_current`와 지속 command를 분리한다.
- position-only DLS IK를 먼저 안정화한다.
- orientation target 시각 검증 없이 rotational Jacobian과 6D IK를 동시에 추가하지 않는다.
