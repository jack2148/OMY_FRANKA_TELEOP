# FR3–OMY Cartesian Teleoperation 디버깅 기록

최초 작성: 2026-07-24
최종 수정: 2026-07-24

개발 문서: [FR3–OMY Cartesian Teleoperation 개발 기록](development.md)

이 문서는 FR3–OMY Cartesian teleoperation 구현 과정에서 발생한 주요 문제, 원인 분석, 수정 방법과 검증 결과를 기록한다.

---

## 1. Runtime initial pose 기준 오류

### 1.1 문제 상황

Bridge 실행 직후 OMY를 거의 움직이지 않았는데도 큰 position delta와 rotation delta가 발생하였다.

```text
OMY delta position ≠ [0, 0, 0]
OMY delta rotation ≠ Identity
```

이 값이 FR3 target pose에 반영되면서 시작 직후 target jump가 발생하였고, FR3 IK 결과에서 여러 관절이 joint limit에 포화되었다.

관측된 saturation:

* Joint2: upper limit
* Joint4: upper limit
* Joint7: lower limit

### 1.2 원인

실제 teleoperation 시작 pose가 아니라 MuJoCo XML의 home pose를 증분 계산 기준으로 사용한 것이 원인이었다.

기존 계산:

```python
delta_p_omy = p_omy_current - p_omy_home

R_omy_delta = (
    R_omy_home.T
    @ R_omy_current
)
```

실제 OMY 시작 자세와 XML home 자세가 다르기 때문에 Bridge 실행 직후부터 큰 상대 pose가 생성되었다.

### 1.3 해결

정상적인 `/leader/joint_states`를 수신한 뒤 OMY MuJoCo FK를 수행하고, Trigger ON 시점의 current EE pose를 runtime initial pose로 저장하였다.

```python
p_omy_initial = p_omy_current.copy()
R_omy_initial = R_omy_current.copy()

p_fr3_initial = p_fr3_current.copy()
R_fr3_initial = R_fr3_current.copy()
```

이후 position과 rotation 증분을 runtime initial pose 기준으로 계산한다.

```python
delta_p_omy = (
    p_omy_current
    - p_omy_initial
)

R_omy_delta = (
    R_omy_initial.T
    @ R_omy_current
)
```

Position-only target은 다음과 같이 계산한다.

```python
delta_p_fr3 = (
    POSITION_SCALE
    * (
        AXIS_MAP
        @ delta_p_omy
    )
)

p_fr3_target = (
    p_fr3_initial
    + delta_p_fr3
)
```

### 1.4 성공 조건

Trigger ON 직후 다음 조건을 만족해야 한다.

```text
OMY delta position ≈ [0, 0, 0]

OMY delta rotation ≈
[[1, 0, 0],
 [0, 1, 0],
 [0, 0, 1]]

FR3 target position ≈ FR3 initial position
```

Position 기준 확인 예시:

```python
np.linalg.norm(delta_p_omy) < 1e-3
```

Rotation tolerance는 센서 노이즈와 제어 주기를 고려해 별도로 설정한다.

또한 Trigger ON 직후 FR3 target jump와 joint-limit saturation이 발생하지 않아야 한다.

### 1.5 결론

XML home pose는 모델 초기화와 검증을 위한 기준이다. Teleoperation 증분 계산은 실제 조작을 시작한 시점의 runtime initial pose를 기준으로 수행해야 한다.

```text
Trigger ON
→ OMY·FR3 runtime initial pose 저장
→ OMY current pose와 initial pose의 차이 계산
→ FR3 initial pose에 상대 이동량 적용
→ FR3 target pose 생성
```

---

## 2. OMY–FR3 수평축 mapping 문제

### 2.1 문제 상황

초기 position-only marker 실험에서 다음 동작이 관찰되었다.

| OMY 조작자 기준 이동 | FR3 marker 이동 |
| ------------- | ------------- |
| 앞으로           | 오른쪽           |
| 뒤로            | 왼쪽            |
| 오른쪽           | 앞으로           |
| 왼쪽            | 뒤로            |

Position delta 계산과 marker 이동은 연속적이었지만, OMY와 FR3의 수평축 방향이 조작자 기준과 일치하지 않았다.

### 2.2 해결

전후·좌우·상하 단축 이동 실험을 수행하고 다음 position mapping을 적용하였다.

```python
AXIS_MAP = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])
```

매핑 의미:

```text
FR3 X =  OMY Y
FR3 Y = -OMY X
FR3 Z =  OMY Z
```

변경 후 조작자 기준 전후·좌우·상하 입력과 FR3 이동 방향이 일치하는 것을 확인하였다.

### 2.3 주의점

현재 행렬은 position mapping을 기준으로 실험적으로 결정하였다.

행렬 자체는 determinant가 `+1`인 proper rotation이지만, position mapping이 맞았다는 이유만으로 orientation mapping에도 검증 없이 재사용하지 않는다.

Orientation retargeting을 구현할 때는 다음 항목을 별도로 확인해야 한다.

* OMY EE frame의 실제 축 방향
* FR3 EE frame의 실제 축 방향
* operator frame 정의
* roll, pitch, yaw 단축 회전 방향
* relative rotation 적용 순서

---

## 3. FR3 target 로그 식별 문제

### 3.1 문제 상황

터미널에서 서로 다른 FR3 target 값이 연속 출력되면서, 현재 OMY delta와 target position이 일치하지 않는 것처럼 보였다.

### 3.2 확인 결과

코드의 계산 순서는 정상적이었다.

```text
OMY delta 계산
→ FR3 mapped delta 계산
→ FR3 target 계산
→ 로그 출력
```

주요 원인은 다음과 같은 로그 식별 문제로 판단하였다.

* 로그 출력 주기가 `0.2 s`로 제어 주기보다 느림
* ROS launch의 여러 프로세스 출력이 같은 터미널에 섞임
* 여러 cycle의 여러 줄 로그를 하나의 cycle처럼 읽기 쉬움
* 이전 실행 프로세스가 남아 있을 가능성

### 3.3 개선 방법

Cycle 번호와 핵심 값을 한 줄에 함께 출력한다.

```python
cycle_count += 1

print(
    f"[cycle={cycle_count}] "
    f"OMY delta={delta_position_omy}, "
    f"mapped={delta_position_fr3}, "
    f"FR3 target={fr3_target_position}"
)
```

필요한 경우 실행 전에 관련 프로세스가 중복 실행 중인지 확인한다.

---

## 4. Trigger ON 시 FR3 하강 및 진동

### 4.1 문제 상황

Position marker는 정상적으로 움직였지만, Trigger ON 시 FR3가 아래로 내려앉거나 흔들리는 듯한 현상이 발생하였다.

관찰된 특징:

* Trigger OFF에서는 FR3가 자세를 유지함
* Trigger ON에서만 불안정한 움직임이 나타남
* 작은 position error에도 FR3가 필요 이상으로 크게 반응함
* position-only IK를 활성화했을 때 현상이 발생함

### 4.2 점검한 항목

다음 항목을 분리하여 확인하였다.

* FR3 actuator command 초기값
* Trigger 전후 command 연속성
* `q_current`와 actuator command 차이
* translational Jacobian column index
* DLS damping
* IK gain
* cycle당 최대 joint update
* joint-limit saturation

실제 관절 상태와 actuator command는 역할을 구분하였다.

```text
q_current
= MuJoCo에서 측정한 실제 관절 상태

fr3_q_command
= position actuator에 전달하는 목표 관절 상태
```

Command를 별도로 관리하는 구조는 mode transition과 actuator target을 추적하는 데 유용하지만, `q_current + dq` 구조 자체를 단독 원인으로 확정하지 않는다.

### 4.3 안정화 방법

DLS damping과 IK gain을 보수적으로 설정하고 cycle당 joint update를 제한하였다.

```python
IK_DAMPING = 0.05
IK_GAIN = 0.05
MAX_DQ = 0.002
```

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

현재 설정에서 Trigger ON 시 하강과 눈에 띄는 진동이 감소하였고, 전후·좌우·상하 position 추종이 안정적으로 동작하는 것을 확인하였다.

### 4.4 파라미터 해석

* `IK_DAMPING`: Jacobian 역산의 수치적 불안정성과 과도한 joint update를 완화
* `IK_GAIN`: 한 cycle에서 적용하는 IK update의 크기를 조절
* `MAX_DQ`: 특정 cycle에서 발생할 수 있는 최대 joint command 변화를 제한

진동이 재발할 경우 한 번에 여러 파라미터를 동시에 변경하지 않고, 우선 raw `dq`, applied `dq`, tracking error를 비교해야 한다.

---

## 5. 왕복 오차 해석

### 5.1 출력 값

현재 다음 값을 출력한다.

```text
tracking error: xx.xx mm
return error: xx.xx mm
max joint step: x.xxxxxx rad
```

각 값의 의미:

```text
tracking error
= 현재 FR3 target position과 실제 FR3 EE position의 차이

return error
= 현재 FR3 EE position과 Trigger ON 시 저장한 FR3 initial position의 차이

max joint step
= 현재 cycle에서 적용된 joint update의 최대 절댓값
```

### 5.2 주의점

`return error`는 FR3 제어 오차만을 의미하지 않는다.

OMY를 runtime initial pose에 정확히 되돌리지 못하면 FR3 target 자체가 initial position에서 떨어져 있으므로, tracking이 정확해도 return error가 남을 수 있다.

따라서 복귀 성능을 정확히 해석하려면 다음 값을 구분해야 한다.

```python
omy_return_error = np.linalg.norm(
    p_omy_current
    - p_omy_initial
)

fr3_target_return_error = np.linalg.norm(
    p_fr3_target
    - p_fr3_initial
)

fr3_actual_return_error = np.linalg.norm(
    p_fr3_current
    - p_fr3_initial
)

tracking_error = np.linalg.norm(
    p_fr3_target
    - p_fr3_current
)
```

### 5.3 대표 로그

안정화 이후 특정 cycle에서 다음 값이 확인되었다.

```text
tracking error: 0.04 mm
return error: 2.48 mm
max joint step: 0.000002 rad
```

해당 로그에서 FR3 tracking error는 매우 작았고, return error는 OMY가 runtime initial pose에 완전히 복귀하지 않아 발생한 mapped target displacement와 유사한 수준이었다.

이 값은 특정 cycle의 대표 로그이며, 여러 trial의 평균 또는 최대 성능을 의미하지 않는다.

---

## 6. 현재 디버깅 결론

* XML home pose와 runtime initial pose를 구분한다.
* Trigger ON 시점의 실제 OMY·FR3 pose를 증분 계산 기준으로 사용한다.
* Position target과 IK를 분리하여 검증한다.
* OMY–FR3 position mapping은 전후·좌우·상하 단축 실험으로 확정한다.
* Position mapping을 orientation에 검증 없이 재사용하지 않는다.
* 다중 프로세스 로그에는 cycle 번호를 부여한다.
* 실제 관절 상태와 actuator command의 역할을 구분한다.
* DLS damping, IK gain, joint-step limit을 한 번에 하나씩 조정한다.
* Position-only DLS IK를 안정화한 뒤 orientation retargeting을 검증한다.
* Orientation target 검증 없이 rotational Jacobian과 6D IK를 동시에 추가하지 않는다.
