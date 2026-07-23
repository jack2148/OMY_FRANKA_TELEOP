# FR3–OMY EE Pose 디버깅 정리

작성일: 2026-07-23

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
