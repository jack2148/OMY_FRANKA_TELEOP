# FR3–OMY EE Pose 구현 기록

작성일: 2026-07-23

## 1. 작업 목표

실제 OMY-L100 리더암의 ROS joint state를 OMY MuJoCo 모델에 적용해 실시간 EE pose를 계산하고, 이후 해당 움직임을 FR3 MuJoCo의 목표 EE pose로 변환하는 Cartesian teleoperation 구조를 구현한다.

이번 작업에서는 다음 단계까지 진행하였다.

- OMY와 FR3의 home EE pose 계산
- 실제 OMY joint state를 이용한 실시간 FK 계산
- OMY MuJoCo joint synchronization 검증
- OMY→FR3 bridge 및 IK 초기 구현
- 초기 pose 기준 오류 확인

## 2. EE 기준점

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

각 모델의 home keyframe을 적용한 뒤 `mujoco.mj_forward()`를 실행하고, `site_xpos`와 `site_xmat`을 이용해 EE pose를 계산하였다.

```python
position = data.site_xpos[site_id].copy()
rotation = data.site_xmat[site_id].reshape(3, 3).copy()
```

## 3. 실제 OMY joint state 기반 실시간 FK

실행 구조:

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

실시간으로 계산되는 값:

- `p_omy_current`
- `R_omy_current`

### 실시간 동기화 결과

![Real-time OMY-L100 Synchronization](real-time-omy-l100-synchronization.gif)

실제 OMY-L100의 `/leader/joint_states`가 MuJoCo 모델에 반영되고, 실제 arm motion에 따라 MuJoCo joint state와 EE pose가 실시간으로 갱신되는 것을 확인하였다.

이 영상은 OMY→FR3 teleoperation 완료 결과가 아니라 다음 파이프라인의 중간 검증 결과이다.

```text
실제 OMY joint state
→ MuJoCo OMY synchronization
→ 실시간 FK
→ EE pose 계산
```

## 4. Home pose와 runtime initial pose

이번 작업에서 가장 중요한 구분은 XML의 home pose와 teleoperation 시작 시점의 runtime initial pose이다.

### Home pose

XML keyframe에서 정의된 고정 configuration 기준 pose이다.

주요 용도:

- 모델 검증
- EE site 위치 확인
- 시뮬레이션 초기 자세 설정
- IK 초기값 설정

### Runtime initial pose

실제 OMY joint state를 수신하고 FK를 수행한 뒤, bridge를 활성화하는 시점에 캡처한 EE pose이다.

```python
p_omy_initial = p_omy_current.copy()
R_omy_initial = R_omy_current.copy()

p_fr3_initial = p_fr3_current.copy()
R_fr3_initial = R_fr3_current.copy()
```

OMY의 상대 이동량은 runtime initial pose 기준으로 계산해야 한다.

```python
delta_p_omy = p_omy_current - p_omy_initial

R_omy_delta = (
    R_omy_initial.T
    @ R_omy_current
)
```

FR3 target position은 다음과 같이 생성한다.

```python
p_fr3_target = (
    p_fr3_initial
    - scale_matrix
    @ axis_map
    @ delta_p_omy
)
```

## 5. Joint6 회전 해석

OMY Joint6의 local 회전축은 Y축이다.

```xml
<default class="Joint6">
    <joint axis="0 1 0"/>
</default>
```

`omy_ee_site`가 Joint6 회전축 위에 위치하면 Joint6이 회전할 때 EE rotation은 변하지만 position 변화는 매우 작을 수 있다.

```text
Joint6 회전
    ├─ EE rotation 변화
    └─ EE position 변화는 작음
```

이는 FK 오류가 아니라 EE site와 Joint6 회전축 사이의 기하학적 관계에 따른 정상적인 결과이다.

Joint6 회전에 따른 위치 변화를 시각적으로 확인하려면 회전축과 수직인 방향으로 별도의 debug tool site를 추가할 수 있다.

## 6. 확인된 문제

Bridge 초기 구현에서 다음 현상이 발생하였다.

- Bridge 시작 직후 OMY delta position이 0이 아님
- OMY delta rotation이 identity와 크게 다름
- FR3 IK 결과에서 여러 joint가 limit에 포화됨

관측된 IK saturation:

- Joint2 = upper limit
- Joint4 = upper limit
- Joint7 = lower limit

가장 가능성이 높은 원인은 실제 teleoperation 시작 pose가 아니라 XML home pose를 기준으로 OMY 증분을 계산한 것이다.

잘못된 방식:

```python
delta_p_omy = p_omy_current - p_omy_home
```

수정할 방식:

```python
delta_p_omy = p_omy_current - p_omy_initial
```

잘못된 큰 OMY delta가 FR3 target pose를 시작부터 멀리 이동시켰고, 그 결과 FR3 IK가 joint limit에 포화된 것으로 추정한다.

