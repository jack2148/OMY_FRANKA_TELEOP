# FR3–OMY EE Pose 구현 정리

작성일: 2026-07-23

## 작업 목표

OMY-L100 리더암의 실제 ROS joint state를 기준으로 OMY EE pose를 계산하고, 이후 FR3 MuJoCo 시뮬레이션의 EE pose 제어로 확장한다.

이번 문서에는 bridge의 실제 구현 코드는 포함하지 않는다. bridge는 별도 디버깅 대상으로 둔다.

## 1. OMY EE pose 계산 코드

파일:

```text
launch/omy_EEposes.py
```

OMY MuJoCo 모델의 `home` keyframe을 적용한 뒤 `omy_ee_site`의 위치와 회전을 계산한다.

```text
OMY model 로드
    ↓
home keyframe 적용
    ↓
mujoco.mj_forward()
    ↓
omy_ee_site 검색
    ↓
site_xpos, site_xmat 저장
```

저장되는 값:

- `p_omy_0`: OMY home EE 위치
- `R_omy_0`: OMY home EE 회전행렬
- `omy_home`: `(p_omy_0, R_omy_0)`

현재 코드는 home pose를 한 번 출력하는 기준 pose 확인용 코드이다.

## 2. FR3 EE pose 계산 코드

파일:

```text
launch/FR3_EEposes.py
```

FR3 Menagerie의 `scene.xml`을 로드하고, XML에 이미 정의되어 있는 `attachment_site`를 EE 기준점으로 사용한다.

```xml
<body name="fr3_link7">
    <site name="attachment_site" pos="0 0 0.107"/>
</body>
```

계산 과정:

```text
FR3 scene.xml 로드
    ↓
home keyframe 적용
    ↓
mujoco.mj_forward()
    ↓
attachment_site 검색
    ↓
site_xpos, site_xmat 저장
```

저장되는 값:

- `p_fr3_0`: FR3 home EE 위치
- `R_fr3_0`: FR3 home EE 회전행렬
- `fr3_home`: `(p_fr3_0, R_fr3_0)`

현재 코드는 FR3 home pose를 한 번 출력하는 기준 pose 확인용 코드이다.

## 3. FR3 시뮬레이션과 OMY 실제 topic 연동 구조

실행 구조는 다음과 같이 구성한다.

```text
OMY-L100 실제 리더암
    ↓
/leader/joint_states
    ↓
FR3–OMY sync 프로그램
    ├─ OMY joint state 수신
    ├─ OMY MuJoCo FK 계산
    ├─ OMY EE pose 계산
    ├─ home 기준 증분 pose 계산
    ├─ FR3 목표 EE pose 생성
    ├─ FR3 IK 계산
    └─ FR3 MuJoCo viewer 실행
```

OMY MuJoCo 모델은 FK 계산용으로 사용하고, 화면에는 FR3 MuJoCo 모델만 표시하는 방향이다.

기존 OMY 단독 실행 구조:

```text
launch/omy_sim_sync.py
    ├─ OMY-L100 leader 실행
    └─ OMY MuJoCo 시뮬레이션 실행
```

FR3 연동 실행 구조:

```text
launch/fr3_omy_sync.py
    ├─ OMY-L100 leader 실행
    └─ FR3 MuJoCo 기반 sync 프로그램 실행
```

FR3 시뮬레이션만 확인하는 명령:

```bash
cd ~/omy_franka_teleop
/usr/bin/python3 -m mujoco.viewer \
  --mjcf=mujoco_menagerie/franka_fr3/scene.xml
```

OMY joint state 확인:

```bash
source /opt/ros/humble/setup.bash
source ~/omy_franka_teleop/open_manipulator_omy/install/setup.bash

ros2 topic echo /leader/joint_states
```

## 4. OMY 시뮬레이션에서 EE pose 변화 확인

OMY 모델의 `omy_ee_site`는 `link6` body 내부에 있다.

```xml
<site
    name="omy_ee_site"
    pos="0 -0.109 0"
    size="0.01"
    rgba="0 1 0 1"
/>
```

`Joint6`의 회전축은 Y축이다.

```xml
<default class="Joint6">
    <joint axis="0 1 0"/>
</default>
```

따라서 site가 Joint6 회전축 위에 있으면 다음과 같이 동작한다.

```text
Joint6 회전
    ├─ EE 회전행렬은 변화
    └─ EE 위치는 거의 변화하지 않음
```

이는 site가 회전축 위에 있기 때문에 정상적인 결과이다. Joint6 회전에 따른 위치 변화까지 시각적으로 확인하려면 회전축에서 떨어진 가상의 tool tip offset이 필요하다.

예:

```xml
<site
    name="omy_ee_site"
    pos="0 -0.109 0.02"
    size="0.01"
    rgba="0 1 0 1"
/>
```

단, offset을 추가하면 EE는 플랜지 중심이 아니라 가상의 tool tip 기준이 된다. 최종 pose 분석에서는 실제 EE 기준에 맞는 offset을 사용해야 한다.

## 5. 확인된 MuJoCo API

home keyframe 적용:

```python
home_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home",
)

mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)
```

site ID 검색:

```python
site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "omy_ee_site",
)
```

site pose 읽기:

```python
position = data.site_xpos[site_id].copy()
rotation = data.site_xmat[site_id].reshape(3, 3).copy()
```

## 6. 다음 작업

1. OMY 실제 `/leader/joint_states` 수신 확인
2. OMY joint state를 OMY MuJoCo FK에 적용
3. OMY current EE pose와 home pose의 증분 계산
4. FR3 home pose에 증분 pose 적용
5. FR3 DLS IK 적용
6. FR3 joint limit 및 collision 처리
7. position mode와 joystick mode 분리

