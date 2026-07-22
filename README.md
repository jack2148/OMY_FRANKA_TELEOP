## Environment

- Ubuntu: 22.04
- ROS: Humble
- Python: 3.10
- MuJoCo: 3.10.0
- OMY package:
- Franka model: Franka Research 3

## 1. 초기 환경 세팅 과정

- 1.1 ROS 2 Humble 환경 준비
    
    ```
    source /opt/ros/humble/setup.bash
    ```
    
    필요하면 사용자 계정을 `dialout` 그룹에 추가합니다.
    
    ```
    sudo usermod -aG dialout $USER
    newgrp dialout
    ```
    
- 1.2 OMY 저장소 Clone
    
    ```
    cd ~/omy_franka_teleop
    
    git clone -b feature-omy-humble \
      https://github.com/ROBOTIS-GIT/open_manipulator.git \
      open_manipulator_omy
    ```
    
    워크스페이스 구조:
    
    ```
    ~/omy_franka_teleop/
    └── open_manipulator_omy/
        ├── open_manipulator_bringup/
        ├── open_manipulator_description/
        ├── ros2_controller/
        ├── src/
        └── ...
    ```
    
    반드시 `open_manipulator_omy`가 ROS 2 워크스페이스 루트가 되도록 합니다.
    
    ```
    cd ~/omy_franka_teleop/open_manipulator_omy
    ```
    
- 1.3 의존성 설치
    
    ```
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
    
    `rosdep`을 사용하는 경우:
    
    ```
    sudo rosdep init
    rosdep update
    
    rosdep install \
      --from-paths . \
      --ignore-src \
      --rosdistro humble \
      -r -y
    ```
    
    이미 `rosdep init`이 되어 있으면 해당 명령은 건너뜁니다.
    
- 1.4 빌드
    
    ```
    cd ~/omy_franka_teleop/open_manipulator_omy
    
    source /opt/ros/humble/setup.bash
    
    colcon build \
      --symlink-install \
      --event-handlers console_direct+
    ```
    
    빌드가 끝난 후 환경을 적용합니다.
    
    ```
    source install/setup.bash
    ```
    
    새 터미널을 열 때마다 다음 순서로 실행합니다.
    
    ```
    source /opt/ros/humble/setup.bash
    source ~/omy_franka_teleop/open_manipulator_omy/install/setup.bash
    ```
    
- 1.5 리더암 실행
    
    OMY-L100에는 `omx_l_leader_ai.launch.py`가 아니라 다음 launch 파일을 사용해야 합니다.
    
    ```
    ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \
      port_name:=/dev/ttyUSB0 \
      use_sim:=false \
      use_mock_hardware:=false
    ```
    
    이 상태에서는 우선 리더암의 조인트 값을 확인합니다.
    
    ```
    ros2 topic echo /leader/joint_states
    ```
    
    토픽 주기 확인:
    
    ```
    ros2 topic hz /leader/joint_states
    ```
    
    정상적으로 확인되어야 하는 조인트:
    
    ```
    joint1
    joint2
    joint3
    joint4
    joint5
    joint6
    rh_r1_joint
    ```
    

## 에러 해결
오류: FastBulkRead Rx Fail -3001

```
FastBulkRead Rx Fail [Dxl Size : 7] [Error code : -3001]
Dynamixel Read Fail
Dynamixel Write Fail
```

원인:

- `3001`은 수신 응답 timeout
- 7개 모터의 BulkRead 응답 중 일부 또는 전체를 제한 시간 안에 받지 못함
- 초기에는 U2D2 USB latency가 `16ms`인 상태였음
- 300Hz 제어 루프와 4Mbps 통신에서 USB latency가 문제가 될 수 있음

해결:

```
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

확인:

```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

결과:

```
1
```

이후 로그에서 반복적인 `FastBulkRead Rx Fail`이 사라지고 초기화 직후 1회 정도만 발생한다면 latency 문제가 해결된 것입니다.