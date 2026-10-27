# third_impact

ROS 2 Humble / Jetson Orin Nano. 기존 지도에서 정지 LiDAR 관측으로 초기 위치를 자동 탐색한 뒤 사용하는 정찰·정밀 복귀 패키지입니다. **실제 지도와 실기에서의 물리적 1cm 미만 도착은 아직 검증하지 않았습니다.** 설정의 8mm는 LiDAR 매칭 기반 추정 오차 기준이며 실제 정확도 보장이 아닙니다.

## 구성과 원칙

- `make_goal_launch.py`: 모터·조이스틱·URDF·LiDAR·IMU/EKF·AMCL, 목표 저장, 직사각형 편집. Nav2 주행은 실행하지 않습니다. 시작 모드는 수동입니다.
- `impact_launch.py`: 위 하드웨어와 AMCL, Nav2, 임무 제어를 함께 실행합니다. 시작 모드는 자동입니다. 지도·위치 추정·센서·저장 목표·구역 준비 후 즉시 정찰합니다.
- `debug_launch.py`: 관찰 전용. `log:=true`일 때만 기록 프로세스를 실행합니다. `false`일 때 별도 실행 노드가 없는 것이 정상입니다. 시각화 자료는 임무/설정 노드가 발행하므로 debug 없이도 RViz와 주행이 작동합니다.
- `laptop_rviz_launch.py`: 노트북에서만 RViz 실행. `teach:=true`에서만 Save Goal 패널·구역 편집 기능이 보입니다. 기본 debug 화면에는 명령·초기 위치 설정 도구가 없습니다.
- Cartographer는 사전 지도 제작에 사용합니다. 저장 지도 주행은 AMCL, Nav2, IMU/EKF를 사용합니다.

`all_in_one_launch`, `make_goal_launch`, `impact_launch`를 동시에 켜지 마세요. 하드웨어를 중복 실행합니다. 새 launch 두 개의 동시 실행은 로컬 프로세스 잠금으로 차단하지만 기존 all_in_one 실행을 대신 종료하지는 않습니다.

## Jetson 빌드

```bash
cd /home/moonshot/capstone_final_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --base-paths src --packages-select third_impact serial_test --symlink-install
source install/setup.bash
```

이 workspace의 `diagnostics` 안에 serial_test 참조 복사본이 있어서 **`--base-paths src`가 필요합니다.**

`serial_test` 변경은 제어 모드 발행과 opt-in 명령 타임아웃/정지 옵션입니다. 기존 launch는 기존 기본값을 사용합니다. 새 launch는 명령 미수신 0.3초 후 누적 모터 목표를 현재 엔코더 위치로 초기화하며, 0 속도 명령에서도 같은 정지 처리를 사용합니다. 속도 명령 적분 간격은 실제 수신 간격을 사용합니다.

## 지도 준비

Cartographer로 새 지도를 만들고 `third_impact_map.yaml`과 이미지 파일을 저장하세요. 지도 제작 자체는 이 패키지의 launch에 포함하지 않습니다. 저장된 지도 경로를 두 launch에 똑같이 전달합니다.

```bash
ros2 launch third_impact make_goal_launch.py map:=/absolute/path/third_impact_map.yaml
```

`map` 인자를 생략하면 설치된 `third_impact/maps/third_impact_map.yaml`을 찾습니다. 기본 위치를 쓰려면 `src/third_impact/maps/`에 지도 파일들을 넣고 다시 빌드하세요. 가짜 지도는 제공하지 않습니다. 지도 파일이 없으면 하드웨어 실행 전에 오류로 종료합니다.

지도 YAML의 내용(이미지 경로 제외)과 이미지 바이트의 해시를 목표와 함께 저장합니다. 지도 내용이 바뀌면 이전 목표를 사용할 수 없습니다. 내용이 완전히 같은 지도를 별도 작업으로 재생성한 경우에도 운영 절차상 목표를 다시 저장하세요.

## 네트워크 — 각 Jetson 터미널

```bash
cd /home/moonshot/capstone_final_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/moonshot/capstone_final_ws/config/remote_rviz/jetson_dual.xml
```

## 노트북 준비

노트북에서는 `src/third_impact` 폴더만 별도 ROS workspace의 `src/third_impact`로 복사하고 빌드하면 됩니다. Jetson의 ARM 빌드 결과를 노트북으로 복사하지 않습니다. RViz 버튼은 이 패키지의 C++ 플러그인이므로 노트북에서도 빌드해야 합니다.

필요한 빌드 패키지 예:

```bash
sudo apt install python3-colcon-common-extensions ros-humble-ament-cmake-python \
  ros-humble-ament-cmake-pytest ros-humble-rviz2 ros-humble-rviz-common \
  ros-humble-rclcpp ros-humble-std-msgs ros-humble-pluginlib qtbase5-dev
cd ~/third_impact_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select third_impact --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/remote_rviz/laptop.xml"
ros2 daemon stop
```

노트북에서 하드웨어 패키지를 실행할 필요는 없습니다. `laptop_rviz_launch`만 실행합니다.

## 1. 목표와 구역 설정

Jetson:

```bash
ros2 launch third_impact make_goal_launch.py map:=/absolute/path/third_impact_map.yaml
```

노트북:

```bash
ros2 launch third_impact laptop_rviz_launch.py teach:=true
```

1. 정지 상태에서 자동 위치 탐색·AMCL 초기화·관측 검증이 끝나기를 기다립니다. 후보가 모호하여 자동 확정되지 않으면 설정 화면의 `2D Pose Estimate`로 실제 초기 위치를 지정할 수 있습니다.
2. 기존 조이스틱으로 주차합니다. 시작은 수동이며 L2+R2를 함께 눌렀다 떼면 수동/자동 전환입니다.
3. 로봇을 완전히 멈추고 `Save Goal` 버튼을 누릅니다. 2초 동안 움직이지 마세요.
4. 지도 pose와 전후방 LiDAR 관측을 저장합니다. 흔들림·관측 부족·형상 퇴화 시 저장을 거부하고 기존 파일을 유지합니다. 새 저장이 성공하면 이전 구역은 삭제됩니다.
5. RViz의 `Interact` 도구로 직사각형 중심을 이동하고, 회전 고리를 돌리고, 모서리 구를 끌어 크기를 바꿉니다. 마우스를 놓으면 편집 값이 반영됩니다.
6. `Save Region` 버튼을 눌러 확정합니다. 편집만 하고 저장하지 않은 값은 임무에 적용되지 않습니다.
7. Jetson의 make_goal launch를 종료합니다. 새 임무 launch와 동시에 실행하지 않습니다.

설정은 기본 `~/.ros/third_impact/goal.json`에 저장합니다. `state_file:=/absolute/path/goal.json`으로 바꿀 수 있으며 양쪽 launch에 같은 경로를 전달해야 합니다. 목표 pose, 지도 해시, 기준 관측, 구역이 한 파일에 원자적으로 저장됩니다.

구역은 지도에 고정된 회전 직사각형 하나입니다. 중심은 자유롭고 goal을 반드시 포함할 필요는 없습니다. **정찰 목적지만 금지하며, 경로와 몸체의 통과는 허용**합니다. 최종 합의에 따라 debug/임무 중 구역 편집은 지원하지 않습니다.

## 2. 임무 실행

Jetson:

```bash
ros2 launch third_impact impact_launch.py map:=/absolute/path/third_impact_map.yaml
```

노트북:

```bash
ros2 launch third_impact laptop_rviz_launch.py
```

각 Jetson launch는 `localization_bootstrap`을 함께 실행합니다. 이 노드는 전후방 LiDAR와 지도 전체를 비교해 충분히 구분되는 위치 후보가 있을 때 `/initialpose`를 발행합니다. AMCL 적용 후 현재 관측이 지도와 계속 일치하는지 확인해야 임무가 시작됩니다. 저장 goal이나 마지막 위치를 현재 위치로 가정하지 않습니다. 자동 확정이 어려운 대칭/반복 공간에서는 `2D Pose Estimate`로 실제 초기 위치를 지정할 수 있습니다. debug 화면은 읽기 전용이며 자동 탐색은 RViz 없이도 작동합니다.

명령:

```bash
ros2 topic pub --once /go_to_goal std_msgs/msg/Empty '{}'
ros2 topic pub --once /go_around std_msgs/msg/Empty '{}'
ros2 topic echo /arrival_at_goal std_msgs/msg/Empty
```

- 준비되면 바로 정찰. 미지 영역·장애물 가까운 곳·구역 내부는 목적지에서 제외합니다. Nav2가 경로를 만들지 못하는 후보는 실패 후 다른 후보를 고릅니다. 정찰 목적지의 5cm 이내에 들어오면 heading과 정지 유지 여부를 따지지 않고 다음 목적지로 전환합니다. 이전 action 종료 확인은 기다리지만 성공 후 2초 재시도 지연은 없습니다. 정찰 도착은 `/arrival_at_goal`을 발행하지 않습니다.
- `/go_to_goal`: 정찰 취소 → 복귀 → 기존 Nav2 action이 끝난 것을 확인 → 최종 정렬.
- 복귀 중 `/go_around`: 무시하며 예약하지 않습니다.
- 최종 정렬: 저장 LiDAR와 현재 관측의 국소 ICP, 매칭 겹침·잔차·관측 가능성 검사, 저속 전후진·회전. 모호하면 정지 상태에서 다시 관측합니다. 데이터가 다시 유효해지면 재시도합니다.
- 정렬 중 측정 장애물에 대한 로봇 사각 footprint의 예상 이동 영역을 검사합니다. 전후방 scan, TF, odometry가 오래되면 자동 명령을 차단합니다.
- 추정 위치 오차 8mm 미만, 방향 ±5°, 실제 odometry 속도가 정지 범위에 들어온 상태를 신선한 관측으로 1초간 확인하면 Empty 도착 이벤트를 한 번 발행합니다.
- 도착 후 HOLD에서는 `/go_to_goal`을 무시하고 이벤트도 재발행하지 않습니다. 새로운 `/go_around`에서만 자율 이동을 재개합니다.
- 조이스틱 수동 전환 시 임무를 일시 중지하고 Nav2 action을 취소합니다. 자동 전환 후 이전 임무를 재개합니다. HOLD 상태에서 수동으로 이동한 뒤 자동 전환해도 HOLD를 유지합니다.
- 재시도 횟수 제한·임무 실패 종료는 없습니다. 장애물/센서 문제 때 정지해서 재확인하며 상태와 이유는 로그로만 제공합니다.

`/save_goal`, `/third_impact/save_region`은 설정 모드에서만 처리합니다. `/arrival_at_goal`은 reliable/volatile 이벤트입니다. 다른 역할을 수행할 노드는 도착 전에 구독해야 합니다. 늦게 구독한 노드에 과거 도착을 재전송하지 않습니다.

## 3. 디버그 기록

별도 Jetson 터미널:

```bash
ros2 launch third_impact debug_launch.py log:=true
```

기본 위치:

```text
/home/moonshot/capstone_final_ws/src/third_impact/debug_log/YYYYMMDD_HHMMSS_microseconds/
  bag/                 모든 발견 ROS 토픽, 숨김 action 토픽 포함
  diagnostics.jsonl    상태·오차·매칭 품질·이벤트
  rosbag.log           기록 프로세스 출력
  config/              패키지 설정
  *.yaml               시작 시 실행 중 노드 파라미터 스냅샷
  goal.json            시작 시 저장 목표/기준 관측/구역 (있는 경우)
  지도 YAML/이미지     시작 시 사용 지도 (있는 경우)
  nodes.txt, topics.txt, session.json
```

상태 변화와 변경된 목표/구역은 bag에도 기록됩니다. 설정 파일 스냅샷은 기록 시작 시점입니다. 날짜 폴더는 한국 시간입니다. bag은 1GiB 단위로 분할하고 여유 공간 2GiB 미만이면 기록만 중단합니다. 모든 토픽을 기록하므로 같은 ROS domain에 카메라 등이 있으면 기록량과 부하가 증가합니다. 기록 프로세스 실패나 debug 종료는 임무를 중단시키지 않습니다. 종료 시 Ctrl+C 후 bag 마무리를 기다리세요.

RViz 지도에는 현재 박스·방향, 저장 목표 박스·방향, 저장 구역, global/local path만 표시합니다. 동작 상태·상세 이유·정렬 오차는 왼쪽 Robot Status 패널에 표시합니다. 정렬 제어는 경로 추종이 아닌 상대 pose 제어이므로 그 단계에 Nav2 local path가 새로 생성되지는 않습니다.

## 정확도 검증과 조정

`config/impact.yaml`의 정렬 속도, 도착 판정, 매칭 품질 기준을 조정합니다. YAML을 변경한 경우 재빌드 또는 `impact_params:=/absolute/path/impact.yaml`로 실행합니다. Nav2 별도 설정은 `config/nav2.yaml`, 실행 인자는 `params_file`입니다. 기존 amr_navigator의 설정은 수정하지 않았습니다.

1. 같은 왼쪽 바퀴 축 기준점/고정 차체 표시를 바닥 기준으로 측정합니다.
2. 각 시도에서 앞뒤·좌우 오차, 가능하면 각도, 접근 방향, bag 세션 이름을 남깁니다.
3. 다른 위치·방향에서 여러 번 복귀해 성공률·오차 분포와 진동·최소 이동량을 확인합니다.
4. 실제 측정과 추정 오차가 다르면 LiDAR 장착 변환, 주변 구조, wheel/IMU 보정부터 확인합니다.

**확인된 기존 기하 불일치:** URDF 바퀴 중심 간격은 0.392m, 시각적 바퀴 지름은 0.130m인데 serial_test에는 간격 0.272m, 지름 0.170m가 사용됩니다. 실제 유효 바퀴 치수·엔코더 보정값을 모르는 상태에서 임의 변경하지 않았습니다. 1cm 검증 전에 실측해야 하는 항목입니다. 바퀴축 중앙의 평면 좌표는 URDF의 base_link 원점과 일치합니다.

왼쪽 바퀴 기준 오차와 바퀴축 중앙 오차는 방향 오차가 있을 때 다릅니다. LiDAR 매칭도 벽·기둥의 형상과 측정 편향에 영향을 받으므로 작은 추정 잔차가 물리적 정확도를 인증하지 않습니다.

## 테스트

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --base-paths src --packages-select third_impact --event-handlers console_direct+
colcon test-result --test-result-base build/third_impact --verbose
```

수치 테스트: 노이즈/이상점 포함 LiDAR 정합, 복도 퇴화, 회전 직사각형, 후진/회전 제어, 예상 footprint 충돌, 원자적 저장, 지도 변경 무효화.
상태 테스트: HOLD 불변, 복귀 우선순위, 수동 개입과 비동기 goal 수락 경쟁, 취소 완료 전 제어권 이전 금지, 센서 준비 상실.
자동 테스트는 물리적 도착 정확도나 실제 조이스틱/모터/장애물 응답을 대신하지 않습니다.

## 이 환경에서 수행한 검증 (2026-10-27)

- Jetson ARM/Humble에서 `third_impact`, `serial_test` 빌드 성공.
- 수치/상태 회귀 테스트 12개 통과. 별도 localhost domain 193에서 실제 ROS 노드 생성, 목표·직사각형 저장, 도착 이벤트 1회 발행 테스트 통과.
- offscreen Qt 환경에서 RViz 플러그인 로드와 두 버튼 생성 확인.
- 별도 domain 195에서 Nav2 서버 7개의 lifecycle configure 성공; `/plan`, `/local_plan` 실제 경로 토픽 확인. 활성화/주행은 실행하지 않음.
- 별도 domain 196에서 rosbag 기록, diagnostics JSONL, 시작 설정 스냅샷, 정상 종료 후 bag metadata 확인.
- 모터 드라이버를 생성하지 않은 상태에서 속도 콜백의 정지 초기화·시간 적분·수동 우선권 확인.
- 기존 환경의 NumPy 1.26.4 / SciPy 호환 버전 경고가 있음. 이번 수치 테스트는 통과했으나 시스템 라이브러리를 임의 변경하지 않음.
- 실제 센서·모터 주행, 노트북 마우스 조작, Wi-Fi 환경, 물리적 도착 오차와 주행 부하는 실기 검증이 남아 있음.

일반 colcon 테스트에서 실제 ROS graph 테스트는 기본 skip입니다. 격리된 graph 테스트를 명시적으로 실행하려면:

```bash
THIRD_IMPACT_ROS_TESTS=1 python3 -m pytest -q src/third_impact/test/test_runtime.py
```

이 테스트는 내부에서 ROS domain 193과 localhost를 강제하고 하드웨어를 시작하지 않습니다.

## RViz 왼쪽 Robot Status 패널

`teach.rviz`, `debug.rviz`의 첫 패널은 읽기 전용 `third_impact/StatusPanel`입니다.
정찰, 목표 복귀, 정밀 정렬, 도착 대기, 수동 조작, 위치·센서 준비 대기를 한국어로 표시합니다.
상태 수신이 2초 끊기면 이전 임무 상태 대신 연결 끊김을 표시합니다. 제어 토픽은 발행하지 않습니다.
기존 노트북 설치에도 새 `plugins.xml`, C++ 헤더/소스 및 RViz 설정을 복사하고 재빌드해야 합니다.
준비 여부 필드는 업데이트된 Jetson 노드를 다음 실행할 때부터 제공됩니다.
패널이 다른 위치에 도킹되면 `Robot Status` 제목 표시줄을 왼쪽 위로 끌어 배치한 뒤 RViz 설정을 저장할 수 있습니다.


## 정찰 도착 조건과 상태 표 (2026-10-28)

- `config/patrol.xml`은 `PatrolFollowPath`와 `patrol_goal_checker`를 선택합니다. 정찰 전용 DWB에서 최종 방향 정렬 critic을 제외하고, goal checker의 yaw 범위를 전체 각도로 설정합니다. 위치는 5cm 이내여야 합니다.
- `config/return.xml`은 기존 `FollowPath`, `goal_checker`를 선택합니다. 복귀 마지막 구간의 8mm 추정 오차·±5°·1초 정지 확인은 바뀌지 않았습니다.
- `impact.yaml`의 `patrol_tolerance`는 감독 노드와 정찰 Nav2 goal checker/controller에 함께 적용됩니다. 기본값은 0.05m입니다.
- 지도에는 글자를 발행하지 않습니다. 현재 로봇, 저장 목표, 구역은 각각 `robot`, `saved_goal`, `patrol_region` namespace를 씁니다. 새 RViz 설정은 이전 `third_impact` namespace를 숨겨 과거 실행의 텍스트도 제외합니다. 변경 적용에는 Jetson 임무 노드 재시작과 노트북 설정 업데이트가 모두 필요합니다.
- 왼쪽 `Robot Status`는 `항목 / 값 / 간단한 설명` 표를 제공합니다. 현재 임무·조작 모드·준비 여부·이동 요청·직전 결과·취소 요청·남은 직선거리·정찰 도착 조건·완료 횟수·정렬 오차·최근 안내를 표시합니다.
- `status=4`: Nav2 이동 성공. `5`: 취소 완료. `6`: Nav2 이동 실패. `purpose=patrol`: 정찰 이동. `cancellation=False`: 취소를 요청하지 않음.
- 5cm 근접 판정으로 이전 정찰 action을 종료하면 Nav2 결과는 `취소됨 (5)`일 수 있습니다. 표의 설명에 정상적인 다음 목적지 전환임을 함께 표시합니다. Nav2의 성공과 정밀 복귀의 `/arrival_at_goal`은 서로 다른 판정입니다.
- 표는 읽기 전용이며, 통신이 2초 끊기면 오래된 표 값을 지웁니다. 내용이 길면 표 내부를 스크롤하거나 패널 폭을 넓혀 확인할 수 있습니다.


## 재시작·전원 OFF 중 위치 이동 대응

`third_impact_localization`은 로봇을 회전시키거나 이동시키지 않고 정지 관측으로 초기 위치를 찾습니다.

1. `/map`, 전후방 필터된 scan, 센서 TF, `/odom`, AMCL 활성화를 기다립니다.
2. 지도 전체의 이동 가능한 후보 위치·방향을 검색하고, 상위 후보를 LiDAR 정합으로 보정합니다.
3. 지도 끝점 일치, 광선 중간의 장애물 통과 여부, 로봇 footprint, 형상 관측 가능성, 대안 후보와의 차이를 검사합니다.
4. 탐색 중 로봇이 움직였거나 현재 관측과 달라졌으면 결과를 버리고 다시 찾습니다.
5. 후보가 충분히 구분될 때만 `/initialpose`로 AMCL을 초기화합니다. 필요하면 `/request_nomotion_update`를 요청해 정지 상태에서도 관측을 갱신합니다.
6. AMCL의 새 위치와 현재 LiDAR가 1초 이상/3회 이상 연속 일치하는지 확인한 뒤 준비 완료를 발행합니다.
7. 준비 완료가 없거나 2초 이상 끊기면 임무 노드는 자율 속도 출력을 차단합니다. 조이스틱 수동 모드는 유지합니다.

마지막 검증 위치는 기본 `~/.ros/third_impact/goal.localization.json`에 저장됩니다. 다른 `state_file`을 쓰면 같은 파일명에 `.localization.json` 확장자를 사용합니다. 이 정보는 검색 후보에 추가할 뿐이며, 이전 위치가 틀려도 전체 지도 검색을 생략하지 않습니다. 지도 내용이 바뀌면 이전 후보는 무시됩니다.

`/third_impact/localization` 토픽과 Robot Status 표의 `초기 위치` 행에서 다음을 구분합니다:

| 상태 | 의미 |
| --- | --- |
| 자동 위치 탐색 중 | 현재 관측을 지도 전체와 비교 중 |
| AMCL 자동 초기화 중 | 찾은 후보를 AMCL에 전달함 |
| 위치 후보 검증 중 | 새 AMCL 결과와 현재 관측의 일치 여부 확인 중 |
| 위치 확인됨 | 위치 검증 완료; 다른 임무 준비 조건도 만족하면 출발 |
| 위치 후보가 여러 개 | 모양이 비슷한 후보를 구분하지 못해 주행 보류 |
| LiDAR·odometry 대기 | 센서 데이터/센서 TF/odometry 부족 또는 오래됨 |
| 정지 대기 | 움직이는 동안에는 자동 전역 초기화를 하지 않음 |

이는 추가 센서 없이 초기화 절차를 자동화하는 기능입니다. 대칭 공간, 변화한 지도, 가려진 LiDAR 등에서 위치가 유일하게 정해진다는 보장은 없으며, 그 경우 임의 초기화 대신 재시도/수동 초기 위치 지정을 기다립니다. 이동 제어·장애물 회피·정밀 도착 검증을 대신하지 않습니다.

검증: 잘못된 이전 위치 후보가 있어도 옮겨진 위치를 찾는 합성 관측, 대칭 공간 거부, 탐색 중 이동/지도 변경/현재 관측 불일치 거부, 초기 위치 발행 후 검증 전 주행 준비 미승인 테스트를 수행했습니다. 저장된 실제 지도와 목표 기준 관측의 오프라인 검색에서는 약 0.6초에 후보 하나를 찾았습니다. 이 결과는 현재 실시간 위치나 물리적 주행 정확도 검증 결과가 아닙니다.
