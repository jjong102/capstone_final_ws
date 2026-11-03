# third_impact

ROS 2 Humble / Jetson Orin Nano. 기존 지도에서 정지 LiDAR 관측으로 초기 위치를 자동 탐색한 뒤 사용하는 정찰·정밀 복귀 패키지입니다. **실제 지도와 실기에서의 물리적 1cm 미만 도착은 아직 검증하지 않았습니다.** 설정의 2cm는 LiDAR 매칭 기반 추정 오차 기준이며 실제 정확도 보장이 아닙니다.

## 구성과 원칙

- `make_goal_launch.py`: 모터·조이스틱·URDF·LiDAR·IMU/EKF·AMCL, 목표 저장, 직사각형 편집. Nav2 주행은 실행하지 않습니다. 시작 모드는 수동입니다.
- `impact_launch.py`: 위 하드웨어와 AMCL, Nav2, 임무 제어를 함께 실행합니다. 시작 모드는 자동입니다. 지도·위치 추정·센서·저장 목표·구역 준비 후 즉시 정찰합니다.
- `impact_launch.py log:=true`: 임무와 함께 bag·진단 자료 기록을 실행합니다. 기본값은 `log:=false`이며 기록 없이도 RViz와 주행이 작동합니다.
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
6. 파랑은 `Save Boundary`, 주황은 `Save Region` 버튼을 눌러 각각 확정합니다. 편집만 하고 저장하지 않은 값은 임무에 적용되지 않습니다.
7. Jetson의 make_goal launch를 종료합니다. 새 임무 launch와 동시에 실행하지 않습니다.

설정은 기본 `~/.ros/third_impact/goal.json`에 저장합니다. `state_file:=/absolute/path/goal.json`으로 바꿀 수 있으며 양쪽 launch에 같은 경로를 전달해야 합니다. 목표 pose, 지도 해시, 기준 관측, 구역이 한 파일에 원자적으로 저장됩니다.

구간은 각각 독립적으로 이동·회전·크기 조절하는 직사각형 두 개입니다.

- **파랑: 한계 구간** — 정찰과 자동 복귀 모두 이 안에서만 이동합니다. 복귀 목표와 몸체 여유를 포함해야 저장할 수 있습니다. `Save Boundary — 한계 구간 저장`으로 확정합니다.
- **주황: 정찰 제한 구간** — 정찰 목적지와 이동 경로 모두 진입 금지입니다. 목표 복귀할 때는 진입을 허용합니다. `Save Region — 정찰 제한 구간 저장`으로 확정합니다.

`make_goal_launch.py`에서 Save Goal → Interact로 파란 한계 구간 편집·저장 → 주황 제한 구간 편집·저장 순서로 설정합니다. 각 저장 버튼은 해당 구간만 저장합니다. 목표를 새로 저장하면 두 구간 모두 다시 설정해야 합니다. 이전 단일 구역 파일은 의미를 자동 변환하지 않고 두 구간 재저장을 요구합니다. PGM/YAML을 칠할 필요는 없습니다.

Nav2 지역 costmap은 `/third_impact/navigation_map`, 전역 경로 계획은 `/third_impact/planning_map`을 사용합니다. 전역 지도는 몸체 외접원+5cm 검사와 추가 10cm 여유에서 costmap의 내접원 반경을 제외한 만큼 가상 구간을 확장합니다. 경계 근처에서 이미 정지한 시작점은 추가 여유만 0.8m에 걸쳐 점진 적용하며 원래 금지 구간과 실제 장애물은 지우지 않습니다. 한계 밖은 항상 장애물이며, 주황 구간은 정찰 중에만 장애물입니다. AMCL과 LiDAR 검증은 수정하지 않은 `/map`을 사용합니다. 구간이 빠져 있으면 경로 계획용 지도를 전부 막고 자동 이동을 대기합니다. 경로 계획과 별개로 속도 명령에도 몸체 외접원+5cm 및 예상 이동 거리 검사를 적용합니다. 위치 추정 오차와 실제 제동 성능에 의존하므로 물리적 울타리를 대신하지 않습니다.

구간 검사는 명령의 직진·후진·회전 궤적을 따라 몸체 여유를 확인합니다. 전진 감속은 회전 속도와 분리합니다. 이동이 차단되거나 12초간 20cm 이상 위치 진행이 없으면 기존 action 취소 완료와 odometry 정지를 확인한 후, 최근 같은 목표 경로의 안전한 방향으로 제자리 정렬합니다. 회전도 구간·LiDAR 장애물 검사를 통과해야 하며 회전각에 따른 제한 시간과 진행 감시를 적용합니다. 정렬 후 완전히 정지한 다음 같은 목표로 재개합니다.

같은 지점의 두 번째 차단부터는 계획 여유를 15cm 더 넓혀 같은 목표를 재계획합니다. 네 번째 차단 또는 정렬 시간 초과는 `PATROL_BLOCKED`로 정지 대기하며 새 목표를 반복 생성하지 않습니다. 수동으로 안전한 위치로 옮긴 뒤 `/go_around` 정찰 재개 명령으로 대기 상태를 해제할 수 있습니다. 목표 복귀 명령은 정렬·대기를 선점합니다. 구간 저장과 PGM은 바꾸지 않습니다.

velocity_smoother는 직전 출력 기반 `OPEN_LOOP`입니다. 20261102_220930 bag에서 CLOSED_LOOP 출력이 반대 회전 요청 후 약 4초 동안 기존 부호로 남았습니다. 가감속 제한(회전 0.8 rad/s²)은 유지하며 실제 odom은 정지·추종 확인에 사용합니다. 정찰 중 반대 회전 요청은 실제 정지를 확인한 뒤 허용합니다. 실제 구동 응답 개선 여부는 저속 실기 시험이 필요합니다.

정찰 BT의 고정 Spin/BackUp 대신 supervisor가 실패 결과를 받아 같은 목표를 보존합니다. 취소 완료·실제 정지 후 최근 경로 방향 또는 양쪽 90도 후보의 전체 회전 공간을 확인하여 제자리 복구 회전합니다. 회전 불가 또는 반복 실패는 이유를 표시하고 정지합니다. 복귀 BT/최종 정렬 로직은 유지합니다.

`log:=true`는 `/third_impact/drive_debug`도 bag과 diagnostics.jsonl에 기록합니다. 명령, 실제/목표 엔코더, 추종 오차, stop_reset/wheel_reversal_reset 사유가 포함됩니다. supervised 모터 제어는 정지 및 바퀴 방향 전환 시 이전 누적 목표를 제거하며, 연속 주행 중에는 누적을 유지합니다. 원래 Nav 명령·평활화 명령·최종 명령·odom은 bag에서 함께 비교할 수 있습니다.

수동 조이스틱 조작에는 두 구간을 적용하지 않습니다. 한계 밖에 있거나 정찰 제한 구간 안에서 정찰을 시작하면 자동으로 탈출하지 않고 정지합니다. 수동으로 파란 구간 안·주황 구간 밖으로 옮긴 뒤 정찰을 재개하세요. 복귀는 주황 구간 안에서도 가능합니다. debug/임무 중 편집은 지원하지 않습니다.

### 정찰 목표의 근접 접근과 복구

정찰 중 목표 위치 40cm 이내에서는 기존 Nav2 action 취소 완료와 odometry 정지를 확인한 후 위치 전용 근접 제어를 사용합니다. 목표의 yaw는 읽지 않습니다. 목표 위치로 이동하기 위한 제자리 방향 전환과 직진을 분리하고, 회전 방향 변경·전진 전환 전에 실제 정지를 확인합니다. 직진은 최대 0.10m/s이며 남은 거리와 제동 여유에 따라 감속합니다. 추정 위치가 목표 5cm 이내이면 방향·정지 유지 조건 없이 다음 정찰 목표를 선택합니다. 65cm 밖으로 이탈하면 정지 후 같은 목표의 Nav2 경로를 재요청합니다.

근접 제어에서도 70×44cm footprint, 두 구간, 지도, LiDAR 충돌 검사를 유지합니다. 복귀 목표의 기존 경로·후진·최종 heading 정렬은 변경하지 않습니다. 복구/근접 회전의 제한 시간은 필요한 회전각÷0.25rad/s+6초로 계산하며, 4초간 0.04rad 이상 각도 오차가 줄지 않으면 별도의 진행 정체 사유를 기록합니다. 근접 전진은 6초간 1.5cm 이상 거리 개선이 없으면 대기합니다. 근접 전체 동작은 최대 60초입니다. 시간 초과를 제한 구간 충돌로 표시하지 않습니다.

### Oscillation 내부 진단 (`log:=true`)

정찰 DWB의 `Oscillation`은 기존 Humble 정책과 동등한 `third_impact::LoggedOscillationCritic`을 사용합니다. `log:=true`로 실행하면 `/third_impact/oscillation` JSON 메시지를 rosbag과 `diagnostics.jsonl` 양쪽에 저장합니다. `log:=false`에서는 내부 진단 메시지를 발행하지 않습니다. 복귀용 FollowPath의 critic은 기존 설정을 유지합니다.

각 제어 주기의 현재 pose/프레임, 제한 기준 pose/시각, 기준 이후 거리·각도·시간, 각 해제 임계값/충족 여부, 선택 명령과 측정 속도, 방향 플래그(명령 전·해제 검사 시·처리 후), 후보·거부 수를 기록합니다. `reset_reason`은 `distance`, `angle`, `time`, `controller_reset`, `none` 중 하나입니다. 여러 조건이 동시에 충족되면 모두 boolean으로 기록하고 실제 판정 순서인 거리→각도→시간의 첫 조건을 사유로 남깁니다. 각도 차이는 원본 critic과 동일하게 정규화하지 않은 차이입니다. controller reset은 plan/lifecycle에서 호출된 reset이며 거리 해제와 구분됩니다.

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

- 준비되면 바로 정찰. 미지 영역·장애물 가까운 곳·한계 구간 외부·정찰 제한 구간 내부 및 경계 여유가 부족한 곳은 목적지에서 제외합니다. Nav2가 경로를 만들지 못하는 후보는 실패 후 다른 후보를 고릅니다. 정찰 목적지의 5cm 이내에 들어오면 heading과 정지 유지 여부를 따지지 않고 다음 목적지로 전환합니다. 이전 action 종료 확인은 기다리지만 성공 후 2초 재시도 지연은 없습니다. 정찰 도착은 `/arrival_at_goal`을 발행하지 않습니다.
- `/go_to_goal`: 정찰 취소 → 복귀 → 기존 Nav2 action이 끝난 것을 확인 → 최종 정렬.
- 복귀 중 `/go_around`: 무시하며 예약하지 않습니다.
- 최종 정렬: 저장 LiDAR와 현재 관측의 국소 ICP, 매칭 겹침·잔차·관측 가능성 검사, 저속 전후진·회전. 모호하면 정지 상태에서 다시 관측합니다. 데이터가 다시 유효해지면 재시도합니다.
- 정렬 중 측정 장애물에 대한 로봇 사각 footprint의 예상 이동 영역을 검사합니다. 전후방 scan, TF, odometry가 오래되면 자동 명령을 차단합니다.
- 추정 위치 오차 2cm 미만, 방향 ±5°, 실제 odometry 속도가 정지 범위에 들어온 상태를 신선한 관측으로 1초간 확인하면 Empty 도착 이벤트를 한 번 발행합니다.
- 도착 후 HOLD에서는 `/go_to_goal`을 무시하고 이벤트도 재발행하지 않습니다. 새로운 `/go_around`에서만 자율 이동을 재개합니다.
- 조이스틱 수동 전환 시 임무를 일시 중지하고 Nav2 action을 취소합니다. 자동 전환 후 이전 임무를 재개합니다. HOLD 상태에서 수동으로 이동한 뒤 자동 전환해도 HOLD를 유지합니다.
- 재시도 횟수 제한·임무 실패 종료는 없습니다. 장애물/센서 문제 때 정지해서 재확인하며 상태와 이유는 로그로만 제공합니다.

`/save_goal`, `/third_impact/save_region`은 설정 모드에서만 처리합니다. `/arrival_at_goal`은 reliable/volatile 이벤트입니다. 다른 역할을 수행할 노드는 도착 전에 구독해야 합니다. 늦게 구독한 노드에 과거 도착을 재전송하지 않습니다.

## 3. 디버그 기록

Jetson에서 임무를 시작할 때 기록 옵션을 함께 지정합니다(기본값 `log:=false`). 이미 실행 중인 임무와 중복 실행하지 마세요:

```bash
ros2 launch third_impact impact_launch.py log:=true
```

`log_dir:=/원하는/경로`로 저장 위치를 변경할 수 있습니다. 기본 위치:

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

상태 변화와 변경된 목표/구역은 bag에도 기록됩니다. 설정 파일 스냅샷은 기록 시작 시점입니다. 날짜 폴더는 한국 시간입니다. bag은 1GiB 단위로 분할하고 여유 공간 2GiB 미만이면 bag 기록만 중단합니다. 모든 토픽을 기록하므로 같은 ROS domain에 카메라 등이 있으면 기록량과 부하가 증가합니다. 기록 프로세스 실패는 임무를 중단시키지 않습니다. impact launch를 Ctrl+C로 종료하면 임무와 기록이 함께 종료되므로 bag 마무리를 기다리세요.

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
- `config/return.xml`은 기존 `FollowPath`, `goal_checker`를 선택합니다. 복귀 마지막 구간의 2cm 추정 오차·±5°·1초 정지 확인은 바뀌지 않았습니다.
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

### 전진 우선 복귀와 후진 진입

`ros2 launch third_impact impact_launch.py`의 `/go_to_goal` 복귀는 이제
Nav2 `ComputePathToPose`로 계산한 경로를 임무 노드가 직접 추종합니다.
정찰은 기존 Nav2 제어기를 사용합니다. 복귀 중에는 NavigateToPose의
주기적 재계획, Spin, BackUp 복구 동작이 실행되지 않습니다.

- 목표에서 진입 방향→저장 heading 회전이 가능하면 전진 접근합니다.
- 불가능하면 기존 경로의 목표 앞쪽 직선 진입축 부근에서 가장 늦게 회전할 수
  있는 지점을 찾습니다. 직사각형 몸체 전체의 회전·후진 통과를 검사합니다.
- 전환 지점이 없으면 기존 경로를 재검사하며 정지합니다. 별도 우회는 만들지 않습니다.
- 전진 경로가 LiDAR/지도 검사에서 막히면 정지하고 5초 뒤 재계획합니다.
  후진 및 진입 방향 정렬 중에는 시간 제한 없이 대기하며 우회하지 않습니다.
- 후진은 전진과 같은 최대 속도 0.4 m/s, 최대 각속도 0.35 rad/s를 사용합니다.
  가감속, 정지거리 검사, 곡선/목표 접근 감속은 유지합니다.
- 목표 6cm 이내에서 최종 LiDAR 정밀 정렬로 넘깁니다. 도착 판정/알림/대기 정책은 동일합니다.
- 수동 개입으로 후진 경로에서 크게 벗어나면 자동 탈출하지 않고 정지합니다.
- `/third_impact/return_path`(보라색), `/third_impact/return_local_path`(청록색),
  진입 준비 위치 화살표와 왼쪽 상태표로 동작을 확인합니다. 로그는 `log:=true`로 기록합니다.

`config/impact.yaml`의 `return_max_speed`, `return_max_angular_speed`,
`return_obstacle_margin`으로 복귀 제한을 설정합니다. 여유 간격 기본값 1cm는
센서/지도 불확실성에 대한 작은 보정이며 실제 충돌 방지나 물리 위치 정확도의 보증은 아닙니다.
목표 앞 직선 공간이 있더라도 기존 글로벌 경로가 진입축과 연결되지 않으면 정지할 수 있습니다.
새 추종기는 기하/상태 전환 및 간단한 운동 시뮬레이션으로 검사했으며 실제 로봇 검증이 필요합니다.


복귀 벽 앞 접근: 목표까지 거리 기반으로 접근 속도를 낮추고, 0.2초 반응 지연 후 선속도 0.5m/s²·각속도 0.8rad/s²로 정지하는 궤적을 검사합니다. 실제 odom의 제동 궤적을 먼저 검사하므로 저속 요청으로 현재의 긴 제동 거리를 숨기지 않습니다. 전진 요청이 막히면 1/2·1/4 속도를 재검사하며 후진은 기존 해소 대기 정책을 유지합니다. `status.return.collision_check`에 요청/실제/선택 속도, 남은 거리, LiDAR/지도/한계/센서 사유, 충돌 예상 시간·위치와 거부한 후보를 기록합니다 (`log:=true`의 bag 및 diagnostics.jsonl에 포함). 제동 상수는 실기에서 확인해야 하며, 목표점에서 궤적 검사를 잘라내거나 외곽을 축소하지 않습니다.

20261102_223646 bag의 22:39:29.9/35.9/41/55 정지 지점에서 TF·스캔·지도·경로·odom을 재구성한 단일 판정 검증: 목표 거리 약 0.484m, 기존 요청 약 0.58m/s → 접근 요청 약 0.318m/s, 새 제동 검사 통과. 실제 폐루프 주행 전체를 재현한 결과는 아닙니다.

센서 준비 상태 진단(20261102_225004): bag에서 복귀 중 두 스캔 최대 간격은 약 112ms, odom 최대 간격은 약 32ms였으나 supervisor ready가 반복 해제되었습니다. 기존 로그에는 개별 실패 조건이 없어 내부 원인은 확정할 수 없습니다. 복귀 제동 검사 벡터화(동일 footprint/샘플 유지), 동일 스캔 cloud 캐시, depth=1 BEST_EFFORT 최신 센서 구독으로 처리 부담과 backlog를 줄였습니다. 같은 기록 지점의 단독 return_command 벤치마크는 약 40~48ms에서 10~12ms로 감소했습니다. 전체 제어 루프나 실기 지연 보장은 아닙니다.

`/third_impact/status`의 `health`에는 실패 reason, odom/manual/localization age, accepted scan age, scan_pair_delta, 수신 stamp age 및 TF 예외가 포함됩니다. `timing.tick_ms/max_tick_ms`는 감독 제어 콜백 시간입니다. `log:=true`의 diagnostics.jsonl/bag에 함께 저장됩니다. scan 0.45s / odom 0.3s / scan pair 0.15s 제한은 유지합니다.


복귀 정렬은 AMCL/TF 기준 제자리 큰 방향 정렬(실제 정지, 지도·LiDAR 전체 회전 외곽 검사, 각도 진행 감시) → 새 스캔 3회 연속 정합 확인 → 기존 정밀 위치/방향 정렬 순서입니다. 공간/진행 문제는 정지 상태로 표시하며 수동 개입 후 다시 확인합니다. 정합 보정 20cm·15도 한계는 유지하고, 거부가 5초 지속되면 2초 정지 후 새 관측으로 재검증합니다. 목표나 AMCL 위치를 자동으로 덮어쓰지 않습니다. `status.alignment`와 RViz 표에는 단계·위치 보정량/허용치·거부 이유를 표시합니다.

20261102_231751 기록의 23:19:54 스캔 비교: 저장 기준 스캔의 지도 장애물까지 거리 중앙값 1.41cm, 10cm 내 비율 89.6%; 현재 스캔 AMCL 배치 1.61cm/83.2%; ICP 배치 1.72cm/86.7%. 초기값을 ±10cm 바꿔도 ICP는 기준 프레임 x≈25~26cm로 수렴했습니다. 둘 다 지도에 어느 정도 맞으므로 기준 스캔 손상 또는 AMCL 오차 중 하나로 확정할 수 없습니다. 작은 ICP 잔차만으로 22cm 보정을 허용하지 않으며, 큰 회전 후의 새 관측으로 실기 재검증해야 합니다.


2026-11-03 속도 조정: 정찰 DWB/velocity_smoother와 복귀 최대 병진 속도를 0.4m/s로 통일했습니다. 복귀 전진의 방향 오차 감속은 raised cosine으로 0°/15°/30°/45°에서 100%/75%/25%/0%입니다. 기존 17°에서 갑자기 전진을 끊는 조건을 전진에서 제거했으며, 경로 lookahead·각속도/가감속 제한·후진 방향 정책·2cm 도착 기준은 유지합니다. 정찰 DWB의 경로 평가 방식은 그대로이며 최고 속도만 낮췄습니다.
오프라인 검증 결과는 `diagnostics/speed_20261103/replay.py`와 `replay.json`에 저장했습니다. 20261102_234102 bag 복귀 187개 기록 위치에서 실제 odom 제동 거리, 지도·LiDAR·한계 구간 검사를 통과했고 최대 새 요청은 0.39999m/s였습니다. 전진 0 요청은 기존 122개, 새 경로 인덱스 재구성에서는 58개였습니다. 수정 후 폐루프 실기 궤적을 재현한 것은 아니며 정찰은 설정·회귀 테스트로 속도 상한을 검증했습니다.

노트북 `laptop_rviz_launch.py`는 RViz 프로세스에 `ROS_DOMAIN_ID=3`, `ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=file://$HOME/remote_rviz/laptop.xml`을 자동 적용합니다. 기존 동작하던 laptop.xml을 사용하며 IP/인터페이스는 변경하지 않습니다. `domain_id:=3 dds_config:=/absolute/path/laptop.xml`로 경로·도메인을 지정할 수 있습니다. 파일이 없으면 안내와 함께 실행을 중단합니다. RViz는 ROS CLI daemon을 사용하지 않으므로 launch에서 daemon을 종료하지 않습니다. 이 설정은 부모 셸이나 별도 ROS CLI 터미널을 바꾸지 않습니다. `ros2`와 패키지 검색을 위한 `/opt/ros/humble/setup.bash`, workspace `install/setup.bash` source는 launch 실행 전에 필요합니다.

### 전·후방 통합 AMCL

`impact_launch.py`와 `make_goal_launch.py`는 `amcl_scan` 노드를 함께 실행합니다.
AMCL은 `/third_impact/amcl_scan`을 구독합니다. 입력은 기존 전·후방
`/rplidar1/scan_filtered`, `/rplidar2/scan_filtered`입니다.

센서 위치와 장착 방향은 `src/amr/description/amr_description.urdf`에서 발행한
TF를 사용합니다. 센서 오프셋을 별도로 하드코딩하지 않으며 pitch/yaw를 포함한
3차원 회전을 적용합니다. 두 관측 시점의 차이는 odom TF로 보정한 뒤 base_link
중심의 720개 각도 구간으로 합칩니다. 이 보정은 스캔 간 시간차 보정이며,
한 바퀴 스캔 내부의 개별 빔 시간차 보정은 아닙니다.

입력 시간차 0.15초 초과, 관측 나이 0.45초 초과, TF 누락 또는 센서별 유효점
10개 미만이면 통합 스캔을 발행하지 않습니다. 단일 센서로 자동 전환하지 않습니다.
기존 70×44cm 외곽 안의 자기 반사점은 제외합니다. 통합 스캔은 AMCL 전용이며,
장애물 감지·costmap·복귀 정밀 정합은 기존 센서별 입력을 유지합니다.

`/third_impact/amcl_scan_status`에는 발행 여부, 대기/거부 사유, 각 센서의 나이,
시간차와 유효점 수가 나옵니다. `ready`는 해당 타이머 회차에서 발행했는지를 뜻하며
위치 추정 준비 상태를 뜻하지 않습니다. `log:=true`에서 bag과 diagnostics.jsonl에
기록됩니다. 사용자 지정 Nav2 params 파일을 쓰면 AMCL의 `scan_topic`도
`/third_impact/amcl_scan`으로 설정해야 합니다.

### 주행 중 위치 검증과 정지 재검증

지도 일치율은 장애물 내부가 아닌 벽 경계까지의 거리로 계산합니다.
`localization.quality`에는 전체 및 `sensors.front/rear`별 벽 경계 일치율,
경계 오차, 광선 자유 공간 비율과 실패 사유를 기록합니다. 센서별 광선 검사는
실제 센서 원점에서 시작합니다. RViz Robot Status 패널에 검증 거부 사유와
전·후방 진단이 표시되며 `log:=true`의 diagnostics.jsonl에도 기록됩니다.

전·후방 관측은 odom TF로 같은 시각에 맞추고 해당 시각의 map->base_link로
검증합니다. 주행 중에도 5Hz로 검사하며, 서로 다른 관측에서 3회 이상,
0.4초 이상 연속 불일치가 지속되면 ready를 내리고 정지를 요청합니다.
정지 후 최대 6초간 현재 AMCL 입자 분포에서 nomotion 관측 갱신을 요청합니다.
3회 이상·1초 이상의 연속 검증 통과 후에만 READY로 복구합니다.
실패가 계속되면 기존 전체 지도 탐색으로 넘어갑니다. 국소 재검증은 별도의
임의 초기 위치 주입이나 로봇 회전을 수행하지 않습니다.

차체 검증은 URDF 기준 70×44cm 외곽 전체를 샘플링합니다. 광선 자유 공간
90%, 정찰 도착 5cm, 한계 구간 및 정찰 제한 구간은 유지됩니다.

전·후방 벽 경계 일치율 통과 기준은 각각 **전방 70%, 후방 50%**입니다.
전체 혼합 일치율 70%로 후방 완화를 다시 차단하지 않고, 센서별 기준을
주행 검증 및 전체 탐색 후보의 최종 검증에 적용합니다. 기존 전체 경계 오차
6.5cm, 광선 자유 공간 90%, 차체 외곽 조건은 그대로 유지합니다.

### 복구 회전의 센서 대기

복구 회전 후보는 한 관측 스냅샷으로 평가하고 종료 시 센서 유효성을 재확인합니다.
센서가 없거나 오래된 경우 `SENSOR_WAIT`으로 정지하며, 전·후방 모두 새로운
스캔이 도착한 경우에만 최대 3회·최초 대기부터 5초 이내에 재검사합니다.
한도를 넘으면 센서 대기 한도 초과로 표시하며 충돌로 표시하지 않습니다.
실제 후보 거부 사유는 `lidar_collision`, `map_collision`, `region_boundary`로
`status.patrol_control.recovery_check`에 기록합니다. 차체와 구간 여유는 유지합니다.
수동·위치·센서 준비 대기 시간은 정찰의 12초 진행 정체 타이머에서 제외하며,
대기 이전에 누적된 실제 주행 정체 시간은 유지합니다.

### Network selection

`impact_launch.py` and `make_goal_launch.py` accept `network:=wifi` (default,
robot Wi-Fi to laptop) or `network:=jetson` (robot eno1 to wired UI Jetson).
These use the same single-interface profiles as mapping. Stop the existing
launch before switching modes. All children inherit the selected DDS environment.
A nonempty `dds_config:=/absolute/path.xml` overrides the selected profile;
`domain_id` defaults to 3. The startup log prints the effective XML path.

```bash
ros2 launch third_impact impact_launch.py network:=wifi log:=true
ros2 launch third_impact impact_launch.py network:=jetson log:=true
ros2 launch third_impact make_goal_launch.py network:=wifi
```

The dual-interface XML is no longer selected by default. Receiver configurations
and URDF are unchanged; remote reception must be checked on the target machine.
