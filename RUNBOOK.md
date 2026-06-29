# Waypoint Manager Runbook

ROS2 Foxy, SSH, real robot 기준 실행 절차입니다.

`waypoint_manager`는 로봇 위치를 직접 판단하지 않습니다. diagnostics를 받아 현재 logical vertex에서 목표 vertex로 가는 leg YAML을 고르고, active waypoint 하나를 `/waypoint_manager/target_waypoint`로 1회 발행합니다. 아래 layer가 완료 신호를 보내면 다음 waypoint를 1회 발행합니다. 실제 주행, 정지, 장애물 회피는 `autonomy_stack_go2` local planner가 담당합니다.

## 구조 요약

시작할 때 launch 파라미터로 두 기준을 정합니다.

| 파라미터 | 의미 | 사용처 |
|----------|------|--------|
| `home_vertex` | 시작 시 manager가 믿는 현재 logical vertex | 어떤 leg YAML을 읽을지 결정 |
| `odometry_origin_vertex` | autonomy stack이 local odometry 원점으로 삼은 vertex | waypoint 좌표 보정 |

예를 들어 `home_vertex:=1`, `odometry_origin_vertex:=1`로 시작한 뒤 `device2` 명령이 들어오면:

```text
device2 non-zero
  -> target vertex = 2
  -> route = [1, 2]
  -> config/legs/1_to_2.yaml 로드
  -> 첫 waypoint 1회 발행
  -> /waypoint_manager/waypoint_reached data: 1 대기
  -> 다음 waypoint 1회 발행
  -> 마지막 waypoint 완료 후 current_vertex = 2
```

좌표는 YAML의 map-frame 절대 좌표에서 `odometry_origin_vertex`의 좌표를 빼서 발행합니다.

```text
published_x = yaml_waypoint_x - vertices[odometry_origin_vertex].x
published_y = yaml_waypoint_y - vertices[odometry_origin_vertex].y
```

정상 운용에서는 명령을 보낼 때마다 `set_current_vertex`나 `set_odometry_origin`을 다시 보낼 필요가 없습니다. 중간에 target을 취소하거나 실제 로봇 위치와 manager 상태가 어긋난 경우에만 수동 보정용으로 사용합니다.

## 1. Waypoint 저장

RViz에서 waypoint를 찍어 leg YAML로 저장할 때:

```bash
cd ~/autonomy_stack_go2
source install/setup.bash

./tools/record_waypoints.py \
  --output Waypoint_Manager/config/legs/1_to_2.yaml \
  --format leg-yaml
```

저장할 구간에 맞게 파일명만 바꿉니다.

```text
Waypoint_Manager/config/legs/1_to_2.yaml
Waypoint_Manager/config/legs/2_to_3.yaml
Waypoint_Manager/config/legs/3_to_4.yaml
```

leg 파일에는 중간 waypoint를 여러 개 넣을 수 있습니다. `waypoint_manager`는 한 번에 하나의 `PointStamped` target만 1회 발행하고, 아래 layer가 완료 신호를 보내면 다음 waypoint를 1회 발행합니다.

예시:

```yaml
frame_id: map
waypoints:
- x: 3.398365020751953
  y: 0.7653948664665222
  z: -0.002190561033785343
  yaw: 0.0
- x: 4.120000000000000
  y: 0.920000000000000
  z: -0.002190561033785343
  yaw: 0.0
```

저장 확인:

```bash
cat Waypoint_Manager/config/legs/1_to_2.yaml
```

## 2. 코드 저장

파일 수정 후 `Waypoint_Manager` repo에 커밋:

```bash
cd ~/autonomy_stack_go2/Waypoint_Manager
git status
git add README.md RUNBOOK.md config/vertices.yaml launch/waypoint_manager.launch.py waypoint_manager/waypoint_manager_node.py
git commit -m "Update waypoint manager runbook"
```

이미 커밋할 내용이 없으면 `nothing to commit`이 나올 수 있습니다.

GitHub에 올리기:

```bash
git push origin main
```

다른 로봇/PC에서 최신 코드 받기:

```bash
cd ~/autonomy_stack_go2/Waypoint_Manager
git pull origin main
```

## 3. 빌드

```bash
cd ~/autonomy_stack_go2
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select waypoint_manager
source install/setup.bash
```

YAML만 수정했고 launch 때 `legs_dir`를 소스 경로로 직접 지정할 경우에는 재빌드 없이도 읽을 수 있습니다. Python 코드나 launch 파일을 수정했으면 다시 빌드하세요.

## 4. 실행

터미널 4개를 사용합니다.

### 터미널 1: autonomy stack

```bash
cd ~/autonomy_stack_go2
source install/setup.bash
./system_real_robot.sh
```

### 터미널 2: waypoint_manager

```bash
cd ~/autonomy_stack_go2
source install/setup.bash

ros2 launch waypoint_manager waypoint_manager.launch.py \
  legs_dir:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/legs \
  vertices_file:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/vertices.yaml \
  home_vertex:=1 \
  odometry_origin_vertex:=1
```

정상 로그:

```text
Waypoint manager ready
```

### 터미널 3: runner

```bash
cd ~/autonomy_stack_go2
source install/setup.bash
./tools/run_relative_waypoint_sequence.py --joy-speed-axis 0.35
```

벽 근처에서 빠르면 낮춥니다.

```bash
./tools/run_relative_waypoint_sequence.py --joy-speed-axis 0.25
```

정상 로그:

```text
manager waypoint in : /waypoint_manager/target_waypoint
robot waypoint out  : /way_point
joy out             : /joy
```

### 터미널 4: 명령 전송

기존 target 제거:

```bash
cd ~/autonomy_stack_go2
source install/setup.bash
ros2 service call /waypoint_manager/cancel std_srvs/srv/Trigger {}
```

`home_vertex`를 launch에서 맞췄다면 명령을 보낼 때마다 현재 논리 위치를 다시 보낼 필요는 없습니다.

1번에서 2번으로 보내기:

```bash
ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: [{name: device2, level: [1], message: '', hardware_id: '', values: []}]}"
```

Foxy에서는 반드시 `level: [1]`을 사용합니다. `level: 1`로 보내면 OK로 들어갈 수 있습니다.

정상 로그:

```text
# 터미널 2
Loaded N waypoint(s) for hop 1 -> 2
Commanding hop: 1 -> 2

# 터미널 3
new manager waypoint: frame=map, x=..., y=..., z=...
```

아래 layer가 현재 waypoint에 도착하면 완료 신호를 보냅니다. 수동 테스트는 이렇게 할 수 있습니다.

```bash
ros2 topic pub --once /waypoint_manager/waypoint_reached std_msgs/msg/Int32 "{data: 1}"
```

`data: 1`을 한 번 받을 때마다 다음 waypoint로 넘어갑니다. 마지막 waypoint까지 완료되면 active target이 비워지고 `/waypoint_manager/current_vertex`가 목표 vertex로 갱신됩니다.

## 5. 확인 명령

Foxy는 `ros2 topic echo --once`가 없으므로 `timeout`을 사용합니다.

```bash
timeout 5 ros2 topic echo /waypoint_manager/target_waypoint
timeout 5 ros2 topic echo /way_point
timeout 5 ros2 topic echo /waypoint_manager/active_waypoint_index
timeout 5 ros2 topic echo /joy
timeout 5 ros2 topic echo /speed
```

`/waypoint_manager/target_waypoint`는 나오는데 `/way_point`가 안 나오면 runner가 안 떠 있거나 토픽명이 다릅니다.

`/way_point`와 `/joy`가 나오는데 안 움직이면 RViz/control panel에서 waypoint mode가 켜져 있는지 확인합니다.

## 6. 좌표 보정 검증

테스트용 map이 아래처럼 설정되어 있다고 가정합니다.

```text
vertex 2 = (0.0, 3.0)
vertex 3 = (3.0, 3.0)
```

`home_vertex:=3`, `odometry_origin_vertex:=3`으로 시작한 뒤 `device2`를 보내면 `config/legs/3_to_2.yaml`을 읽습니다. 예를 들어 `3_to_2.yaml` 첫 waypoint가 `(2.0, 3.0)`이면 발행 좌표는 다음과 같아야 합니다.

```text
published_x = 2.0 - 3.0 = -1.0
published_y = 3.0 - 3.0 =  0.0
```

실행:

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py \
  legs_dir:=$HOME/Waypoint_Manager/config/legs \
  vertices_file:=$HOME/Waypoint_Manager/config/vertices.yaml \
  home_vertex:=3 \
  odometry_origin_vertex:=3
```

`device2` 명령:

```bash
ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: [
  {name: device1, level: [0], message: '', hardware_id: '', values: []},
  {name: device2, level: [1], message: '', hardware_id: '', values: []},
  {name: device3, level: [0], message: '', hardware_id: '', values: []},
  {name: device4, level: [0], message: '', hardware_id: '', values: []}
]}"
```

정상 로그:

```text
Published waypoint 1/3: map=(2.000, 3.000), origin=(3.000, 3.000), target=(-1.000, 0.000, 0.000)
```

완료 신호를 보낼 때마다 다음 waypoint가 1회 발행됩니다.

```bash
ros2 topic pub --once /waypoint_manager/waypoint_reached std_msgs/msg/Int32 "{data: 1}"
```

예상 로그:

```text
Published waypoint 2/3: map=(1.000, 3.000), origin=(3.000, 3.000), target=(-2.000, 0.000, 0.000)
Published waypoint 3/3: map=(0.000, 3.000), origin=(3.000, 3.000), target=(-3.000, 0.000, 0.000)
Completed hop 3 -> 2 after 3 waypoint(s).
```

`map=` 값이 YAML 원본 좌표와 달라지거나, `target=`에서 origin이 두 번 빠진 것처럼 보이면 marker 또는 다른 publish 경로가 원본 waypoint를 수정하고 있는지 확인해야 합니다.

## 7. 다음 구간 실행

1번에서 2번으로 가는 leg가 마지막 waypoint까지 완료되면 `/waypoint_manager/current_vertex`가 자동으로 2번이 됩니다. 그 상태에서 2번에서 3번으로 보내려면 `device3`을 보냅니다.

```bash
ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: [{name: device3, level: [1], message: '', hardware_id: '', values: []}]}"
```

target을 중간에 취소했거나 실제 위치와 manager 상태가 다르면 그때만 수동으로 맞춥니다.

```bash
ros2 service call /waypoint_manager/cancel std_srvs/srv/Trigger {}
ros2 topic pub --once /waypoint_manager/set_current_vertex std_msgs/msg/Int32 "{data: 2}"
```

필요한 leg 파일:

```text
Waypoint_Manager/config/legs/2_to_3.yaml
```

## 8. 원점 vertex 변경

autonomy stack을 2번 위치에서 새로 켰다면 시작 logical vertex와 local origin을 launch 파라미터로 같이 맞춥니다.

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py \
  legs_dir:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/legs \
  vertices_file:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/vertices.yaml \
  home_vertex:=2 \
  odometry_origin_vertex:=2
```

stack을 재시작하지 않았다면 `set_odometry_origin`을 바꾸지 마세요.
