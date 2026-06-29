# Waypoint Manager Runbook

ROS2 Foxy, SSH, real robot 기준 실행 절차입니다.

`waypoint_manager`는 로봇 위치를 직접 판단하지 않습니다. diagnostics를 받아 leg YAML의 target waypoint를 `/waypoint_manager/target_waypoint`로 계속 발행하고, runner가 이것을 `/way_point`로 넘깁니다. 실제 주행, 정지, 장애물 회피는 `autonomy_stack_go2` local planner가 담당합니다.

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

현재 운영 방식은 leg 파일의 첫 waypoint를 target으로 계속 발행합니다. 지금처럼 구간별 최종 목표점 1개만 넣는 구성이 가장 단순합니다.

예시:

```yaml
frame_id: map
waypoints:
- x: 3.398365020751953
  y: 0.7653948664665222
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
  vertices_file:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/vertices.yaml
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

현재 논리 위치를 1번으로 설정:

```bash
ros2 topic pub --once /waypoint_manager/set_current_vertex std_msgs/msg/Int32 "{data: 1}"
```

1번에서 2번으로 보내기:

```bash
ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: [{name: device2, level: [1], message: '', hardware_id: '', values: []}]}"
```

Foxy에서는 반드시 `level: [1]`을 사용합니다. `level: 1`로 보내면 OK로 들어갈 수 있습니다.

정상 로그:

```text
# 터미널 2
Loaded 1 waypoint(s) for hop 1 -> 2
Commanding hop: 1 -> 2

# 터미널 3
new manager waypoint: frame=map, x=..., y=..., z=...
```

## 5. 확인 명령

Foxy는 `ros2 topic echo --once`가 없으므로 `timeout`을 사용합니다.

```bash
timeout 5 ros2 topic echo /waypoint_manager/target_waypoint
timeout 5 ros2 topic echo /way_point
timeout 5 ros2 topic echo /joy
timeout 5 ros2 topic echo /speed
```

`/waypoint_manager/target_waypoint`는 나오는데 `/way_point`가 안 나오면 runner가 안 떠 있거나 토픽명이 다릅니다.

`/way_point`와 `/joy`가 나오는데 안 움직이면 RViz/control panel에서 waypoint mode가 켜져 있는지 확인합니다.

## 6. 다음 구간 실행

2번에서 3번으로 보내려면 현재 논리 위치를 2번으로 맞춘 뒤 `device3`을 보냅니다.

```bash
ros2 service call /waypoint_manager/cancel std_srvs/srv/Trigger {}
ros2 topic pub --once /waypoint_manager/set_current_vertex std_msgs/msg/Int32 "{data: 2}"

ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: [{name: device3, level: [1], message: '', hardware_id: '', values: []}]}"
```

필요한 leg 파일:

```text
Waypoint_Manager/config/legs/2_to_3.yaml
```

## 7. 원점 vertex 변경

autonomy stack을 2번 위치에서 새로 켰다면 local origin도 2번으로 맞춥니다.

```bash
ros2 topic pub --once /waypoint_manager/set_odometry_origin std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /waypoint_manager/set_current_vertex std_msgs/msg/Int32 "{data: 2}"
```

stack을 재시작하지 않았다면 `set_odometry_origin`을 바꾸지 마세요.
