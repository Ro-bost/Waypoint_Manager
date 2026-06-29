# waypoint_manager

ICROS `/factory/diagnostics` + leg YAML → **[autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)** `/way_point` 주행.

## 흐름

```
diagnostics (다음 목적지 1개)  →  leg YAML 로드  →  /way_point 발행
        ↑                              ↓
   (도착 후에만 수신)            /state_estimation으로 도착 판정
                                        ↓
                              ~/arrival (도착 꼭짓점) publish
                                        ↓
                              다음 diagnostics 처리
```

**주행 중에는 새 diagnostics를 저장만 하고 출발하지 않음.**  
**도착 완료 후**에 그때까지 쌓인 diagnostics를 보고 다음 hop 시작.

예:
```
0010 → 1→3 주행 중 (0200 와도 무시) → 3 도착 → arrival=3
0200 → 3→2 출발 → 2 도착 → arrival=2
1000 → 2→1 출발 → 1 도착 → arrival=1
```

## autonomy_stack_go2 연동

| 토픽 | 방향 | 설명 |
|------|------|------|
| `/way_point` | publish | `geometry_msgs/PointStamped` (local_planner 목표) |
| `/speed` | publish | `std_msgs/Float32` |
| `/state_estimation` | subscribe | `nav_msgs/Odometry` (도착 판정) |

`./system_real_robot.sh` 또는 simulation 실행 후 waypoint_manager launch.

RViz에서 **Resume Navigation to Goal** 눌러 waypoint mode 확인.

## 빌드

```bash
cd ~/waypoint_manager
source /opt/ros/foxy/setup.bash   # Jetson
colcon build --symlink-install
source install/setup.bash
```

## 실행

```bash
# 터미널 1: autonomy stack
cd ~/autonomy_stack_go2 && ./system_real_robot.sh

# 터미널 2: waypoint manager
ros2 launch waypoint_manager waypoint_manager.launch.py
```

## leg YAML

`config/legs/{from}_to_{to}.yaml`

```yaml
frame_id: map
waypoints:
  - x: 1.0
    y: 2.0
    yaw: 0.0
```

## 토픽 / 서비스

| 이름 | 타입 | 설명 |
|------|------|------|
| `/factory/diagnostics` | in | 설비 상태 |
| `/waypoint_manager/route` | out | 현재 hop `[from, to]` |
| `/waypoint_manager/current_vertex` | out | 마지막 도착 꼭짓점 |
| `/waypoint_manager/arrival` | out | hop 완료 시 도착 꼭짓점 |
| `/waypoint_manager/start` | srv | 수동 출발 |

## 파라미터

- `auto_start`: diagnostics 수신 시 자동 출발 (기본 `true`)
- `waypoint_xy_radius`: 도착 판정 반경 m (기본 `0.5`, waypoint_example과 동일)
- `navigation_speed`: `/speed` 값 (기본 `1.0`)
- `legs_dir`: leg YAML 폴더

## 테스트 (Humble PC)

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py auto_start:=false
```

```bash
ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
status:
- {name: device3, level: [1], message: '', hardware_id: '', values: []}"
```

→ `route` = `[1, 3]`
