# waypoint_manager

ICROS 2026 공장 설비 `/factory/diagnostics` 수신 → leg YAML 로드 → **[autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)** `/way_point` 주행.

---

## 내비게이션 담당자에게 (연동 안내)

> 실제 주행·local planner·`autonomy_stack_go2` 쪽은 네가 담당하는 걸로 알고 있어.  
> 이 패키지는 **“어디로 갈지” 정해서 `/way_point`를 쏴주는 쪽**이고, **로봇을 움직이는 코드는 건드리지 않았어.**  
> 아래만 맞춰 주면 같이 돌아가.

### 역할 나누기

| 담당 | 하는 일 |
|------|---------|
| **waypoint_manager (이 repo)** | `/factory/diagnostics` 파싱, leg YAML 로드, `/way_point`·`/speed` 발행, 도착 판정, `~/arrival` 알림 |
| **네 쪽 (autonomy_stack_go2)** | stack 실행, localization, local planner, `/state_estimation` 발행, `/way_point` 구독해서 실제 주행 |

### 같이 켤 때 순서

```bash
# 1) 네 쪽 — 반드시 먼저 (실행 위치가 odom (0,0)이 됨)
cd ~/autonomy_stack_go2 && ./system_real_robot.sh
# RViz에서 Resume Navigation to Goal → waypoint mode

# 2) 이쪽 — diagnostics + waypoint 발행
ros2 launch waypoint_manager waypoint_manager.launch.py
```

**중요:** stack은 **1번 꼭짓점에서** 켜는 걸 기본으로 생각했어. 다른 데서 켜면 `odometry_origin_vertex` 맞춰야 함 (아래 좌표 섹션).

### 네가 구독하면 되는 토픽 (waypoint_manager → 너)

| 토픽 | 타입 | 주기 | 내용 |
|------|------|------|------|
| `/way_point` | `geometry_msgs/PointStamped` | 5Hz | **현재 leg의 다음 waypoint** (이미 autonomy 좌표로 보정됨) |
| `/speed` | `std_msgs/Float32` | 5Hz | 기본 `1.0` (파라미터 `navigation_speed`) |

- 한 hop에 waypoint가 여러 개면 **순서대로**보내. 하나 도착 판정 나면 다음 점으로 바꿔.
- hop 끝나면 `/way_point` 안 쏘다가, 다음 diagnostics 오면 새 leg 시작.

### 네가 발행해야 하는 토픽 (너 → waypoint_manager)

| 토픽 | 타입 | 용도 |
|------|------|------|
| `/state_estimation` | `nav_msgs/Odometry` | **도착 판정** — 지금 위치랑 목표 waypoint 거리 비교 |

도착 조건 (waypoint_example이랑 동일하게 맞춤):

- XY 거리 `< 0.5 m` (`waypoint_xy_radius`)
- Z 차이 `< 5.0 m` (`waypoint_z_bound`)

**네 odom이랑 내가 쏘는 `/way_point`는 같은 좌표계**여야 해. 둘 다 “stack 켠 위치 = (0,0)” 기준.

### 좌표계 — 꼭 읽어줘

leg YAML에 적는 좌표는 **맵 절대좌표**야.  
근데 autonomy는 **stack 실행 위치를 (0,0)** 으로 쓰니까, 내가 보내기 전에 이렇게 빼:

```
/way_point.x = map_x - vertices[origin].x
/way_point.y = map_y - vertices[origin].y
```

- 기본 `origin = 1번 꼭짓점`
- `config/vertices.yaml` — 꼭짓점 1~4 맵 좌표 (**실측해서 같이 채워야 함**)
- `config/legs/*.yaml` — 구간별 waypoint (**이것도 실측 좌표 필요**)

네가 map / RViz에서 찍은 값이랑 **같은 frame**으로 맞춰 주면 됨. 보통 `frame_id: map`.

### skip / 실패했을 때

1→2 가다 실패해서 2번 근처에서 다시 태울 때:

1. `~/cancel` 로 내 쪽 hop 중단
2. **stack을 2번 위치에서 다시 실행** (odom 원점이 바뀜)
3. 아래 두 개 publish:

```bash
ros2 topic pub --once /waypoint_manager/set_odometry_origin std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /waypoint_manager/set_current_vertex std_msgs/msg/Int32 "{data: 2}"
```

stack **안** 재시작하고 skip만 하면 origin은 여전히 1번이야. 그때 origin을 2로 바꾸면 waypoint 틀어짐.

### 참고용으로 보면 좋은 토픽

| 토픽 | 설명 |
|------|------|
| `/waypoint_manager/route` | 지금 hop `[출발, 도착]` 예: `[1, 3]` |
| `/waypoint_manager/current_vertex` | 내가 생각하는 현재 꼭짓점 |
| `/waypoint_manager/arrival` | **hop 한 번 끝났을 때** 도착 꼭짓점 번호 (운영팀 연동용) |
| `/waypoint_manager/markers` | RViz용 화살표 (보정된 좌표) |

### 같이 테스트할 때

```bash
# diagnostics 없이 수동 확인하려면
ros2 launch waypoint_manager waypoint_manager.launch.py auto_start:=false

ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
status:
- {name: device3, level: [1], message: '', hardware_id: '', values: []}"

# 기대: route [1,3], /way_point에 보정된 좌표 나옴
ros2 topic echo /waypoint_manager/route --once
ros2 topic echo /way_point
```

### 건드리면 안 맞는 것들

- `/way_point`, `/speed`, `/state_estimation` **이름 바꾸면** launch 인자로 맞춰야 함  
  `ros2 launch waypoint_manager waypoint_manager.launch.py way_point_topic:=/네토픽`
- stack 실행 위치 바꿨는데 `odometry_origin_vertex` 안 맞추면 전부 어긋남
- `/way_point`를 map 절대좌표로 받도록 바꿔 놓으면 **이 패키지랑 안 맞음** (지금은 autonomy 상대좌표로 쏨)

### 아직 placeholder인 것 (대회 전 같이 채우자)

- `config/vertices.yaml` — 1~4번 좌표 `0,0` placeholder
- `config/legs/*.yaml` — 구간 waypoint 좌표 비어 있음

맵 밟으면서 RViz 찍은 값 알려주면 내가 YAML에 넣을게. 반대로 네 쪽에서 이미 쓰는 waypoint 파일 있으면 그 좌표계 기준 알려줘.

질문 있으면 `waypoint_manager_node.py` 보면 됨. 주행 로직은 거기 없고 **토픽 연동 + YAML + 도착 판정**만 있어.

---

## 맵 / 설비 매핑

```
1 -------- 4
|          |
2 -------- 3
```

| diagnostics `name` | 꼭짓점 | 위치 |
|--------------------|--------|------|
| `device1` | 1 | 왼쪽 위 |
| `device2` | 2 | 왼쪽 아래 |
| `device3` | 3 | 오른쪽 아래 |
| `device4` | 4 | 오른쪽 위 |

경로는 **구간(leg) YAML**을 이어 붙입니다. 예: `1→3→4→2→1` = `1_to_3.yaml` + `3_to_4.yaml` + …

## 동작 흐름

```
diagnostics (다음 목적지 1개)  →  leg YAML 로드  →  /way_point 발행 (좌표 보정)
        ↑                              ↓
   (도착 후에만 출발)            /state_estimation 도착 판정
                                        ↓
                              ~/arrival (도착 꼭짓점)
                                        ↓
                              다음 diagnostics 처리
```

- `status[]`에서 **첫 번째 `level≠0`** 항목의 `name` → 목적 꼭짓점
- 운영팀이 1Hz로 **목적지를 하나씩** 보내는 전제 (예: device3만 WARN → 1→3)
- **주행 중** diagnostics는 저장만 하고 새 hop은 시작하지 않음
- **도착 후** 쌓인 diagnostics로 다음 hop 시작

예:

```
device3 WARN → 1→3 주행 (중간에 device2 WARN 와도 무시) → 3 도착 arrival=3
device2 WARN → 3→2 출발 → 2 도착 arrival=2
device1 OK이지만 level≠0 복귀 신호 → 2→1 → 1 도착 arrival=1
```

## ICROS diagnostics (docx 규격)

| 항목 | 값 |
|------|-----|
| 토픽 | `/factory/diagnostics` |
| 메시지 | `diagnostic_msgs/msg/DiagnosticArray` |
| QoS | RELIABLE + VOLATILE |
| 설비 ID | `status[].name` = `device1`~`device4` |
| 상태 | `level` 0=OK, 1=WARN, 2=ERROR |

수신 시 **TXT 로그**에 설비별 기록 (`name`, `level`, `hardware_id`, `message`, `values`).

## autonomy_stack_go2 연동

| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/way_point` | publish | `geometry_msgs/PointStamped` | local_planner 목표 (보정된 좌표) |
| `/speed` | publish | `std_msgs/Float32` | 주행 속도 |
| `/state_estimation` | subscribe | `nav_msgs/Odometry` | 도착 판정 |

```bash
# 터미널 1
cd ~/autonomy_stack_go2 && ./system_real_robot.sh

# 터미널 2
ros2 launch waypoint_manager waypoint_manager.launch.py
```

RViz에서 **Resume Navigation to Goal** 로 waypoint mode 활성화.

## 좌표 보정 (autonomy (0,0) 문제)

`autonomy_stack_go2`는 **stack 실행 위치를 (0,0)** 으로 씁니다.  
leg YAML waypoint는 **맵 절대좌표**이므로 `/way_point`·도착 판정 시 **원점 꼭짓점 좌표를 뺍니다**.

```
autonomy_x = map_x - vertices[origin_vertex].x
autonomy_y = map_y - vertices[origin_vertex].y
```

| 상황 | `odometry_origin_vertex` |
|------|--------------------------|
| 1번에서 stack 실행 (기본) | `1` |
| skip 후 **2번에서 stack 재실행** | `2` |

### 설정 파일

**`config/vertices.yaml`** — 꼭짓점 1~4 맵 좌표 (leg YAML과 **동일 좌표계**)

```yaml
frame_id: map
vertices:
  1: {x: 0.0, y: 0.0, yaw: 0.0}
  2: {x: 0.0, y: -4.0, yaw: 0.0}
  3: {x: 5.0, y: -4.0, yaw: 0.0}
  4: {x: 5.0, y: 0.0, yaw: 0.0}
```

**`config/legs/{from}_to_{to}.yaml`** — 구간별 waypoint (맵 좌표)

```yaml
frame_id: map
waypoints:
  - x: 1.0
    y: 2.0
    yaw: 0.0
```

> **주의:** stack을 재시작하지 않고 skip만 하면 오도메트리 원점은 여전히 **처음 켠 꼭짓점**입니다.  
> origin만 바꾸면 waypoint가 틀어집니다. skip 후 stack 재실행 → `set_odometry_origin` + `set_current_vertex`.

### skip 예시 (1→2 실패, 2번에서 재출발)

```bash
ros2 service call /waypoint_manager/cancel std_srvs/srv/Trigger {}

ros2 topic pub --once /waypoint_manager/set_odometry_origin std_msgs/msg/Int32 "{data: 2}"
ros2 topic pub --once /waypoint_manager/set_current_vertex std_msgs/msg/Int32 "{data: 2}"
```

`sync_odometry_origin_on_vertex_set:=true` 는 stack을 해당 꼭짓점에서 다시 켠 경우에만 사용.

## 빌드

```bash
cd ~/waypoint_manager
source /opt/ros/foxy/setup.bash    # Jetson (Foxy)
# source /opt/ros/humble/setup.bash  # PC 테스트
colcon build --symlink-install
source install/setup.bash
```

## 토픽 / 서비스

| 이름 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/factory/diagnostics` | in | `DiagnosticArray` | 설비 상태 (docx) |
| `/waypoint_manager/route` | out | `Int32MultiArray` | 현재 hop `[from, to]` |
| `/waypoint_manager/current_vertex` | out | `Int32` | 현재 논리 위치 꼭짓점 |
| `/waypoint_manager/arrival` | out | `Int32` | hop 완료 시 도착 꼭짓점 |
| `/waypoint_manager/waypoint_count` | out | `Int32` | 로드된 waypoint 수 |
| `/waypoint_manager/markers` | out | `MarkerArray` | RViz 마커 (보정 좌표) |
| `/waypoint_manager/set_current_vertex` | in | `Int32` | skip 시 논리 위치 (idle만) |
| `/waypoint_manager/set_odometry_origin` | in | `Int32` | autonomy (0,0) 꼭짓점 |
| `/waypoint_manager/start` | srv | `Trigger` | 수동 출발 |
| `/waypoint_manager/cancel` | srv | `Trigger` | 주행 취소 |
| `/waypoint_manager/reload` | srv | `Trigger` | leg YAML 다시 로드 |

## diagnostics TXT 로그

기본 경로: `~/waypoint_manager/logs/diagnostics_log.txt`

한 줄 형식:

```text
[timestamp] device=device3 level=1(WARN) vertex=3 hardware_id=SN-003 message=temp high values=temp=85
```

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py diagnostics_log_enabled:=false
ros2 launch waypoint_manager waypoint_manager.launch.py \
  diagnostics_log_path:=/home/philoshan/logs/factory_diagnostics.txt
```

## 파라미터

### launch 인자

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `auto_start` | `true` | diagnostics 수신 시 자동 출발 |
| `legs_dir` | `share/.../config/legs` | leg YAML 폴더 |
| `vertices_file` | `share/.../config/vertices.yaml` | 꼭짓점 좌표 |
| `odometry_origin_vertex` | `1` | autonomy (0,0) 꼭짓점 |
| `sync_odometry_origin_on_vertex_set` | `false` | `set_current_vertex` 시 origin 동기화 |
| `way_point_topic` | `/way_point` | |
| `state_estimation_topic` | `/state_estimation` | |
| `diagnostics_log_enabled` | `true` | TXT 로그 |
| `diagnostics_log_path` | `~/waypoint_manager/logs/diagnostics_log.txt` | |

### 노드 파라미터 (추가)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `diagnostics_topic` | `/factory/diagnostics` | |
| `home_vertex` | `1` | 시작 꼭짓점 |
| `navigation_speed` | `1.0` | `/speed` 값 |
| `waypoint_xy_radius` | `0.5` | 도착 판정 반경 (m) |
| `waypoint_z_bound` | `5.0` | z 허용 오차 (m) |
| `waypoint_publish_rate` | `5.0` | `/way_point` 발행 Hz |

## 테스트 (PC / Humble)

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py auto_start:=false
```

```bash
ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
status:
- {name: device3, level: [1], message: 'temp high', hardware_id: 'SN-003', values: [{key: temp, value: '85'}]}"
```

```bash
ros2 topic echo /waypoint_manager/route --once   # [1, 3] 기대
cat ~/waypoint_manager/logs/diagnostics_log.txt
```

## 패키지 구조

```
waypoint_manager/
├── config/
│   ├── vertices.yaml      # 꼭짓점 1~4 맵 좌표
│   └── legs/              # {from}_to_{to}.yaml
├── launch/
│   └── waypoint_manager.launch.py
└── waypoint_manager/
    ├── waypoint_manager_node.py
    ├── leg_loader.py
    └── diagnostics_logger.py
```

대회 전 **`vertices.yaml`과 leg YAML 좌표를 실측값으로 채워야** 합니다 (현재 placeholder).
