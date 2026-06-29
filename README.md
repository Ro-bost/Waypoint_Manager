# waypoint_manager

ICROS factory `/factory/diagnostics` -> leg YAML -> autonomy_stack_go2 waypoint target publisher.

It does not judge whether the robot moved or arrived. The autonomy stack/local planner handles motion exactly like an RViz waypoint target.

## Runtime Role

| Part | Role |
|------|------|
| `waypoint_manager` | Parse diagnostics, load `config/legs/{from}_to_{to}.yaml`, publish target waypoint |
| `run_relative_waypoint_sequence.py` | Relay manager target to `/way_point` and publish `/joy` for waypoint mode |
| `autonomy_stack_go2` | Local planning, obstacle avoidance, actual robot motion |

## Control Flow

`waypoint_manager` keeps two pieces of state:

| State | Meaning | Used for |
|-------|---------|----------|
| `home_vertex` / `current_vertex` | Logical vertex where the manager believes the robot is | Selects the leg YAML, e.g. `1_to_2.yaml` |
| `odometry_origin_vertex` | Vertex where the autonomy stack local odometry started | Converts map-frame YAML coordinates to the autonomy local frame |

At launch, set both to the vertex where the robot/autonomy stack starts:

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py \
  legs_dir:=$HOME/Waypoint_Manager/config/legs \
  vertices_file:=$HOME/Waypoint_Manager/config/vertices.yaml \
  home_vertex:=1 \
  odometry_origin_vertex:=1
```

After launch, normal operation does not require publishing `set_current_vertex`
or `set_odometry_origin` for every command. `current_vertex` is updated after
the last waypoint in a leg is reported reached.

For a command from vertex 1 to `device2`, the flow is:

```text
/factory/diagnostics says device2 is non-zero
  -> target vertex = 2
  -> current_vertex = 1
  -> route = [1, 2]
  -> load config/legs/1_to_2.yaml
  -> publish waypoints[0] once on /waypoint_manager/target_waypoint
  -> wait for /waypoint_manager/waypoint_reached data: 1
  -> publish waypoints[1] once
  -> repeat until the last waypoint is reached
  -> clear active target and set current_vertex = 2
```

`/waypoint_manager/route`, `/waypoint_manager/current_vertex`, and
`/waypoint_manager/active_waypoint_index` are status/debug topics. The lower
navigation layer only needs `/waypoint_manager/target_waypoint`, `/speed`, and
the completion signal `/waypoint_manager/waypoint_reached`.

## Real Robot Run Order (ROS2 Foxy / SSH)

Use four SSH terminals.

### Terminal 1: autonomy stack

```bash
cd ~/autonomy_stack_go2
source install/setup.bash
./system_real_robot.sh
```

### Terminal 2: waypoint_manager

Use explicit source paths so edited YAML files are read directly.

```bash
cd ~/autonomy_stack_go2
source install/setup.bash

ros2 launch waypoint_manager waypoint_manager.launch.py \
  legs_dir:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/legs \
  vertices_file:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/vertices.yaml \
  home_vertex:=1 \
  odometry_origin_vertex:=1
```

### Terminal 3: manager runner

```bash
cd ~/autonomy_stack_go2
source install/setup.bash
./tools/run_relative_waypoint_sequence.py --joy-speed-axis 0.35
```

If it is still too aggressive near walls, lower the value:

```bash
./tools/run_relative_waypoint_sequence.py --joy-speed-axis 0.25
```

### Terminal 4: send command

Reset the manager-side target if one is active. If `home_vertex` was set
correctly at launch, you do not need to publish `set_current_vertex` before
each command.

```bash
cd ~/autonomy_stack_go2
source install/setup.bash

ros2 service call /waypoint_manager/cancel std_srvs/srv/Trigger {}
```

Send `device2` as the next target:

```bash
ros2 topic pub --once /factory/diagnostics diagnostic_msgs/msg/DiagnosticArray \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: [{name: device2, level: [1], message: '', hardware_id: '', values: []}]}"
```

If your ROS environment reports `DiagnosticStatus.level` as a byte field, use
`level: [1]`. If it accepts integer values, `level: 1` is also valid.

Expected logs:

```text
# terminal 2
Loaded 1 waypoint(s) for hop 1 -> 2
Commanding hop: 1 -> 2

# terminal 3
new manager waypoint: frame=map, x=3.398, y=0.765, z=-0.002
```

## Topics

| Topic | Direction | Type | Description |
|------|-----------|------|-------------|
| `/factory/diagnostics` | in | `diagnostic_msgs/msg/DiagnosticArray` | Factory device target |
| `/waypoint_manager/target_waypoint` | out | `geometry_msgs/msg/PointStamped` | Target consumed by runner |
| `/speed` | out | `std_msgs/msg/Float32` | Speed command value |
| `/waypoint_manager/waypoint_reached` | in | `std_msgs/msg/Int32` | Publish `data: 1` when the lower layer reached the active waypoint |
| `/waypoint_manager/route` | out | `std_msgs/msg/Int32MultiArray` | Current logical route, e.g. `[1, 2]` |
| `/waypoint_manager/current_vertex` | out | `std_msgs/msg/Int32` | Manager-side logical vertex |
| `/waypoint_manager/waypoint_count` | out | `std_msgs/msg/Int32` | Loaded waypoint count |
| `/waypoint_manager/active_waypoint_index` | out | `std_msgs/msg/Int32` | Zero-based active waypoint index, or `-1` when idle |
| `/waypoint_manager/markers` | out | `visualization_msgs/msg/MarkerArray` | RViz markers |
| `/waypoint_manager/set_current_vertex` | in | `std_msgs/msg/Int32` | Manually set logical vertex |
| `/waypoint_manager/set_odometry_origin` | in | `std_msgs/msg/Int32` | Set local-origin vertex for coordinate offset |

`set_current_vertex` and `set_odometry_origin` are manual override topics. In
normal startup, prefer launch parameters `home_vertex` and
`odometry_origin_vertex`.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/waypoint_manager/start` | `std_srvs/srv/Trigger` | Start from latest diagnostics |
| `/waypoint_manager/cancel` | `std_srvs/srv/Trigger` | Stop publishing current target |
| `/waypoint_manager/reload` | `std_srvs/srv/Trigger` | Reload current route YAML |

## Device Mapping

```text
1 -------- 4
|          |
2 -------- 3
```

| diagnostics `name` | Vertex |
|--------------------|--------|
| `device1` | 1 |
| `device2` | 2 |
| `device3` | 3 |
| `device4` | 4 |

The first non-zero `status[].level` in a diagnostics message becomes the next target.

## Coordinates

Leg YAML coordinates are map-frame absolute points. Before publishing, waypoint_manager subtracts the configured local-origin vertex:

```text
published_x = map_x - vertices[origin].x
published_y = map_y - vertices[origin].y
```

Default origin vertex is `1`.

When the autonomy stack is started at vertex 1, keep the default. If the stack is restarted at another vertex, set both launch arguments:

```bash
ros2 launch waypoint_manager waypoint_manager.launch.py \
  legs_dir:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/legs \
  vertices_file:=$HOME/autonomy_stack_go2/Waypoint_Manager/config/vertices.yaml \
  home_vertex:=2 \
  odometry_origin_vertex:=2
```

## Leg YAML

Path:

```text
Waypoint_Manager/config/legs/{from}_to_{to}.yaml
```

Example:

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

The manager publishes only one `PointStamped` target at a time. It publishes
the first waypoint once when the leg starts, then waits until the lower layer
reports completion:

```bash
ros2 topic pub --once /waypoint_manager/waypoint_reached std_msgs/msg/Int32 "{data: 1}"
```

Each `data: 1` advances to the next waypoint and publishes that next point
once. After the last waypoint is reported reached, the active target is cleared
and `current_vertex` is updated to the route target vertex.

## Build

```bash
cd ~/autonomy_stack_go2
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select waypoint_manager
source install/setup.bash
```
