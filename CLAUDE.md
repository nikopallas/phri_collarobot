# CollaboRobot — CLAUDE.md

## Project overview
ROS2 Jazzy workspace for a Kinova Gen3 6-DOF arm + Robotiq gripper.
Three packages: `collarobot_actions`, `collarobot_controller`, `collarobot_perception`.

## Hardware / ROS topics
| Resource | Topic / Server |
|---|---|
| Arm trajectory | `/joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`) |
| Gripper action | `/robotiq_gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`) |
| Joint feedback | `/joint_states` (`sensor_msgs/JointState`) |
| Gripper: open | `position = 0.0` |
| Gripper: close | `position = 0.8` |

```python
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
```

## positions.toml
Location (hardcoded in all nodes):
```
~/collarobot_ws/src/collarobot_actions/collarobot_actions/positions.toml
```
Record a position with:
```bash
ros2 run collarobot_actions record_position <name> [--time SECS]
```

### Schema (current)
```toml
[home]
joints = [j1, j2, j3, j4, j5, j6]   # radians
time_from_start = 3.0                 # seconds
```

### Planned schema extension — transition graph
Add `allowed_next` to encode which moves are legal from each position.
This lets `move_controller` enforce safe sequencing without hard-coding it in nodes.
```toml
[home]
joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
time_from_start = 3.0
allowed_next = ["pre_pick", "pre_place"]

[pre_pick]
joints = [...]
time_from_start = 3.0
allowed_next = ["pick", "home"]
```

## Architecture (planned)

### collarobot_msgs  ← NEW package
Custom service for arm control:
```
# MoveToPosition.srv
string position_name
---
bool success
string message          # failure reason
string from_position    # detected current position
```

### collarobot_controller — move_controller node
Service server: `/move_to_position` (`collarobot_msgs/MoveToPosition`)

**Service logic:**
1. Cache latest `/joint_states`
2. On call: compare cached joints vs all TOML positions (L∞ threshold ~0.05 rad)
3. If no match → reject ("unknown current position")
4. Look up `allowed_next` for matched position
5. If target not in `allowed_next` → reject ("transition X→Y not allowed")
6. Publish trajectory, sleep, return success

Also exposes a **busy flag** so callers know if a move is already in progress.

### collarobot_actions — pick_place_node
Refactor `_move_to()` to call `/move_to_position` service instead of publishing directly.
This makes the node thin: just sequencing + gripper control.

## Build / run
```bash
# Build
colcon build --packages-select collarobot_actions collarobot_controller --symlink-install
. install/setup.bash

# Quick move test (one shot, no service needed)
ros2 run collarobot_controller goto_position <name>

# Full pick-place
ros2 run collarobot_actions pick_place_node
ros2 service call /pick_place_node/trigger std_srvs/srv/Trigger
ros2 topic echo /pick_place_node/state
```

## Key decisions / lessons
- `maintainer_email` in package.xml must be a valid email format (e.g. `phri3@todo.todo`)
  or ament_index registration silently fails → `Package not found` at runtime.
- `--symlink-install` only symlinks .py files; TOML/data files are copied.
  Hardcode POSITIONS_PATH to `Path.home() / 'collarobot_ws' / 'src' / ...` so writes
  always go to the source tree.
- Use `MultiThreadedExecutor` whenever a node has a background thread that sleeps
  (state machine, blocking motion wait) so ROS callbacks keep firing.
- Gripper wait: use `threading.Event`, never `spin_until_future_complete` from a
  non-main thread — it causes executor re-entrance deadlocks.
