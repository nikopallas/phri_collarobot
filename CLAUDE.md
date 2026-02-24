# CollaboRobot — CLAUDE.md

## Project overview
ROS2 Jazzy workspace for a Kinova Gen3 6-DOF arm + Robotiq gripper + Elmo carriage/lift.
Packages: `collarobot_msgs`, `collarobot_actions`, `collarobot_controller`, `collarobot_perception`.

## Hardware / ROS topics
| Resource | Topic / Server |
|---|---|
| Arm trajectory | `/joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`) |
| Gripper action | `/robotiq_gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`) |
| Joint feedback | `/joint_states` (`sensor_msgs/JointState`) |
| Carriage position read | `/elmo/id1/carriage/position/get` (`std_msgs/Float32`) |
| Carriage position set  | `/elmo/id1/carriage/position/set` (`std_msgs/Float32`) |
| Lift position read | `/elmo/id1/lift/position/get` (`std_msgs/Float32`) |
| Lift position set  | `/elmo/id1/lift/position/set` (`std_msgs/Float32`) |
| Gripper: open | `position = 0.0` |
| Gripper: close | `position = 0.8` (calibrate with `test_gripper`) |

```python
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
```

## positions.toml
Location (hardcoded in all nodes):
```
~/collarobot_ws/src/collarobot_actions/collarobot_actions/positions.toml
```

### Schema
```toml
gripper_open  = 0.0   # top-level scalars read by pick_place_node
gripper_close = 0.8

[home]
joints          = [j1, j2, j3, j4, j5, j6]   # radians
time_from_start = 5.0                          # seconds
carriage        = 2.5000   # optional — recorded automatically if topic is live
lift            = 0.3000   # optional — recorded automatically if topic is live
```

Carriage/lift in `[home]` are used as the **pre-run position check** in `pick_place_node`.

## CLI tools (all in `collarobot_actions`)
```bash
# Record current arm + carriage/lift position
ros2 run collarobot_actions record_position <name> [--time SECS]

# Move arm to a named position (one-shot test)
ros2 run collarobot_actions goto_position <name>

# Command carriage+lift to a named position's recorded values (operator setup)
ros2 run collarobot_actions goto_carriage_lift <name>

# Test gripper at a specific position value (calibration)
ros2 run collarobot_actions test_gripper <0.0–1.0>
```

## pick_place_node — action server
Action: `/pick_place_node/pick_place` (`collarobot_msgs/action/PickPlace`)

Sequence:
```
PRE_PICK → PICKING (close gripper) → POST_PICK → HOME → PRE_PLACE → PLACING (open gripper) → POST_PLACE
```
- `POST_PICK` reuses `pre_pick` position (retreat to approach height)
- `POST_PLACE` reuses `pre_place` position (retreat to approach height)
- Before starting: checks carriage/lift are within `CARRIAGE_LIFT_TOL = 0.05` of `home` values

```bash
# Start node
ros2 run collarobot_actions pick_place_node

# Trigger (with live step feedback)
ros2 action send_goal --feedback /pick_place_node/pick_place \
    collarobot_msgs/action/PickPlace '{}'
```

## Build
```bash
# Must build collarobot_msgs first (generates PickPlace.action)
colcon build --packages-select collarobot_msgs --symlink-install
. install/setup.bash
colcon build --packages-select collarobot_actions --symlink-install
. install/setup.bash
```

## Architecture (planned — not yet implemented)
### collarobot_msgs — future additions
```
# MoveToPosition.srv
string position_name
---
bool success
string message
string from_position
```

### collarobot_controller — move_controller node
Service `/move_to_position` with:
1. Current position detection via L∞ joint comparison (~0.05 rad threshold)
2. `allowed_next` transition graph from TOML
3. Reject if unknown position or disallowed transition

When implemented, `pick_place_node._move_to()` will call this service instead of publishing directly.

## DDS / RMW
ROS2 default RMW is FastDDS, which pre-allocates message buffers and silently drops
messages that exceed the initial payload size → causes `[RTPS_READER_HISTORY Error]`
hangs (callbacks never fire).

**Fix: use CycloneDDS** (allocates dynamically, no size issues):
```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

Root cause on this machine: MoveIt source build in `~/ws_moveit` recompiles
`rmw_fastrtps` with PREALLOCATED_MEMORY_MODE, overriding the system version.
CycloneDDS bypasses this entirely.

Must be set in every terminal before running any ROS2 node, or added to `~/.bashrc`.

## Key decisions / lessons
- `maintainer_email` in package.xml must be valid format (`phri3@todo.todo`) or
  ament_index silently fails → `Package not found` at runtime.
- `--symlink-install` only symlinks .py files; TOML/data files are copied on build.
  Hardcode `POSITIONS_PATH` to source tree so `record_position` writes go to the right place.
- `MultiThreadedExecutor` required whenever a node blocks in a background thread
  (action execute_callback, motion sleep) so other callbacks (gripper result) keep firing.
- Gripper wait: use `threading.Event` + `add_done_callback`. Never `spin_until_future_complete`
  from a non-executor thread — causes deadlock.
- `ReentrantCallbackGroup` on both ActionServer and ActionClient so the execute_callback
  thread and gripper result callbacks can run concurrently.
- Publisher subscriber-count fix: poll `get_subscription_count() > 0` before publishing
  trajectory — the controller needs to be matched first or messages are dropped.
