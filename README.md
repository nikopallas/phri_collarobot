# CollaboRobot

ROS2 Jazzy workspace for a collaborative pick-and-place system using a **Kinova Gen3 6-DOF arm**, **Robotiq gripper**, and **Elmo carriage/lift**. A camera-based perception layer detects ingredient positions on a shared table, a state machine decides which ingredient to propose or accept, and the robot executes pick-and-place and gesture motions autonomously.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│  collarobot_perception                                               │
│  state_publisher  ── ArUco detection ──► /collarobot/state (JSON)   │
└─────────────────────────────┬────────────────────────────────────────┘
                              │ std_msgs/String
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│  collarobot_controller                                               │
│                                                                      │
│  controller_node ──────────────────────────────► gesture_node       │
│    (state machine)  Gesture action                (collarobot_actions│
│         │                                         /collarobot/gesture│
│         │ MoveIngredient action                                      │
│         ▼                                                            │
│  motion_coordinator_node                                             │
│    tracks exact slot per ingredient                                  │
│    resolves (id, destination) → (pick_slot, place_slot)             │
│         │                                                            │
│         │ PickPlace action                                           │
│         ▼                                                            │
│  pick_place_node  ─────────► /joint_trajectory_controller/...       │
│    (collarobot_actions)  ──► /robotiq_gripper_controller/...        │
│    /collarobot/pick_place    ◄── /elmo/id2/*/position/get           │
└──────────────────────────────────────────────────────────────────────┘
```

---

## Package Structure

```
phri_collarobot/
│
├── collarobot_msgs/                  # Custom ROS2 interfaces
│   └── action/
│       ├── PickPlace.action          # pick_place_node goal/result/feedback
│       ├── Gesture.action            # gesture_node goal/result/feedback
│       └── MoveIngredient.action     # motion_coordinator_node interface
│
├── collarobot_actions/               # Robot motion nodes + CLI tools
│   ├── collarobot_actions/
│   │   ├── pick_place_node.py        # Action server: /collarobot/pick_place
│   │   ├── gesture_node.py           # Action server: /collarobot/gesture
│   │   ├── positions.toml            # Named arm positions (edit source here)
│   │   ├── gestures.toml             # Named gesture waypoints
│   │   └── utils/
│   │       ├── record_position.py    # CLI: capture robot state to TOML
│   │       ├── record_gesture.py     # CLI: record gesture waypoints
│   │       ├── goto_position.py      # CLI: move arm to a named position
│   │       ├── goto_carriage_lift.py # CLI: move carriage/lift to stored values
│   │       └── test_gripper.py       # CLI: test gripper at a specific value
│   └── launch/
│       └── collarobot_actions.launch.py  # Starts pick_place_node + gesture_node
│
├── collarobot_controller/            # State machine + motion coordinator
│   ├── collarobot_controller/
│   │   ├── controller_flow.py        # Main state machine (controller_node)
│   │   ├── motion_coordinator_node.py # Slot tracking + MoveIngredient server
│   │   ├── models.py                 # Decision logic (pick_action)
│   │   └── flow.py                   # State definitions
│   ├── data/
│   │   ├── ingredients_mapping.toml  # ArUco ID → storage slot
│   │   ├── ingredients.json          # Ingredient names
│   │   └── recipes.json              # Recipe definitions
│   └── OPERATOR_GUIDE.md             # Zone/slot layout reference
│
└── collarobot_perception/            # Vision / ArUco detection
    └── collarobot_perception/
        ├── state_detection.py        # state_publisher node → /collarobot/state
        ├── transform_ArUco_tags.py   # ArUco pose estimation
        ├── detect_zones.py           # Table zone detection
        └── transformation.py         # Camera calibration helpers
```

---

## Build

> **DDS requirement:** use CycloneDDS — FastDDS drops large messages silently.
> ```bash
> sudo apt install ros-jazzy-rmw-cyclonedds-cpp
> echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && source ~/.bashrc
> ```

Build in dependency order:

```bash
cd ~/collarobot_ws
git pull

# 1 — interfaces first (other packages depend on these action types)
colcon build --packages-select collarobot_msgs --symlink-install
source install/setup.bash

# 2 — all remaining packages (can build in parallel)
colcon build --packages-select collarobot_actions collarobot_controller collarobot_perception --symlink-install
source install/setup.bash
```

> `--symlink-install` symlinks `.py` files so edits take effect immediately without rebuilding.
> TOML and JSON data files are **copied** on build — always edit the source files in `~/collarobot_ws/src/`.

---

## Running the Full System

**Terminal 1** — robot motion nodes:
```bash
ros2 launch collarobot_actions collarobot_actions.launch.py
# Starts: pick_place_node + gesture_node
```

**Terminal 2** — motion coordinator:
```bash
ros2 run collarobot_controller motion_coordinator_node
```

**Terminal 3** — perception:
```bash
ros2 run collarobot_perception state_publisher
```

**Terminal 4** — main controller:
```bash
ros2 run collarobot_controller controller_node
```

---

## Configuration

### `positions.toml`
Located at `~/collarobot_ws/src/collarobot_actions/collarobot_actions/positions.toml`

```toml
# Gripper calibration
gripper_open  = 0.35
gripper_close = 0.8

# Top-level scalars — used by pick_place_node for carriage/lift pre-flight check
carriage_position = 12.55
lift_position     = 0.0

[home]
joints          = [j1, j2, j3, j4, j5, j6]   # radians
time_from_start = 5.0                           # seconds to reach position

[storage1-1]
joints          = [...]
time_from_start = 4.0
```

Named positions follow the slot naming convention: `<zone><row>-<col>`
(e.g. `storage1-1`, `proposal2-3`, `accepted1-2`, `pre_storage1-1`).

Record positions with:
```bash
ros2 run collarobot_actions record_position <name> [--time SECS]
```

### `gestures.toml`
Located at `~/collarobot_ws/src/collarobot_actions/collarobot_actions/gestures.toml`

Each gesture is a list of joint waypoints + cumulative timestamps. Available gestures: `wiggle`, `circle`.

### `ingredients_mapping.toml`
Located at `~/collarobot_ws/src/collarobot_controller/data/ingredients_mapping.toml`

```toml
[storage_slots]
6  = "storage1-1"   # beans
7  = "storage1-2"   # beef
# ...
```

Maps ArUco marker ID → fixed storage slot. The corresponding slot must be recorded in `positions.toml`.

---

## Zone / Slot Layout

See [`collarobot_controller/OPERATOR_GUIDE.md`](collarobot_controller/OPERATOR_GUIDE.md) for the full zone reference.

| Zone       | Robot (rows 1–4)           | Human (rows 5–6)           |
|------------|----------------------------|----------------------------|
| `storage`  | fixed per ingredient       | —                          |
| `proposal` | robot places here          | human places here          |
| `accepted` | robot places here          | human places here          |

Slot format: `<zone><row>-<col>` — e.g. `proposal1-1`, `accepted5-2`

---

## Pick-and-Place Sequence

```
goal received (ingredient_id, destination)
  │
  ▼  motion_coordinator_node resolves slots
  ├─ pick_slot  = current ingredient position
  └─ place_slot = next free slot in destination zone
  │
  ▼  pick_place_node executes
pre_pick → PICK (gripper close) → pre_pick → home
→ pre_place → PLACE (gripper open) → pre_place → home
```

`pre_<slot>` positions fall back to `home` if not recorded. The arm always returns to `home` after each pick-and-place so the next move starts from a known pose.

---

## CLI Tools Reference

| Command                                 | Description                                                        |
|-----------------------------------------|--------------------------------------------------------------------|
| `record_position <name> [--time SECS]`  | Capture current joints + carriage/lift to `positions.toml`        |
| `record_gesture <name>`                 | Record gesture waypoints interactively                             |
| `goto_position <name>`                  | Move arm to a named position (one-shot)                            |
| `goto_carriage_lift <name>`             | Move carriage/lift to a named position's recorded values           |
| `test_gripper <value> [--effort N]`     | Send a single gripper command and report the result                |

---

## ROS Interfaces

| Type          | Name                                             | Message type                              |
|---------------|--------------------------------------------------|-------------------------------------------|
| Action server | `/collarobot/pick_place`                         | `collarobot_msgs/action/PickPlace`        |
| Action server | `/collarobot/gesture`                            | `collarobot_msgs/action/Gesture`          |
| Action server | `/collarobot/move_ingredient`                    | `collarobot_msgs/action/MoveIngredient`   |
| Publisher     | `/joint_trajectory_controller/joint_trajectory`  | `trajectory_msgs/JointTrajectory`         |
| Action client | `/robotiq_gripper_controller/gripper_cmd`        | `control_msgs/action/GripperCommand`      |
| Subscriber    | `/joint_states`                                  | `sensor_msgs/JointState`                  |
| Subscriber    | `/elmo/id2/carriage/position/get`                | `std_msgs/Float32`                        |
| Subscriber    | `/elmo/id2/lift/position/get`                    | `std_msgs/Float32`                        |
| Publisher     | `/elmo/id2/carriage/position/set`                | `std_msgs/Float32`                        |
| Publisher     | `/elmo/id2/lift/position/set`                    | `std_msgs/Float32`                        |
| Publisher     | `/collarobot/state`                              | `std_msgs/String` (JSON)                  |

---

## Manual Testing

```bash
# Move arm to a specific ingredient slot
ros2 action send_goal /collarobot/move_ingredient \
    collarobot_msgs/action/MoveIngredient '{ingredient_id: 6, destination: "proposal"}'

# Trigger a gesture
ros2 action send_goal --feedback /collarobot/gesture \
    collarobot_msgs/action/Gesture '{gesture_name: "wiggle", return_position: "home"}'

# Direct pick-and-place (bypasses coordinator)
ros2 action send_goal --feedback /collarobot/pick_place \
    collarobot_msgs/action/PickPlace '{pick_position: "storage1-1", place_position: "proposal1-1"}'
```
