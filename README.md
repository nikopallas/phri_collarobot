# CollaboRobot — Pick-and-Place System

ROS2 Jazzy workspace for a **Kinova Gen3 6-DOF arm** with **Robotiq gripper** and **Elmo carriage/lift**.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Package Structure](#package-structure)
3. [Build](#build)
4. [Configuration — positions.toml](#configuration--positionstoml)
5. [Workflow](#workflow)
   - [Step 1 — Calibrate the Gripper](#step-1--calibrate-the-gripper)
   - [Step 2 — Set Up Carriage and Lift](#step-2--set-up-carriage-and-lift)
   - [Step 3 — Record Positions](#step-3--record-positions)
   - [Step 4 — Test Individual Moves](#step-4--test-individual-moves)
   - [Step 5 — Run Pick-and-Place](#step-5--run-pick-and-place)
6. [CLI Tools Reference](#cli-tools-reference)
7. [Pick-and-Place Sequence](#pick-and-place-sequence)
8. [ROS Topics and Actions](#ros-topics-and-actions)

---

## System Overview

```
┌─────────────────────────────────────────────────────┐
│                   Operator / Client                  │
│   ros2 action send_goal  /pick_place_node/pick_place │
└────────────────────┬────────────────────────────────┘
                     │ PickPlace action (goal / feedback / result)
                     ▼
         ┌───────────────────────┐
         │    pick_place_node    │
         │   (action server)     │
         │                       │
         │  pre-flight check     │◄── /elmo/id2/*/position/get
         │  ─────────────────    │
         │  PRE_PICK             │──► /joint_trajectory_controller/...
         │  PICKING ─ close grip │──► /robotiq_gripper_controller/...
         │  POST_PICK            │
         │  HOME                 │
         │  PRE_PLACE            │
         │  PLACING ─ open grip  │
         │  POST_PLACE           │
         └───────────────────────┘
```

The operator **manually positions the carriage and lift** before triggering the sequence. The node verifies they are in the correct position before executing any motion.

---

## Package Structure

```
phri_collarobot/
├── collarobot_msgs/               # Custom ROS2 interfaces
│   └── action/
│       └── PickPlace.action       # Action definition
│
└── collarobot_actions/            # All robot behaviour
    └── collarobot_actions/
        ├── positions.toml         # Named positions (edit / record here)
        ├── pick_place_node.py     # Main action server
        ├── record_position.py     # CLI: capture current robot state to TOML
        ├── goto_position.py       # CLI: move arm to a named position
        ├── goto_carriage_lift.py  # CLI: move carriage/lift to recorded values
        └── test_gripper.py        # CLI: test gripper at a specific position value
```

---

## Build

`collarobot_msgs` must be built **before** `collarobot_actions` because it generates the `PickPlace` action type.

```bash
cd ~/collarobot_ws
git pull

colcon build --packages-select collarobot_msgs --symlink-install
source install/setup.bash

colcon build --packages-select collarobot_actions --symlink-install
source install/setup.bash
```

> `--symlink-install` symlinks Python source files so edits to `.py` files take effect immediately without rebuilding. TOML and data files are still copied on build — always edit the source file at `~/collarobot_ws/src/collarobot_actions/collarobot_actions/positions.toml`.

---

## Configuration — positions.toml

All named positions are stored in a single TOML file:

```
~/collarobot_ws/src/collarobot_actions/collarobot_actions/positions.toml
```

### Schema

```toml
# Gripper calibration (0.0 = fully open, 1.0 = fully closed)
gripper_open  = 0.0
gripper_close = 0.65          # set this after calibrating with test_gripper

[home]
joints          = [0.0, -0.35, 0.0, -1.57, 0.0, 0.0]
time_from_start = 5.0         # seconds the arm gets to reach this position
carriage        = 1.2500      # optional — used for pre-flight check
lift            = 0.3000      # optional — used for pre-flight check

[pre_pick]
joints          = [...]
time_from_start = 4.0

[pick]
joints          = [...]
time_from_start = 2.5

[pre_place]
joints          = [...]
time_from_start = 4.0

[place]
joints          = [...]
time_from_start = 2.5
```

### Positions used in the sequence

| Name        | Role                         | Notes                                                |
| ----------- | ---------------------------- | ---------------------------------------------------- |
| `pre_pick`  | Approach height above object | Also used as **retreat** after picking               |
| `pick`      | At the object, gripper open  | Gripper closes here                                  |
| `home`      | Safe neutral transit pose    | Arm passes through here between pick and place zones |
| `pre_place` | Approach height above target | Also used as **retreat** after placing               |
| `place`     | At the place target          | Gripper opens here                                   |

> `post_pick` and `post_place` are not separate positions — the arm simply retraces back to `pre_pick` / `pre_place`.

### Carriage and lift in `[home]`

If `carriage` and/or `lift` are recorded in the `[home]` entry, `pick_place_node` will **verify** the current position of each axis before starting the sequence. The tolerance is `±0.05` (same units as `/elmo/id2/*/position/get`). If either axis is out of tolerance the action is aborted with a clear error message telling the operator what to adjust.

---

## Workflow

### Step 1 — Calibrate the Gripper

Find the right closing value for your object using `test_gripper`:

```bash
ros2 run collarobot_actions test_gripper 0.50
ros2 run collarobot_actions test_gripper 0.60
```

The result shows `reached_goal`, final `position`, and `effort`. When it grips firmly without stalling, update `gripper_close` in `positions.toml`:

```toml
gripper_close = 0.62
```

---

### Step 2 — Set Up Carriage and Lift

Move the carriage and lift to the required position manually or using `goto_carriage_lift`:

```bash
# Command carriage+lift to the values stored for a named position
ros2 run collarobot_actions goto_carriage_lift home
```

The tool publishes the setpoint, then waits until `/elmo/id2/*/position/get` confirms arrival within tolerance. It exits once both axes are in position (or after a 30 s timeout).

---

### Step 3 — Record Positions

Move the robot to each position (e.g. by jogging or gravity compensation mode), then capture the state:

```bash
ros2 run collarobot_actions record_position home      --time 5
ros2 run collarobot_actions record_position pre_pick  --time 4
ros2 run collarobot_actions record_position pick      --time 2.5
ros2 run collarobot_actions record_position pre_place --time 4
ros2 run collarobot_actions record_position place     --time 2.5
```

Each call:

1. Reads the current `/joint_states`
2. Waits 0.5 s for carriage/lift topics to deliver a value (saves them if available)
3. Writes the entry to `positions.toml`, overwriting only that position

The `--time` value is how many seconds the arm controller is allowed to reach that position during the sequence. Set it generously — it is better to wait a little longer than to send the next command while the arm is still moving.

---

### Step 4 — Test Individual Moves

Verify each recorded position looks correct before running the full sequence:

```bash
ros2 run collarobot_actions goto_position home
ros2 run collarobot_actions goto_position pre_pick
ros2 run collarobot_actions goto_position pick
# etc.
```

---

### Step 5 — Run Pick-and-Place

**Terminal 1** — start the action server:

```bash
ros2 run collarobot_actions pick_place_node
```

**Terminal 2** — trigger the sequence with live step feedback:

```bash
ros2 action send_goal --feedback /pick_place_node/pick_place \
    collarobot_msgs/action/PickPlace '{}'
```

Feedback prints the current step name as the arm moves through the sequence. The result reports `success: True` on completion, or `success: False` with a reason on failure.

To cancel mid-sequence press `Ctrl+C` in the terminal running `send_goal`.

---

## CLI Tools Reference

| Command                                | Description                                                         |
| -------------------------------------- | ------------------------------------------------------------------- |
| `record_position <name> [--time SECS]` | Capture current joints + carriage/lift to TOML                      |
| `goto_position <name>`                 | Move arm to a named position (exits when done)                      |
| `goto_carriage_lift <name>`            | Move carriage/lift to a named position's values (waits for arrival) |
| `test_gripper <value> [--effort N]`    | Send a single gripper command and report the result                 |

---

## Pick-and-Place Sequence

```
IDLE
  │
  ▼  [action goal received — carriage/lift pre-flight check passes]
PRE_PICK      → arm moves to pre_pick
  │
  ▼
PICKING       → arm moves to pick  →  gripper CLOSES
  │
  ▼
POST_PICK     → arm retreats to pre_pick  (same position as PRE_PICK)
  │
  ▼
HOME          → arm moves to home  (safe transit between zones)
  │
  ▼
PRE_PLACE     → arm moves to pre_place
  │
  ▼
PLACING       → arm moves to place  →  gripper OPENS
  │
  ▼
POST_PLACE    → arm retreats to pre_place  (same position as PRE_PLACE)
  │
  ▼
IDLE          [action result: success: True]
```

Any exception at any step immediately aborts the action with `success: False` and a message describing what failed and where.

---

## ROS Topics and Actions

| Type          | Name                                            | Message                           | Direction                       |
| ------------- | ----------------------------------------------- | --------------------------------- | ------------------------------- |
| Action server | `/pick_place_node/pick_place`                   | `collarobot_msgs/PickPlace`       | in                              |
| Publisher     | `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | out                             |
| Action client | `/robotiq_gripper_controller/gripper_cmd`       | `control_msgs/GripperCommand`     | out                             |
| Subscriber    | `/joint_states`                                 | `sensor_msgs/JointState`          | in                              |
| Subscriber    | `/elmo/id2/carriage/position/get`               | `std_msgs/Float32`                | in                              |
| Subscriber    | `/elmo/id2/lift/position/get`                   | `std_msgs/Float32`                | in                              |
| Publisher     | `/elmo/id2/carriage/position/set`               | `std_msgs/Float32`                | out (`goto_carriage_lift` only) |
| Publisher     | `/elmo/id2/lift/position/set`                   | `std_msgs/Float32`                | out (`goto_carriage_lift` only) |
