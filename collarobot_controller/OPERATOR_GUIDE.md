# CollaboRobot — Operator Slot Guide

## Zone overview

The workspace is divided into named **zones**, each split into **rows** and **columns**.

| Zone       | Rows 1–4 (robot)         | Rows 5–6 (human)         |
|------------|--------------------------|--------------------------|
| `storage`  | fixed per ingredient     | —                        |
| `proposal` | robot places here        | human places here        |
| `accepted` | robot places here        | human places here        |

Slot name format: `<zone><row>-<col>`

Examples: `proposal1-1`, `proposal5-2`, `accepted2-1`, `storage1-3`

---

## Current recording status

### Recorded — real hardware positions

| Positions                                               | Notes                                        |
|---------------------------------------------------------|----------------------------------------------|
| `home`                                                  | arm home / rest pose                         |
| `storage1-1` .. `storage2-3`                            | 6 ingredient storage slots (rows 1–2)        |
| `pre_storage1-1` .. `pre_storage2-3`                    | approach heights for storage rows 1–2        |
| `proposal1-1` .. `proposal2-3`                          | 6 robot proposal slots (rows 1–2)            |
| `pre_proposal1-1`                                       | approach height for proposal1-1 only         |
| `accepted1-1`, `accepted1-2`, `accepted2-1`, `accepted2-2` | 4 robot accepted slots                   |

### Simulated — dummy values, must be re-recorded before hardware use

| Positions                                               | Notes                                        |
|---------------------------------------------------------|----------------------------------------------|
| `proposal5-1` .. `proposal6-3`                          | 6 human proposal slots (rows 5–6)            |
| `accepted5-1`, `accepted5-2`, `accepted6-1`, `accepted6-2` | 4 human accepted slots                   |

### Missing — not yet recorded

| Positions                                               | Notes                                        |
|---------------------------------------------------------|----------------------------------------------|
| `storage3-1` .. `storage6-3`                            | storage rows 3–6 (12 slots, ingredients 12–22) |
| `storage7-1`                                            | tomato (ingredient 24, ArUco ID 24)          |
| `pre_proposal1-2`, `pre_proposal1-3`, `pre_proposal2-*` | fall back to `home` until recorded           |
| `pre_accepted*`                                         | fall back to `home` until recorded           |

---

## Slot filling order (human zones)

When you place an ingredient on the table, the system assigns it to the **next
free human slot in order**:

```
proposal5-1  →  proposal5-2  →  proposal5-3
proposal6-1  →  proposal6-2  →  proposal6-3
```

Once a slot is freed (robot picks from it, or you remove it), **that slot becomes
available again and will be filled first** on the next placement.

### Example session

| Event                           | Slot assigned       | Human slots state                  |
|---------------------------------|---------------------|------------------------------------|
| You place ingredient A          | `proposal5-1`       | [A, _, _, _, _, _]                 |
| You place ingredient B          | `proposal5-2`       | [A, B, _, _, _, _]                 |
| Robot picks A → accepted zone   | `proposal5-1` freed | [_, B, _, _, _, _]                 |
| You place ingredient C          | `proposal5-1`       | [C, B, _, _, _, _]                 |

---

## Recording positions

Move the arm to the target position (jogging or gravity compensation mode), then:

```bash
ros2 run collarobot_actions record_position <name> [--time SECS]
```

`--time` is how many seconds the arm controller gets to reach this position during execution.
Set it generously — better to wait slightly longer than to issue the next command while
the arm is still moving.

### Priority recording list

```bash
# Human proposal slots (rows 5–6) — overwrite the dummy values
ros2 run collarobot_actions record_position proposal5-1 --time 2
ros2 run collarobot_actions record_position proposal5-2 --time 2
ros2 run collarobot_actions record_position proposal5-3 --time 2
ros2 run collarobot_actions record_position proposal6-1 --time 2
ros2 run collarobot_actions record_position proposal6-2 --time 2
ros2 run collarobot_actions record_position proposal6-3 --time 2

# Human accepted slots (rows 5–6) — overwrite the dummy values
ros2 run collarobot_actions record_position accepted5-1 --time 2
ros2 run collarobot_actions record_position accepted5-2 --time 2
ros2 run collarobot_actions record_position accepted6-1 --time 2
ros2 run collarobot_actions record_position accepted6-2 --time 2

# Missing storage slots (rows 3–6 + row 7 for tomato)
ros2 run collarobot_actions record_position storage3-1 --time 2
# ... storage3-2, storage3-3, storage4-1 ... storage6-3, storage7-1
```

---

## Quick reference: slot counts

| Zone       | Robot (rows 1–4) | Human (rows 5–6) | Total |
|------------|------------------|------------------|-------|
| `storage`  | up to 21 (3 col) | —                | 21    |
| `proposal` | up to 12 (3 col) | 6 (3 col)        | 18    |
| `accepted` | up to 8 (2 col)  | 4 (2 col)        | 12    |
