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

| Event                           | Slot assigned   | Human slots state                  |
|---------------------------------|-----------------|------------------------------------|
| You place ingredient A          | `proposal5-1`   | [A, _, _, _, _, _]                 |
| You place ingredient B          | `proposal5-2`   | [A, B, _, _, _, _]                 |
| Robot picks A → accepted zone   | `proposal5-1` freed | [_, B, _, _, _, _]             |
| You place ingredient C          | `proposal5-1`   | [C, B, _, _, _, _]                 |

---

## Recording human zone positions (required once per setup)

The robot needs a recorded arm position for each human slot before it can pick
from there.  Run these commands **once** after placing the arm at each slot:

```bash
# Move arm to the pick position above proposal5-1, then:
ros2 run collarobot_actions record_position proposal5-1

# Repeat for each slot the robot needs to reach:
ros2 run collarobot_actions record_position proposal5-2
ros2 run collarobot_actions record_position proposal5-3
ros2 run collarobot_actions record_position proposal6-1
ros2 run collarobot_actions record_position proposal6-2
ros2 run collarobot_actions record_position proposal6-3

ros2 run collarobot_actions record_position accepted5-1
ros2 run collarobot_actions record_position accepted5-2
ros2 run collarobot_actions record_position accepted6-1
ros2 run collarobot_actions record_position accepted6-2
```

Until a slot is recorded, **tracking still works** — the system knows the
ingredient is in that slot — but the robot will refuse to pick from it and log
an error.

---

## Robot zone positions (rows 1–4)

These are recorded during initial calibration and should already be in
`positions.toml`:

| Zone       | Slots                                                      |
|------------|------------------------------------------------------------|
| `proposal` | proposal1-1, proposal1-2, proposal1-3, proposal2-1, ...   |
| `accepted` | accepted1-1, accepted1-2, accepted2-1, accepted2-2, ...   |

---

## Quick reference: slot counts

| Zone       | Robot (rows 1–4) | Human (rows 5–6) | Total |
|------------|------------------|------------------|-------|
| `proposal` | up to 12 (3 col) | 6 (3 col)        | 18    |
| `accepted` | up to 8 (2 col)  | 4 (2 col)        | 12    |
