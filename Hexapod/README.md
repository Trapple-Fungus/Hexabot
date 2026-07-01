# Hexapod — Tripod Gait + Stance Framework

`Hexapod.ino` drives the full robot: 6 legs × 3 joints (coxa/femur/tibia),
18 servos total. On boot, every **enabled** leg moves to the neutral standing
pose from the reference photo and holds it. Serial commands then walk the
robot **forward, backward, sideways (crab), or turn in place** using a tripod
gait, and a calibration UI lets you tune each joint's mechanical offset live.

## ⚠ Hardware limits you must know about

1. **Servo.h maxes out at 12 channels on the Uno.** The sketch compiles and
   runs, but can only physically drive 12 of the 18 joints at once. Joints
   past the limit are skipped with a warning at boot. For the full robot you
   need either a PCA9685 16-channel I2C driver board (+2 joints on Uno pins,
   or a second PCA9685) or an Arduino Mega. All pulse output goes through the
   `ServoBackend` class — that is the **single swap point** when the hardware
   is chosen; nothing above it changes.
2. **Power.** 18× 20 kg-class servos can pull well over 10 A under load. USB
   and the Uno's 5V pin cannot supply this — even a few servos will brown-out
   the board. Use a dedicated 5–6.8 V high-current supply wired directly to
   the servo power rails, with grounds common to the Uno.

## Serial controls (9600 baud, single characters)

### Motion

| Key | Action |
|-----|--------|
| `w` / `s` | Walk forward / backward |
| `a` / `d` | Crab-walk left / right (body translates sideways, no turning) |
| `q` / `e` | Turn in place left / right |
| `x` | Stop — settles smoothly back to the neutral stance |

Motion continues until you send `x` or a different direction. Direction
changes take effect at the next half-cycle boundary (when all six feet are
on the ground), so the robot never gets yanked mid-step.

### Calibration

| Key | Action |
|-----|--------|
| `0`–`5` | Select leg (0=RF 1=RM 2=RR 3=LF 4=LM 5=LR) |
| `c` / `f` / `t` | Select coxa / femur / tibia on the selected leg |
| `+` / `-` | Nudge the selected joint's offset ±2° (servo moves immediately) |
| `n` | Re-apply the neutral stance to all enabled legs |
| `p` | Print the full offset table |
| `h` | Help |

Workflow: upload → legs move to neutral → nudge each joint until the physical
pose matches the photo → press `p` → copy the printed offsets into the
`offsetDeg` fields in `legs[]` → re-upload. Offsets then persist.

## How the tripod gait works

The six legs are split into two tripods, each with **2 legs from one side and
1 from the other**, so the body always rests on a stable triangle:

- **Tripod A** = RF, RR, LM (legs 0, 2, 4)
- **Tripod B** = RM, LF, LR (legs 1, 3, 5)

Each half-cycle, one tripod **swings** (feet lift in a parabolic arc and land
at their next foothold) while the other tripod is in **stance** (feet stay
down and push the body). Then the roles swap. Both phases run through an
ease-in-out cubic so the servos accelerate and decelerate smoothly instead of
jerking.

Every leg interpolates from *wherever it currently is* to its target, so
starting from standstill, stopping, and switching direction are all smooth —
there are no assumed start positions.

### Why sideways walking needs per-leg stride factors

The legs are mounted **radially** (60° apart, pointing outward), so a coxa
sweep moves each foot along a different real-world direction — the arc is
always perpendicular to the direction the leg points. The gait scales each
leg's coxa sweep by the projection of the desired travel direction onto that
leg's arc (`k = cos` of the angle between them):

| Motion | RF | RM | RR | LF | LM | LR |
|--------|----|----|----|----|----|----|
| Forward | 0.50 | 1.00 | 0.50 | 0.50 | 1.00 | 0.50 |
| Crab right | −0.87 | 0 | 0.87 | 0.87 | 0 | −0.87 |
| Turn right | −1 | −1 | −1 | 1 | 1 | 1 |

(Backward, crab left, and turn left are the same tables negated.)

Reading the crab-right row: the front and rear legs' arcs point 30° off the
lateral axis (cos 30° ≈ 0.87), and their fore-aft components cancel between
the front and rear pair, leaving pure sideways push. The mid legs' arcs are
purely fore-aft, so they can't contribute — during crab walking they just
lift and replant in place, acting as support. Turning sweeps all feet in the
same rotational sense, which is opposite signs on opposite sides.

### Gait tuning constants (top of the sketch)

| Constant | Default | Meaning |
|----------|---------|---------|
| `STRIDE_DEG` | 25 | Coxa half-sweep at full stride factor — stride length |
| `LIFT_FEMUR` | 25 | Extra femur lift at mid-swing — foot clearance |
| `LIFT_TIBIA` | 20 | Extra tibia fold at mid-swing (flip sign if the foot digs down instead of tucking up) |
| `GAIT_STEPS` | 30 | Interpolation steps per half-cycle — smoothness |
| `STEP_MS` | 15 | ms per step — overall speed (half-cycle ≈ 450 ms) |

## Leg layout and pins

Left legs are mirror images of right legs, handled by the `sign` field (−1)
in the config table.

| # | Leg | Coxa | Femur | Tibia | Tripod | Enabled by default |
|---|-----|------|-------|-------|--------|--------------------|
| 0 | RF (right front) | D9 | D10 | D11 | A | ✅ (matches the single-leg build) |
| 1 | RM (right mid)   | D2 | D3  | D4  | B | ❌ |
| 2 | RR (right rear)  | D5 | D6  | D7  | A | ❌ |
| 3 | LF (left front)  | D8 | D12 | D13 | B | ❌ |
| 4 | LM (left mid)    | A0 | A1  | A2  | A | ❌ |
| 5 | LR (left rear)   | A3 | A4  | A5  | B | ❌ |

As you build each leg, wire it to its pins and flip its `enabled` flag to
`true` in `legs[]`. (Analog pins A0–A5 work fine as digital servo outputs.)

## Logical angle convention

All poses are written in body-frame logical degrees; the per-leg `sign` and
`offsetDeg` translate them to raw servo positions.

- **Coxa**: `0°` = leg pointing radially outward from its body corner.
  Positive sweeps the leg toward the robot's front.
- **Femur**: `0°` = horizontal. Positive lifts the knee up.
- **Tibia**: `0°` = in line with the femur (leg straight). Negative bends
  the foot down toward the ground.

The neutral stance (from the photo — knee up, foot below and outboard):

```cpp
const LegPose NEUTRAL = { 0, 35, -65 };  // coxa, femur, tibia
```

Mapping to a servo: `servoDeg = 135 + sign × logicalDeg + offsetDeg`, then
converted to a 500–2500 µs pulse across 270° and sent with
`writeMicroseconds()` (Servo's `write()` clamps to 180° and can't use these
servos' full range).

## Bring-up order

1. With only RF enabled, verify the neutral stance and calibrate its offsets.
2. Send `w` and watch RF step through the gait in the air — sanity-check the
   swing direction and foot clearance before other legs exist.
3. Build and enable the remaining legs one at a time, calibrating each.
4. Pick the 18-channel hardware (PCA9685 recommended) and swap the
   `ServoBackend` internals — until then Servo.h silently caps you at 12
   joints (4 legs).
5. First full-robot walk: prop the body on a box so the feet hang free, run
   each motion, and only then put it on the ground.
