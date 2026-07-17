# Hexapod — Tripod Gait + Stance Framework

`Hexapod.ino` drives the full robot: 6 legs × 3 joints (coxa/femur/tibia),
18 servos total. On boot, every **enabled** leg moves to the neutral standing
pose (sized against the CAD-measured leg segments) and holds it. Serial
commands then walk the robot **forward, backward, sideways (crab), or turn in
place** using a tripod gait with Bezier-curve swing paths, and a calibration
UI lets you tune each joint's mechanical offset live.

## Hardware setup

The servos are driven by a **PCA9685 16-channel I2C PWM board** (default
address 0x40) wired to the Uno: VCC→5V, GND→GND, SDA→A4, SCL→A5. Since the
robot has 18 servos and the board 16 channels, five legs sit on PCA9685
channels 0–14 (one leg per 3 consecutive channels, CH15 spare) and the LR
leg runs on Uno pins D9–D11 through `Servo.h`. All pulse output goes through
the `ServoBackend` class — that is the **single swap point** if the hardware
changes (second PCA9685, Mega); nothing above it changes.

The sketch needs the **Adafruit PWM Servo Driver Library** (Arduino IDE →
Library Manager → search "Adafruit PWM Servo Driver").

### ⚠ Power

18× 20 kg-class servos can pull well over 10 A under load. USB and the Uno's
5V pin cannot supply this — even a few servos will brown-out the board. Feed
a dedicated 5–6.8 V high-current supply into the PCA9685's V+ screw terminal
(grounds common to the Uno), and for the full robot distribute the servo
current instead of running it all through the board's ~5–10 A-rated terminal
block and traces — see the power section of the [main README](../README.md).

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
| `<` / `>` | Coarse nudge ±10° (for large horn-mount offsets) |
| `j` | Wiggle the selected joint to identify it physically |
| `v` | Move to the **calibration pose**: femur + tibia hang plumb straight down, knee locked straight |
| `z` | **Diagnostic scan**: wiggle every enabled joint in sequence, announcing each over Serial |
| `n` | Ease back to the neutral standing stance |
| `p` | Print the full offset table |
| `h` | Help |

Workflow: prop the body **at least 25 cm** off the bench (the feet hang
~245 mm below the hip axis in the calibration pose) → press `v` → for each
leg, nudge the **femur** until it hangs plumb vertical, the **tibia** until
it continues dead straight in line with it, and the **coxa** (checked from
directly above) until the leg points straight out from its body corner →
press `p` → copy the printed offsets into the `offsetDeg` fields in
`legs[]` → re-upload. Offsets are pose-independent, so values tuned in the
hanging pose are exactly right for standing and walking.

If a knee physically can't straighten all the way (bracket hits the femur),
don't force it — calibrate that tibia against a 90° square at the neutral
stance (`n`) instead.

## How the tripod gait works

The six legs are split into two tripods, each with **2 legs from one side and
1 from the other**, so the body always rests on a stable triangle:

- **Tripod A** = RF, RR, LM (legs 0, 2, 4)
- **Tripod B** = RM, LF, LR (legs 1, 3, 5)

Each half-cycle, one tripod **swings** — each foot arcs to its next foothold
along a **quadratic Bezier curve** (start → lifted apex → target, with the
control point inflated so the curve actually passes through the apex at
mid-swing, same as the single-leg sketch) — while the other tripod is in
**stance** (feet stay loaded on the ground and push the body in a straight
line). Then the roles swap. Both phases run their timing through an
ease-in-out cubic. The curved path plus the eased timing means the servos
never see a velocity discontinuity at lift-off or touch-down — that's what
keeps shock loads off the gear trains.

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

Defaults are derived from the CAD-measured leg geometry (femur ~105 mm
axis-to-axis, tibia+foot ~140 mm, hip ~113 mm from body center → foot
~274 mm from center at neutral).

| Constant | Default | Meaning |
|----------|---------|---------|
| `STRIDE_DEG` | 20 | Coxa half-sweep at full stride factor — ~187 mm/step for mid legs; keeps adjacent feet ≥140 mm apart at full stride |
| `LIFT_FEMUR` | 10 | Femur lift at the swing apex — together with `LIFT_TIBIA` the foot rises ~39 mm |
| `LIFT_TIBIA` | 6 | Tibia fold at the swing apex (flip sign if the foot digs down instead of tucking up) |
| `GAIT_STEPS` | 30 | Interpolation steps per half-cycle — smoothness |
| `STEP_MS` | 15 | ms per step — overall speed (half-cycle ≈ 450 ms) |

## Leg layout and channels

The six legs are identical printed assemblies mounted radially — rotated
around the body, not mirrored. So femur/tibia rotate the same physical way
on both sides (`sign = +1` everywhere) and only the **coxa** direction flips
on the left (`sign = −1`), because "sweep toward the front" reverses when
the leg points the other way. Each leg occupies 3 consecutive PCA9685
channels; the LR leg overflows the 16-channel board onto Uno pins.

| # | Leg | Coxa | Femur | Tibia | Tripod | Enabled by default |
|---|-----|------|-------|-------|--------|--------------------|
| 0 | RF (right front) | CH0 | CH1  | CH2  | A | ✅ |
| 1 | RM (right mid)   | CH3 | CH4  | CH5  | B | ✅ |
| 2 | RR (right rear)  | CH6 | CH7  | CH8  | A | ✅ |
| 3 | LF (left front)  | CH9 | CH10 | CH11 | B | ✅ |
| 4 | LM (left mid)    | CH12 | CH13 | CH14 | A | ✅ |
| 5 | LR (left rear)   | Uno D9 | Uno D10 | Uno D11 | B | ✅ |

CH15 is spare. All six legs are enabled in `legs[]`; set a leg's `enabled`
flag to `false` if it isn't built or wired yet. The full channel-by-channel
table and the PCA9685↔Uno wiring are in the [main README](../README.md).

## If some legs don't move

The sketch sends pulses to **every enabled joint on every gait step** — a leg
that never moves is a hardware problem, not a gait problem. Diagnose in this
order:

1. **Check the boot banner.** The Serial Monitor must say `Hexapod fw build 3`.
   If it says `Leg ready. Press 'r'...` you are running the old single-leg
   `RightFrontLeg.ino` test sketch — which only ever drives CH0–CH2. Upload
   `Hexapod.ino` instead.
2. **Check the boot I2C scan line.** It must list `0x40`. If it doesn't, the
   PCA9685 is miswired or strapped to a different address, and every PCA
   channel is dead.
3. **Send `z`.** Every joint wiggles in sequence with its name and channel
   printed. A servo that stays still on its turn: wrong channel plug, dead
   servo, or no servo power on V+.
4. **If joints move one at a time (`z`) but sag or freeze when walking**, the
   servo supply is browning out under the combined load — see the power
   section of the main README.

## Logical angle convention

All poses are written in body-frame logical degrees; the per-leg `sign` and
`offsetDeg` translate them to raw servo positions.

- **Coxa**: `0°` = leg pointing radially outward from its body corner.
  Positive sweeps the leg toward the robot's front.
- **Femur**: `0°` = horizontal. Positive lifts the knee up.
- **Tibia**: `0°` = in line with the femur (leg straight). Negative bends
  the foot down toward the ground.

The neutral stance (knee up, foot below and outboard), sized against the
measured leg segments for a **3 in (76 mm) belly clearance**:

```cpp
const LegPose NEUTRAL = { 0, 15, -90 };  // coxa, femur, tibia
```

With femur ≈ 105 mm and tibia+foot ≈ 140 mm this puts the foot ~138 mm
outboard of the hip and **~108 mm below the femur axis**; with the femur
axis at the 65 mm body's mid-height, the body plate rides ~76 mm ≈ 3 in
off the ground. (The previous `{0, 30, −90}` stance stood ~36 mm.)

Calibrate the offsets first, then measure the real standing height —
the trim knob is the femur angle: **1° less femur ≈ 2.4 mm taller**, and
vice versa. The tibia angle changes height much more slowly (~0.6 mm/°)
at this pose, so leave it at −90 unless the stance width needs changing.

Mapping to a servo: `servoDeg = 135 + sign × logicalDeg + offsetDeg`, then
converted to a 500–2500 µs pulse across 270° and sent as microseconds to
the PCA9685 (or via `Servo.writeMicroseconds()` for the LR leg's Uno pins) —
degree-based `write()` APIs clamp to 180° and can't use these servos' full
range.

## Bring-up order

1. With only RF enabled, verify the neutral stance and calibrate its offsets.
2. Send `w` and watch RF step through the gait in the air — sanity-check the
   swing direction and foot clearance before other legs exist.
3. Build and enable the remaining legs one at a time, calibrating each.
4. Before enabling more than a couple of legs, sort out servo power: a
   dedicated 5–6.8 V high-current supply on the PCA9685's V+ terminal, and
   distributed power wiring for the full 18-servo load (see the main
   README's power section).
5. First full-robot walk: prop the body on a box so the feet hang free, run
   each motion, and only then put it on the ground.
