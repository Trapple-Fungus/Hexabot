# Right Front Leg — Bezier Walking Test

This sketch (`RightFrontLeg.ino`) drives a single hexapod leg (3 servos: coxa, femur,
tibia) through a triangular foot path using Bezier-curve motion, so the servos ease
in and out instead of snapping between positions.

> **Note:** full-robot walking now lives in [`../Hexapod/`](../Hexapod/), which
> handles all 6 legs with a tripod gait (forward/backward/crab/turn). This sketch
> is kept as a standalone testbed for exercising a freshly built leg in isolation.

## Hardware as described

- 3D-printed black leg assembly with a triangular truss link at the top.
- Two Miuzei 20kg digital servos (4.8V–6.8V) in series for the femur/tibia joints,
  plus a third servo for the coxa (hip rotation).
- ELEGOO UNO R3 (Arduino Uno clone) + PCA9685 16-channel I2C PWM driver
  (VCC→5V, GND→GND, SDA→A4, SCL→A5, default address 0x40).
- Servo power comes from an external 5–6.8 V supply on the PCA9685's V+
  screw terminal (ground common with the Uno) — not from USB or the Uno's
  5V pin.
- Needs the **Adafruit PWM Servo Driver Library** (Arduino IDE → Library
  Manager).

## Channel assignment

The test channels match the RF leg's slots in the full Hexapod sketch, so a
freshly built leg plugs straight into the board. Change the `*_CH` constants
at the top of the sketch to test a leg on other channels.

| Joint | PCA9685 channel |
|-------|-----------------|
| Coxa  | CH0             |
| Femur | CH1             |
| Tibia | CH2             |

## Degree convention (logical angles used in the code)

These are the angles the code's math is written in terms of — they are **not**
necessarily the raw `servo.write()` degrees, see Calibration below.

- **Coxa**: `90°` = leg pointing straight out, matching the resting pose in the
  reference photo. Less than 90° sweeps the leg one way (toward the rear),
  more than 90° sweeps it the other way (toward the front).
- **Femur**: `0°` = pointing straight down (vertical). Increasing the angle lifts
  the femur up/forward.
- **Tibia**: `0°` = in line with the femur (leg fully extended/straight).
  Increasing the angle bends the knee joint.

## Calibration offsets

Real servos rarely have their mechanical zero land exactly on the logical zero
above. `COXA_OFFSET`, `FEMUR_OFFSET`, and `TIBIA_OFFSET` (top of the sketch, all
`0` by default) are added to the logical angle before it's sent to the servo:

```cpp
int deg = (int)round(logicalDeg) + offset;
deg = constrain(deg, 0, SERVO_MAX_DEG);
int us = map(deg, 0, SERVO_MAX_DEG, SERVO_MIN_US, SERVO_MAX_US);
s.writeMicroseconds(us);
```

If, say, your femur servo's true "vertical down" position is actually
75° rather than 0°, set `FEMUR_OFFSET = 75`. Tune these first, against the leg's
real resting pose, before touching anything else — the trajectory math always
works purely in logical angles.

### Why microsecond pulses instead of degree APIs

These are 270° servos, but degree-based servo APIs clamp their input to
0–180° before mapping it to a pulse width — they physically cannot reach past
180° no matter what you pass them. To use the full sweep, logical degrees
(0–270, `SERVO_MAX_DEG`) are mapped straight to a pulse width in microseconds
and sent with `pwm.writeMicroseconds(channel, us)` on the PCA9685.
`SERVO_MIN_US`/`SERVO_MAX_US` (default `500`/`2500`) are typical for 270°
hobby servos but aren't guaranteed for your exact model — if a servo buzzes,
stalls, or stops short of where you told it to go, narrow this range and
re-check against the datasheet. If all servos are consistently off by the
same proportion, the PCA9685's internal oscillator (nominally 27 MHz, but it
varies board to board) is the culprit — trim `PCA_OSC_HZ` at the top of the
sketch.

## How the motion works

The foot is driven around a triangle made of three corner poses (each a
coxa/femur/tibia angle triple), defined near the top of the file:

- `GROUND_BACK` — foot down, swept toward the rear. End of stance / start of swing.
- `LIFT_APEX` — foot raised, swept to the middle. The top corner of the triangle.
- `GROUND_FRONT` — foot down, swept toward the front. End of swing / start of stance.

```cpp
LegPose GROUND_BACK  = { 40,  50,  40 };
LegPose LIFT_APEX    = { 80,  265, 90 };
LegPose GROUND_FRONT = { 120, 50,  40 };
```

These values are still being tuned against the real leg — adjust them further
as needed (see Tuning below). Note the femur apex (`265°`) is right up against
the servo's `270°` hard limit, so there's very little headroom left before it
strains; back it off if the servo buzzes or stalls there.

Each full cycle (`loop()`) has two phases:

1. **Swing phase** — `GROUND_BACK → LIFT_APEX → GROUND_FRONT`, driven as a
   **quadratic Bezier curve** over `SWING_STEPS` (default 30) steps. This is the
   part where the foot is off the ground, lifting and arcing forward.
   - A plain quadratic Bezier only gets *pulled toward* its middle control point,
     it doesn't pass through it. The code compensates by inflating the control
     point (`control = 2*LIFT_APEX - 0.5*(GROUND_BACK + GROUND_FRONT)`) so the
     curve actually reaches `LIFT_APEX` at the midpoint (`t = 0.5`) of the swing.
2. **Stance phase** — `GROUND_FRONT → GROUND_BACK`, driven as a straight linear
   interpolation over `STANCE_STEPS` (default 30) steps. This is the part where
   the foot is loaded on the ground and (once the full hexapod is built) pushes
   the body forward — it intentionally stays low and straight rather than curving.

Both phases pass their progress `t` (0 → 1) through an **ease-in-out cubic**
function before interpolating:

```cpp
float easeInOutCubic(float t) {
  return (t < 0.5f) ? 4*t*t*t : 1 - pow(-2*t + 2, 3) / 2;
}
```

This is what actually protects the servos: it removes the instantaneous
start/stop velocity change at the beginning and end of each phase, so the servo
accelerates and decelerates smoothly instead of jerking. The Bezier shape gives
the *path* a curve; the easing gives the *timing along that path* a curve — both
together are what reduce mechanical stress.

`STEP_DELAY_MS` (default 15ms) sets the delay between each step and therefore the
overall speed of the gait cycle.

## Triggering the cycle with 'r'

The leg no longer runs continuously. On `setup()` it moves to `GROUND_BACK` and
then idles, holding that pose, until it receives the character `r` (or `R`) over
Serial — at which point it runs exactly one full swing+stance cycle and goes back
to idling.

To trigger it: open the Arduino IDE's Serial Monitor (Tools → Serial Monitor) at
9600 baud, type `r`, and hit Enter. Each press runs one more cycle. Any other
character is ignored. This is handled in `loop()` by `Serial.available()` /
`Serial.read()`, with the actual cycle logic factored out into `runGaitCycle()`.

## Tuning the leg

1. Upload the sketch. On `setup()` it moves straight to `GROUND_BACK` and holds
   there so you can check the resting pose before triggering any movement.
2. Watch the swing phase — adjust `LIFT_APEX`'s femur/tibia angles until the foot
   clears the ground by the amount you want without overdriving the servo.
3. Adjust `GROUND_BACK` and `GROUND_FRONT`'s coxa angles to set how far the leg
   sweeps back and forward (stride length).
4. Adjust `STEP_DELAY_MS` and the `*_STEPS` constants to change gait speed/smoothness
   (more steps + lower delay-per-step = smoother but same total time; fewer steps =
   choppier).

## Next steps

Full hexapod walking is implemented in [`../Hexapod/`](../Hexapod/): the same
swing/stance approach runs on all 6 legs with two tripod groups (RF/RR/LM and
RM/LF/LR) 180° out of phase — one tripod swings while the other pushes, then
they swap. Once a new leg passes its solo test with this sketch, plug it into
its PCA9685 channels from the Hexapod README's channel table and flip its
`enabled` flag there.
