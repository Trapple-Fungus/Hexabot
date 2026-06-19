# Right Front Leg — Bezier Walking Test

This sketch (`RightFrontLeg.ino`) drives a single hexapod leg (3 servos: coxa, femur,
tibia) through a triangular foot path using Bezier-curve motion, so the servos ease
in and out instead of snapping between positions.

## Hardware as described

- 3D-printed black leg assembly with a triangular truss link at the top.
- Two Miuzei 20kg digital servos (4.8V–6.8V) in series for the femur/tibia joints,
  plus a third servo for the coxa (hip rotation).
- ELEGOO UNO R3 (Arduino Uno clone) driving the servos, powered via USB.
- Servos wired to the Uno's digital pins; loose jumper wires for power/signal.

## Pin assignment

| Joint | Arduino pin |
|-------|-------------|
| Coxa  | D9          |
| Femur | D10         |
| Tibia | D11         |

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
`0` by default) are added to the logical angle before it's sent to `servo.write()`:

```cpp
int deg = (int)round(logicalDeg) + offset;
deg = constrain(deg, 0, 180);
```

If, say, your femur servo's true "vertical down" position is actually
`servo.write(75)` rather than `servo.write(0)`, set `FEMUR_OFFSET = 75`. Tune these
first, against the leg's real resting pose, before touching anything else —
the trajectory math always works purely in logical angles.

## How the motion works

The foot is driven around a triangle made of three corner poses (each a
coxa/femur/tibia angle triple), defined near the top of the file:

- `GROUND_BACK` — foot down, swept toward the rear. End of stance / start of swing.
- `LIFT_APEX` — foot raised, swept to the middle. The top corner of the triangle.
- `GROUND_FRONT` — foot down, swept toward the front. End of swing / start of stance.

```cpp
LegPose GROUND_BACK  = { 70,  25, 40 };
LegPose LIFT_APEX    = { 90,  70, 90 };
LegPose GROUND_FRONT = { 110, 25, 40 };
```

These starting values are **placeholders** — tune them once the leg is powered up
(see Tuning below).

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

1. Upload the sketch. On `setup()` it moves straight to `GROUND_BACK` and holds for
   1 second so you can check the resting pose before it starts moving.
2. Watch the swing phase — adjust `LIFT_APEX`'s femur/tibia angles until the foot
   clears the ground by the amount you want without overdriving the servo.
3. Adjust `GROUND_BACK` and `GROUND_FRONT`'s coxa angles to set how far the leg
   sweeps back and forward (stride length).
4. Adjust `STEP_DELAY_MS` and the `*_STEPS` constants to change gait speed/smoothness
   (more steps + lower delay-per-step = smoother but same total time; fewer steps =
   choppier).

## Next steps (not yet implemented)

This sketch only drives one leg. For full hexapod walking you'd run this same
swing/stance state machine on all 6 legs simultaneously, with the two tripod
groups (legs 1/3/5 and 2/4/6, or however your gait pairs them) running 180° out
of phase with each other — i.e., one tripod is in swing phase while the other is
in stance phase, then they swap.
