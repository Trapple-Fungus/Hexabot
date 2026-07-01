# Hexabot

A 3D-printed hexapod robot: hexagonal body with 6 radially mounted legs
(60° apart), 3 servos per leg (coxa/femur/tibia), 18 servos total, driven
by an ELEGOO UNO R3.

## Hardware

- **Body**: 3D-printed hexagonal plate with six identical servo pockets
  around the perimeter, one per corner.
- **Per leg**: Miuzei MS24 270° digital servo for the coxa (hip yaw,
  vertical axis) + two 20 kg-class 270° metal-gear digital servos for the
  femur and tibia (pitch, horizontal axes). Femur ~100–120 mm, tibia
  ~130–150 mm to the foot tip.
- **Stance**: spider-like inverted-V — hip up to the knee (femur ~+35°
  above horizontal), then steeply down to the foot (tibia ~−65° off the
  femur line), foot landing outboard of the body.
- **Controller**: ELEGOO UNO R3 (Arduino Uno clone).

Left-side legs are mirror images of right-side legs; the code handles this
with a per-leg direction-sign table.

## Sketches

| Sketch | Purpose |
|--------|---------|
| [`Hexapod/`](Hexapod/) | **The main sketch.** Full 6-leg framework: neutral standing pose on boot, tripod-gait walking (forward / backward / crab left-right / turn in place) driven over Serial, plus a live per-joint calibration UI. See its README for controls, gait math, and tuning. |
| [`RightFrontLeg/`](RightFrontLeg/) | Original single-leg test: drives one leg (pins 9/10/11) through a Bezier-curve triangular step cycle, one cycle per `r` over Serial. Kept as a standalone testbed for a freshly built leg. |

Both sketches drive the 270° servos with `writeMicroseconds()` over a
500–2500 µs range, because Arduino's `Servo::write()` clamps to 0–180° and
can't reach the servos' full travel.

## Build status

One leg (right front) is assembled and wired to pins D9/D10/D11. The
`Hexapod` sketch ships with only that leg enabled; flip each leg's
`enabled` flag in its config table as it gets built.

## Known constraints for the full robot

1. **Servo count**: Arduino's `Servo.h` supports at most 12 servos on an
   Uno — the full 18 needs a PCA9685 16-channel PWM driver board (planned;
   the code isolates this behind a single `ServoBackend` class) or a Mega.
2. **Power**: 18× 20 kg servos draw well over 10 A under load. USB / the
   Uno's 5V pin cannot power them — a dedicated 5–6.8 V high-current supply
   on the servo rails (grounds common with the Uno) is required.
