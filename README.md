# Hexabot

A 3D-printed hexapod robot: hexagonal body with 6 radially mounted legs
(60° apart), 3 servos per leg (coxa/femur/tibia), 18 servos total, driven
by an ELEGOO UNO R3 through a PCA9685 16-channel PWM driver.

## Hardware

- **Body**: 3D-printed hexagonal plate, 227 × 186 mm and 65 mm tall
  (measured from `Body.3mf`), with six identical servo pockets around the
  perimeter ~113 mm from center, one per corner.
- **Per leg**: Miuzei MS24 270° digital servo for the coxa (hip yaw,
  vertical axis) + two 20 kg-class 270° metal-gear digital servos for the
  femur and tibia (pitch, horizontal axes). Measured from the print files:
  femur link 125 mm (~105 mm axis-to-axis), tibia part 100 mm (~140 mm
  knee-to-foot-tip with the foot).
- **Stance**: spider-like inverted-V — hip up to the knee (femur +30°
  above horizontal), then steeply down to the foot (tibia −90° off the
  femur line), foot landing ~161 mm outboard of the hip with ~69 mm of
  ground clearance below the femur axis.
- **Controller**: ELEGOO UNO R3 (Arduino Uno clone) + PCA9685 16-channel
  I2C PWM servo driver (address 0x40).

Part dimensions come from `Cad files/(C) measure_parts.py`, which prints
the bounding box of every STL/3MF in `Cad files/3D print files/` — re-run
it after reprinting parts and re-derive the gait constants if lengths
change.

Left-side legs are mirror images of right-side legs; the code handles this
with a per-leg direction-sign table.

## Servo channel map

The PCA9685 has 16 channels but the robot has 18 servos, so 5 legs live on
the board (one leg per 3 consecutive channels) and the left rear leg runs
directly off Uno pins D9/D10/D11.

| PCA9685 channel | Leg | Joint |
|---|---|---|
| CH 0 | Right front (RF) | Coxa |
| CH 1 | Right front (RF) | Femur |
| CH 2 | Right front (RF) | Tibia |
| CH 3 | Right mid (RM) | Coxa |
| CH 4 | Right mid (RM) | Femur |
| CH 5 | Right mid (RM) | Tibia |
| CH 6 | Right rear (RR) | Coxa |
| CH 7 | Right rear (RR) | Femur |
| CH 8 | Right rear (RR) | Tibia |
| CH 9 | Left front (LF) | Coxa |
| CH 10 | Left front (LF) | Femur |
| CH 11 | Left front (LF) | Tibia |
| CH 12 | Left mid (LM) | Coxa |
| CH 13 | Left mid (LM) | Femur |
| CH 14 | Left mid (LM) | Tibia |
| CH 15 | — | Spare |

| Uno pin | Leg | Joint |
|---|---|---|
| D9 | Left rear (LR) | Coxa |
| D10 | Left rear (LR) | Femur |
| D11 | Left rear (LR) | Tibia |

### PCA9685 ↔ Uno wiring

| PCA9685 pin | Connects to |
|---|---|
| VCC | Uno 5V (logic power only) |
| GND | Uno GND |
| SDA | Uno A4 |
| SCL | Uno A5 |
| V+ (screw terminal) | External 5–6.8 V high-current servo supply **+** |
| GND (screw terminal) | External servo supply **−** (shares ground with the Uno) |

**Never** power the servos from the Uno's 5V pin or USB — see Power below.

## Sketches

Both sketches need the **Adafruit PWM Servo Driver Library** (Arduino IDE →
Library Manager → search "Adafruit PWM Servo Driver").

| Sketch | Purpose |
|--------|---------|
| [`Hexapod/`](Hexapod/) | **The main sketch.** Full 6-leg framework: neutral standing pose on boot, tripod-gait walking (forward / backward / crab left-right / turn in place) driven over Serial, plus a live per-joint calibration UI. See its README for controls, gait math, and tuning. |
| [`RightFrontLeg/`](RightFrontLeg/) | Original single-leg test: drives one leg (PCA9685 channels 0/1/2) through a Bezier-curve triangular step cycle, one cycle per `r` over Serial. Kept as a standalone testbed for a freshly built leg. |

Both sketches drive the 270° servos with 500–2500 µs pulses via
`writeMicroseconds()` (the PCA9685's, or `Servo.h`'s for the three Uno-pin
joints), because degree-based `write()` APIs clamp to 0–180° and can't reach
the servos' full travel.

## Build status

One leg (right front) is assembled; it plugs into PCA9685 channels 0–2. The
`Hexapod` sketch ships with only that leg enabled; flip each leg's
`enabled` flag in its config table as it gets built.

## Power — read this before wiring

18× 20 kg servos draw well over 10 A under load. USB / the Uno's 5V pin
cannot power them:

- Feed a dedicated 5–6.8 V high-current supply into the PCA9685's V+ screw
  terminal, with grounds common to the Uno.
- The PCA9685 board's terminal block and traces are only rated for roughly
  5–10 A total. For the full 18-servo robot, don't run all servo current
  through the little board — distribute power (e.g., a separate heavy-gauge
  5–6.8 V bus bar feeding the servo +/− wires, with only signal pins going
  to the PCA9685) or at minimum solder heavier feed wires to the rail.
- Put a large electrolytic capacitor (1000 µF+) across the servo rail to
  absorb start-up surges.
