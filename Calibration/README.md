# Calibration — Static Reference Pose

`Calibration.ino` does exactly one thing: on power-up it drives **all 18
servos to the calibration pose and holds it**, so you can walk around the
robot and instantly see which joints are right and which are off.

Every leg should look identical:

| Joint | Target |
|-------|--------|
| Coxa  | Leg points **straight out** from its body corner (check from directly above) |
| Femur | Hanging **plumb straight down** (compare against a hanging string or the body's vertical side) |
| Tibia | **Dead straight** in line with the femur — the whole leg is one vertical line, no kink at the knee |

## ⚠ Before powering up

- **Prop the body at least 25 cm off the bench.** The legs extend fully
  downward — feet hang ~245 mm below the hip axis. On the desk, the robot
  will do a push-up and fall over.
- Servos snap to the pose at full speed the moment power arrives (there is
  no soft-start on a cold boot — the first pulse a servo sees *is* the
  target). Keep fingers clear.
- Same wiring as the Hexapod sketch: PCA9685 at 0x40 (SDA=A4, SCL=A5),
  legs on CH0–14, LR leg on Uno D9–D11, servo power on V+.

## Fixing a joint that's off

The `legs[]` table (channels, signs, `offsetDeg`) is **identical in layout
to `Hexapod.ino`'s** — keep the two in sync:

1. Small error (a few degrees → ~25°): find the offset. Either tune it live
   in the Hexapod sketch (`v` pose + `+`/`-` keys, then `p`), or edit
   `offsetDeg` here, re-upload, and look again. Repeat until plumb.
2. Large error (> ~40°): re-mount the servo horn a spline tooth or two
   instead — the horns only mount in ~14.4° steps, and big offsets eat the
   servo's usable travel in one direction.
3. When the pose is perfect, copy the final `offsetDeg` values into
   `Hexapod.ino`'s `legs[]` so walking uses them too.

The Serial Monitor (9600 baud) prints the exact pulse width sent to every
joint at boot — useful for confirming a servo that isn't moving is at least
being commanded.
