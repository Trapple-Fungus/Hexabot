#include <Servo.h>

// ============================================================
// Single-leg test sketch: right-front coxa/femur/tibia leg.
// Drives the foot through a triangular path (lift -> swing
// forward -> ground contact -> drag back) using a quadratic
// Bezier curve for the swing phase, so the servos accelerate
// and decelerate smoothly instead of snapping between angles.
// ============================================================

// ---- Pin assignment ----
const int COXA_PIN  = 9;
const int FEMUR_PIN = 10;
const int TIBIA_PIN = 11;

Servo coxa, femur, tibia;

// ---- Calibration offsets ----
// Logical angle convention used throughout this sketch:
//   coxa  : 90 deg = leg pointing straight out (as in the reference photo)
//   femur : 0  deg = vertically down
//   tibia : 0  deg = in line with the femur (leg fully extended)
// If your servo's physical zero doesn't actually land on that pose,
// adjust the offset here rather than touching the trajectory math below.
const int COXA_OFFSET  = 0;
const int FEMUR_OFFSET = 0;
const int TIBIA_OFFSET = 0;

struct LegPose {
  float coxa;
  float femur;
  float tibia;
};

void writeJoint(Servo &s, float logicalDeg, int offset) {
  int deg = (int)round(logicalDeg) + offset;
  deg = constrain(deg, 0, 180);
  s.write(deg);
}

void applyPose(const LegPose &p) {
  writeJoint(coxa, p.coxa, COXA_OFFSET);
  writeJoint(femur, p.femur, FEMUR_OFFSET);
  writeJoint(tibia, p.tibia, TIBIA_OFFSET);
}

LegPose lerpPose(const LegPose &a, const LegPose &b, float t) {
  LegPose r;
  r.coxa  = a.coxa  + (b.coxa  - a.coxa)  * t;
  r.femur = a.femur + (b.femur - a.femur) * t;
  r.tibia = a.tibia + (b.tibia - a.tibia) * t;
  return r;
}

LegPose bezierPose(const LegPose &p0, const LegPose &p1, const LegPose &p2, float t) {
  float u = 1.0f - t;
  LegPose r;
  r.coxa  = u * u * p0.coxa  + 2 * u * t * p1.coxa  + t * t * p2.coxa;
  r.femur = u * u * p0.femur + 2 * u * t * p1.femur + t * t * p2.femur;
  r.tibia = u * u * p0.tibia + 2 * u * t * p1.tibia + t * t * p2.tibia;
  return r;
}

// Eases velocity at the start/end of each phase instead of an instant
// start/stop, which is the main thing that shock-loads small servos.
float easeInOutCubic(float t) {
  return (t < 0.5f) ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
}

// ---- Triangle corner poses (tune these to your leg's real range) ----
// GROUND_BACK  : foot down, swept toward the rear  -> end of stance / start of swing
// LIFT_APEX    : foot raised, swept to the middle  -> top corner of the triangle
// GROUND_FRONT : foot down, swept toward the front -> end of swing / start of stance
LegPose GROUND_BACK  = { 70,  25, 40 };
LegPose LIFT_APEX    = { 90,  70, 90 };
LegPose GROUND_FRONT = { 110, 25, 40 };

const int SWING_STEPS   = 30;  // resolution of the Bezier swing arc
const int STANCE_STEPS  = 30;  // resolution of the ground push-back
const int STEP_DELAY_MS = 15;  // time between steps -> overall cycle speed

void setup() {
  coxa.attach(COXA_PIN);
  femur.attach(FEMUR_PIN);
  tibia.attach(TIBIA_PIN);
  applyPose(GROUND_BACK);
  delay(1000);
}

void loop() {
  // ---- Swing phase: Bezier arc GROUND_BACK -> LIFT_APEX -> GROUND_FRONT ----
  // A plain quadratic Bezier only gets pulled toward its control point,
  // it doesn't pass through it. Inflating the control point like this
  // makes the curve actually hit LIFT_APEX at t = 0.5.
  LegPose control;
  control.coxa  = 2 * LIFT_APEX.coxa  - 0.5f * (GROUND_BACK.coxa  + GROUND_FRONT.coxa);
  control.femur = 2 * LIFT_APEX.femur - 0.5f * (GROUND_BACK.femur + GROUND_FRONT.femur);
  control.tibia = 2 * LIFT_APEX.tibia - 0.5f * (GROUND_BACK.tibia + GROUND_FRONT.tibia);

  for (int i = 0; i <= SWING_STEPS; i++) {
    float t = easeInOutCubic((float)i / SWING_STEPS);
    applyPose(bezierPose(GROUND_BACK, control, GROUND_FRONT, t));
    delay(STEP_DELAY_MS);
  }

  // ---- Stance phase: straight ground drag GROUND_FRONT -> GROUND_BACK ----
  // This is the part of the cycle where the foot is loaded and pushing
  // the body forward, so it stays low and moves in a straight line
  // instead of another curve.
  for (int i = 0; i <= STANCE_STEPS; i++) {
    float t = easeInOutCubic((float)i / STANCE_STEPS);
    applyPose(lerpPose(GROUND_FRONT, GROUND_BACK, t));
    delay(STEP_DELAY_MS);
  }
}
