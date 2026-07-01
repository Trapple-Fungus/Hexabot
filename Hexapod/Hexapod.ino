#include <Servo.h>

// ============================================================
// Hexapod: 6 legs x 3 joints (coxa/femur/tibia), 18 servos.
//
// On boot every enabled leg moves to the neutral standing pose
// and holds it. Serial commands then drive a tripod gait —
// forward / backward / crab left / crab right / turn — plus a
// live calibration UI for tuning per-joint offsets.
//
// Tripod gait: legs are split into two groups of three that
// always contain 2 legs from one side + 1 from the other:
//   Tripod A = RF, RR, LM    Tripod B = RM, LF, LR
// One tripod swings through the air toward its next foothold
// while the other pushes on the ground, then they swap. The
// body is always supported by a stable 3-point triangle.
//
// Servo backend note: Arduino's Servo.h tops out at 12 channels
// on an Uno (single 16-bit timer), so this sketch can drive at
// most 12 of the 18 joints at once. All pulse output funnels
// through the ServoBackend class so an 18-channel driver
// (PCA9685) or a Mega can replace it by editing only that class.
// ============================================================

// ---- 270-degree servo pulse range ----
// Servo::write() clamps to 0-180, so logical degrees are mapped to
// microseconds directly and sent with writeMicroseconds(). Typical
// values for 270-degree hobby servos; narrow the range if a servo
// buzzes or stalls at either end.
const int SERVO_MIN_US     = 500;
const int SERVO_MAX_US     = 2500;
const int SERVO_MAX_DEG    = 270;
const int SERVO_CENTER_DEG = 135;  // logical 0 deg maps here

const int NUM_LEGS   = 6;
const int NUM_JOINTS = 3;
enum Joint { COXA = 0, FEMUR = 1, TIBIA = 2 };

// ---- Logical angle convention (degrees) ----
//   coxa  : 0 = leg pointing radially outward from its body corner,
//           positive = sweeps toward the robot's front
//   femur : 0 = horizontal, positive = knee lifts up
//   tibia : 0 = in line with the femur, negative = bends down toward
//           the ground
struct LegPose {
  float coxa;
  float femur;
  float tibia;
};

// Standing pose from the reference photo: coxa centered radially
// outward, femur ~+35 deg above horizontal (knee is the highest point),
// tibia ~-65 deg off the femur line so the foot sits below and outboard
// of the body.
const LegPose NEUTRAL = { 0, 35, -65 };

// ---- Per-leg configuration ----
// sign flips a joint's direction: left-side legs are mirror images of
// right-side legs, so their femur/tibia (and coxa "toward front")
// rotate the opposite way for the same logical angle.
// offsetDeg absorbs each individual servo's mechanical zero error —
// tune it with the serial UI below, then copy the printed values back
// into this table so they persist across uploads.
struct JointConfig {
  uint8_t pin;
  int8_t  sign;       // +1 or -1
  int16_t offsetDeg;
};

struct LegConfig {
  const char* name;
  bool enabled;       // only attach/drive legs that are physically built
  JointConfig joint[NUM_JOINTS];
};

LegConfig legs[NUM_LEGS] = {
  // name  enabled   coxa          femur         tibia
  { "RF",  true,  {{ 9, +1, 0}, {10, +1, 0}, {11, +1, 0}} },
  { "RM",  false, {{ 2, +1, 0}, { 3, +1, 0}, { 4, +1, 0}} },
  { "RR",  false, {{ 5, +1, 0}, { 6, +1, 0}, { 7, +1, 0}} },
  { "LF",  false, {{ 8, -1, 0}, {12, -1, 0}, {13, -1, 0}} },
  { "LM",  false, {{A0, -1, 0}, {A1, -1, 0}, {A2, -1, 0}} },
  { "LR",  false, {{A3, -1, 0}, {A4, -1, 0}, {A5, -1, 0}} },
};

// Tripod A = RF, RR, LM (legs 0, 2, 4) — 2 right + 1 left.
// Tripod B = RM, LF, LR (legs 1, 3, 5) — 1 right + 2 left.
const bool TRIPOD_A[NUM_LEGS] = { true, false, true, false, true, false };

// ---- Gait tuning ----
const float STRIDE_DEG = 25;  // coxa half-sweep at full stride factor
const float LIFT_FEMUR = 25;  // extra femur lift at mid-swing (foot up)
const float LIFT_TIBIA = 20;  // extra tibia at mid-swing; positive folds
                              // the foot up toward the femur line — flip
                              // the sign if your foot digs down instead
const int   GAIT_STEPS = 30;  // interpolation steps per half-cycle
const int   STEP_MS    = 15;  // ms per step (speed of the gait)

// ---- Per-leg stride factors ----
// The legs point radially outward (60 deg apart), so a coxa sweep moves
// each foot along a different real-world arc. Each motion direction
// scales every leg's coxa sweep by the projection of that direction
// onto the leg's arc: k = cos(angle between desired travel and the
// leg's tangent). Corner legs sit 60 deg off the fore-aft axis
// (cos 60 = 0.5) and 30 deg off the lateral axis (cos 30 = 0.87);
// mid legs are fully fore-aft (1.0 forward, 0 sideways).
//
// Sign convention: positive k means the foot lands toward the robot's
// front at swing end, then pushes rearward during stance.
//                                      RF     RM    RR     LF     LM    LR
const float K_FORWARD[NUM_LEGS] = {  0.50f, 1.0f, 0.50f, 0.50f, 1.0f, 0.50f };
const float K_RIGHT[NUM_LEGS]   = { -0.87f, 0.0f, 0.87f, 0.87f, 0.0f, -0.87f };
const float K_TURN_R[NUM_LEGS]  = { -1.0f, -1.0f, -1.0f,  1.0f, 1.0f,  1.0f };

enum Motion { STOP, FORWARD, BACKWARD, CRAB_LEFT, CRAB_RIGHT, TURN_LEFT, TURN_RIGHT };
Motion motion = STOP;

float strideFactor(Motion m, uint8_t li) {
  switch (m) {
    case FORWARD:    return  K_FORWARD[li];
    case BACKWARD:   return -K_FORWARD[li];
    case CRAB_RIGHT: return  K_RIGHT[li];
    case CRAB_LEFT:  return -K_RIGHT[li];
    case TURN_RIGHT: return  K_TURN_R[li];
    case TURN_LEFT:  return -K_TURN_R[li];
    default:         return 0;
  }
}

// ---- Servo backend ----
// The single swap point for the 18-channel hardware decision: replace
// the body of these three methods with PCA9685 calls (or recompile for
// a Mega and raise MAX_CHANNELS) and nothing above this layer changes.
class ServoBackend {
public:
  void begin() {
    for (int l = 0; l < NUM_LEGS; l++)
      for (int j = 0; j < NUM_JOINTS; j++)
        handle[l][j] = -1;
    used = 0;
  }

  // Writes initialUs before attaching so the servo's very first pulse
  // is the target pose instead of an uncommanded jump.
  bool attach(uint8_t leg, uint8_t j, uint8_t pin, int initialUs) {
    if (used >= MAX_CHANNELS) return false;
    handle[leg][j] = used;
    pool[used].writeMicroseconds(initialUs);
    pool[used].attach(pin, SERVO_MIN_US, SERVO_MAX_US);
    used++;
    return true;
  }

  void writeUs(uint8_t leg, uint8_t j, int us) {
    int8_t h = handle[leg][j];
    if (h >= 0) pool[h].writeMicroseconds(us);
  }

private:
  static const uint8_t MAX_CHANNELS = 12;  // Servo.h limit on the Uno
  Servo   pool[MAX_CHANNELS];
  int8_t  handle[NUM_LEGS][NUM_JOINTS];
  uint8_t used;
};

ServoBackend backend;
LegPose currentPose[NUM_LEGS];

int jointToUs(const JointConfig &jc, float logicalDeg) {
  float servoDeg = SERVO_CENTER_DEG + jc.sign * logicalDeg + jc.offsetDeg;
  servoDeg = constrain(servoDeg, 0, SERVO_MAX_DEG);
  return map((int)round(servoDeg), 0, SERVO_MAX_DEG, SERVO_MIN_US, SERVO_MAX_US);
}

void applyLegPose(uint8_t li, const LegPose &p) {
  currentPose[li] = p;
  if (!legs[li].enabled) return;
  backend.writeUs(li, COXA,  jointToUs(legs[li].joint[COXA],  p.coxa));
  backend.writeUs(li, FEMUR, jointToUs(legs[li].joint[FEMUR], p.femur));
  backend.writeUs(li, TIBIA, jointToUs(legs[li].joint[TIBIA], p.tibia));
}

void applyAllLegs(const LegPose &p) {
  for (uint8_t li = 0; li < NUM_LEGS; li++)
    applyLegPose(li, p);
}

LegPose lerpPose(const LegPose &a, const LegPose &b, float t) {
  LegPose r;
  r.coxa  = a.coxa  + (b.coxa  - a.coxa)  * t;
  r.femur = a.femur + (b.femur - a.femur) * t;
  r.tibia = a.tibia + (b.tibia - a.tibia) * t;
  return r;
}

// Eases velocity at the start/end of each phase instead of an instant
// start/stop, which is the main thing that shock-loads the servos.
float easeInOutCubic(float t) {
  return (t < 0.5f) ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
}

// ---- Gait engine ----
// One half-cycle: the swinging tripod lifts and arcs to its forward
// foothold while the stance tripod stays down and pushes the body.
// Every leg interpolates from wherever it currently is to its target,
// so starting from neutral, stopping, and changing direction at a
// half-cycle boundary are all smooth — no assumed start positions.
void runHalfCycle(bool aSwings, Motion m) {
  LegPose start[NUM_LEGS], target[NUM_LEGS];
  for (uint8_t li = 0; li < NUM_LEGS; li++) {
    start[li]  = currentPose[li];
    target[li] = NEUTRAL;
    float dir = (TRIPOD_A[li] == aSwings) ? +1 : -1;  // swing lands front, stance pushes rear
    target[li].coxa += dir * strideFactor(m, li) * STRIDE_DEG;
  }

  for (int i = 0; i <= GAIT_STEPS; i++) {
    float t = easeInOutCubic((float)i / GAIT_STEPS);
    // Parabolic lift bump: 0 at both ends of the swing, max at mid-swing
    // (this IS the quadratic Bezier arc from the single-leg sketch, in
    // the degenerate case where both ground endpoints share a height).
    float bump = 4 * t * (1 - t);
    for (uint8_t li = 0; li < NUM_LEGS; li++) {
      LegPose p = lerpPose(start[li], target[li], t);
      if (TRIPOD_A[li] == aSwings) {
        p.femur += LIFT_FEMUR * bump;
        p.tibia += LIFT_TIBIA * bump;
      }
      applyLegPose(li, p);
    }
    delay(STEP_MS);
  }
}

void returnToNeutral() {
  LegPose start[NUM_LEGS];
  for (uint8_t li = 0; li < NUM_LEGS; li++)
    start[li] = currentPose[li];
  for (int i = 0; i <= GAIT_STEPS; i++) {
    float t = easeInOutCubic((float)i / GAIT_STEPS);
    for (uint8_t li = 0; li < NUM_LEGS; li++)
      applyLegPose(li, lerpPose(start[li], NEUTRAL, t));
    delay(STEP_MS);
  }
  Serial.println(F("Stopped — neutral stance."));
}

// ---- Serial UI ----
// Motion (single characters, Serial Monitor at 9600 baud):
//   w/s      walk forward / backward
//   a/d      crab-walk left / right
//   q/e      turn in place left / right
//   x        stop (settle back to neutral stance)
// Calibration:
//   0-5      select leg (0=RF 1=RM 2=RR 3=LF 4=LM 5=LR)
//   c f t    select joint on the selected leg
//   + -      nudge the selected joint's offset by +/-2 deg (moves live)
//   n        re-apply the neutral stance to all enabled legs
//   p        print the offset table (copy values back into legs[])
//   h        print this help
uint8_t selLeg   = 0;
uint8_t selJoint = COXA;

const char* jointName(uint8_t j) {
  return j == COXA ? "coxa" : (j == FEMUR ? "femur" : "tibia");
}

void printHelp() {
  Serial.println(F("Motion:  w fwd | s back | a crab left | d crab right"));
  Serial.println(F("         q turn left | e turn right | x stop"));
  Serial.println(F("Calibrate: 0-5 leg | c/f/t joint | +/- nudge 2deg"));
  Serial.println(F("           n neutral | p print offsets | h help"));
}

void printSelection() {
  Serial.print(F("Selected: leg "));
  Serial.print(legs[selLeg].name);
  Serial.print(F(" / "));
  Serial.print(jointName(selJoint));
  Serial.print(F("  offset="));
  Serial.println(legs[selLeg].joint[selJoint].offsetDeg);
}

void printOffsets() {
  Serial.println(F("Offsets (coxa, femur, tibia):"));
  for (int li = 0; li < NUM_LEGS; li++) {
    Serial.print(legs[li].name);
    Serial.print(legs[li].enabled ? F("  ") : F(" (disabled) "));
    for (int j = 0; j < NUM_JOINTS; j++) {
      Serial.print(legs[li].joint[j].offsetDeg);
      Serial.print(j < NUM_JOINTS - 1 ? F(", ") : F("\n"));
    }
  }
}

void nudgeOffset(int delta) {
  legs[selLeg].joint[selJoint].offsetDeg += delta;
  applyLegPose(selLeg, currentPose[selLeg]);  // re-send with new offset
  printSelection();
}

void setMotion(Motion m, const __FlashStringHelper* name) {
  motion = m;
  Serial.print(F("Motion: "));
  Serial.println(name);
}

void handleCommand(char c) {
  // Motion keys
  if      (c == 'w' || c == 'W') setMotion(FORWARD,    F("forward"));
  else if (c == 's' || c == 'S') setMotion(BACKWARD,   F("backward"));
  else if (c == 'a' || c == 'A') setMotion(CRAB_LEFT,  F("crab left"));
  else if (c == 'd' || c == 'D') setMotion(CRAB_RIGHT, F("crab right"));
  else if (c == 'q' || c == 'Q') setMotion(TURN_LEFT,  F("turn left"));
  else if (c == 'e' || c == 'E') setMotion(TURN_RIGHT, F("turn right"));
  else if (c == 'x' || c == 'X') setMotion(STOP,       F("stop"));
  // Calibration keys
  else if (c >= '0' && c <= '5') { selLeg = c - '0'; printSelection(); }
  else if (c == 'c' || c == 'C') { selJoint = COXA;  printSelection(); }
  else if (c == 'f' || c == 'F') { selJoint = FEMUR; printSelection(); }
  else if (c == 't' || c == 'T') { selJoint = TIBIA; printSelection(); }
  else if (c == '+') { nudgeOffset(+2); }
  else if (c == '-') { nudgeOffset(-2); }
  else if (c == 'n' || c == 'N') {
    applyAllLegs(NEUTRAL);
    Serial.println(F("Neutral stance applied."));
  }
  else if (c == 'p' || c == 'P') { printOffsets(); }
  else if (c == 'h' || c == 'H') { printHelp(); }
  // anything else (newlines etc.) is ignored
}

void setup() {
  Serial.begin(9600);
  backend.begin();

  int attached = 0, skipped = 0;
  for (uint8_t li = 0; li < NUM_LEGS; li++) {
    currentPose[li] = NEUTRAL;
    if (!legs[li].enabled) continue;
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      int us = jointToUs(legs[li].joint[j], j == COXA ? NEUTRAL.coxa
                       : j == FEMUR ? NEUTRAL.femur : NEUTRAL.tibia);
      if (backend.attach(li, j, legs[li].joint[j].pin, us)) attached++;
      else skipped++;
    }
  }

  Serial.print(F("Hexapod ready. Joints attached: "));
  Serial.println(attached);
  if (skipped > 0) {
    Serial.print(F("WARNING: "));
    Serial.print(skipped);
    Serial.println(F(" joints skipped — Servo.h supports only 12 channels"));
    Serial.println(F("on the Uno. An 18-channel driver (PCA9685) or a Mega"));
    Serial.println(F("is needed for the full robot."));
  }
  printHelp();
}

bool aSwings = true;  // which tripod swings on the next half-cycle

void loop() {
  while (Serial.available() > 0)
    handleCommand(Serial.read());

  if (motion == STOP) return;  // holding the current stance

  // Latch the motion for this half-cycle so a direction change can't
  // yank the stride factors mid-air; new commands take effect at the
  // next half-cycle boundary, when all six feet are on the ground.
  Motion m = motion;
  runHalfCycle(aSwings, m);
  aSwings = !aSwings;

  if (motion == STOP) returnToNeutral();
}
