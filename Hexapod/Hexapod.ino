#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>  // Adafruit PWM Servo Driver Library
#include <Servo.h>                    // fallback for the LR leg on Uno pins

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
// Servo backend: a PCA9685 16-channel I2C PWM driver (SDA=A4,
// SCL=A5, address 0x40) carries 15 of the 18 joints — one leg
// per 3 consecutive channels, CH0-CH14. The 18th-16th joint
// overflow (the whole LR leg) runs on Uno pins D9-D11 via
// Servo.h. All pulse output funnels through the ServoBackend
// class, so different hardware (a second PCA9685, a Mega) only
// touches that class.
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

// ---- PCA9685 setup ----
const int      SERVO_FREQ_HZ = 50;        // standard 20 ms servo frame
const uint32_t PCA_OSC_HZ    = 27000000;  // nominal internal oscillator — it
                                          // varies board to board, so trim this
                                          // if measured pulses run long/short

// ---- Leg geometry (measured from the CAD 3D-print files) ----
// Femur link (motor holder parts): 125 mm part, ~105 mm axis-to-axis.
// Tibia part: 100 mm, ~140 mm knee axis to foot tip with the foot on.
// Body: 227 x 186 mm hex plate, hip pockets ~113 mm from center.
// The sketch works purely in joint angles — these numbers are what the
// pose and gait constants below were derived from, so re-derive those
// if the printed parts change.
const float FEMUR_MM = 105;
const float TIBIA_MM = 140;

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

// Standing pose, sized against the measured leg (femur ~105 mm axis to
// axis, tibia+foot ~140 mm): knee up at +30 deg, tibia -90 deg off the
// femur line. That puts the foot ~161 mm outboard of the hip and ~69 mm
// below the femur axis — real ground clearance for the 65 mm-tall body.
// (The old photo-eyeballed pose {0,35,-65} computes to the foot ~10 mm
// ABOVE the hip axis with these segment lengths — belly on the floor.)
// Verify against the physical leg and trim with the calibration UI.
const LegPose NEUTRAL = { 0, 30, -90 };

// ---- Per-leg configuration ----
// sign flips a joint's direction: left-side legs are mirror images of
// right-side legs, so their femur/tibia (and coxa "toward front")
// rotate the opposite way for the same logical angle.
// offsetDeg absorbs each individual servo's mechanical zero error —
// tune it with the serial UI below, then copy the printed values back
// into this table so they persist across uploads.
struct JointConfig {
  uint8_t channel;    // PCA9685 channel (0-15), or Uno pin when onUno is set
  bool    onUno;      // 18 joints > 16 channels: the LR leg lives on Uno pins
  int8_t  sign;       // +1 or -1
  int16_t offsetDeg;
};

struct LegConfig {
  const char* name;
  bool enabled;       // only attach/drive legs that are physically built
  JointConfig joint[NUM_JOINTS];
};

// One leg per 3 consecutive PCA9685 channels; CH15 is spare. The LR
// leg overflows the 16-channel board onto Uno pins D9/D10/D11 (the
// same pins the original single-leg build used, so old wiring fits).
LegConfig legs[NUM_LEGS] = {
  // name  enabled   coxa                 femur                tibia
  { "RF",  true,  {{ 0, false, +1, 0}, { 1, false, +1, 0}, { 2, false, +1, 0}} },
  { "RM",  true,  {{ 3, false, +1, 0}, { 4, false, +1, 0}, { 5, false, +1, 0}} },
  { "RR",  true,  {{ 6, false, +1, 0}, { 7, false, +1, 0}, { 8, false, +1, 0}} },
  { "LF",  true,  {{ 9, false, -1, 0}, {10, false, -1, 0}, {11, false, -1, 0}} },
  { "LM",  true,  {{12, false, -1, 0}, {13, false, -1, 0}, {14, false, -1, 0}} },
  { "LR",  true,  {{ 9, true,  -1, 0}, {10, true,  -1, 0}, {11, true,  -1, 0}} },
};

// Tripod A = RF, RR, LM (legs 0, 2, 4) — 2 right + 1 left.
// Tripod B = RM, LF, LR (legs 1, 3, 5) — 1 right + 2 left.
const bool TRIPOD_A[NUM_LEGS] = { true, false, true, false, true, false };

// ---- Gait tuning (derived from the measured geometry) ----
const float STRIDE_DEG = 20;  // coxa half-sweep at full stride factor:
                              // ~187 mm/step for mid legs at the 274 mm
                              // foot radius, and adjacent feet stay
                              // >=140 mm apart at full stride (25 deg
                              // was 271 mm steps with only 107 mm gap)
const float LIFT_FEMUR = 10;  // swing-apex lift; with LIFT_TIBIA = 6 the
const float LIFT_TIBIA = 6;   // foot rises ~39 mm — clears the ground
                              // without wasting servo travel (the old
                              // 25/20 lifted ~120 mm every step). Tibia
                              // positive folds the foot up toward the
                              // femur line — flip the sign if your foot
                              // digs down instead
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
// The single swap point for the servo hardware: PCA9685 channels for
// most joints, a small Servo.h pool for the Uno-pin overflow (LR leg).
// A second PCA9685 or a Mega would only change the code in this class.
class ServoBackend {
public:
  void begin() {
    pwm.begin();
    pwm.setOscillatorFrequency(PCA_OSC_HZ);
    pwm.setPWMFreq(SERVO_FREQ_HZ);
    Wire.setClock(400000);  // 15 I2C writes per gait step; 100 kHz is too slow
    for (int l = 0; l < NUM_LEGS; l++)
      for (int j = 0; j < NUM_JOINTS; j++)
        unoHandle[l][j] = -1;
    unoUsed = 0;
  }

  // The first pulse a servo ever sees is the target pose, not an
  // uncommanded jump: PCA9685 channels are silent until first written,
  // and Uno-pin servos get writeMicroseconds() before attach().
  bool attach(uint8_t leg, uint8_t j, const JointConfig &jc, int initialUs) {
    if (!jc.onUno) {
      pwm.writeMicroseconds(jc.channel, initialUs);
      return true;
    }
    if (unoUsed >= MAX_UNO_SERVOS) return false;
    unoHandle[leg][j] = unoUsed;
    unoPool[unoUsed].writeMicroseconds(initialUs);
    unoPool[unoUsed].attach(jc.channel, SERVO_MIN_US, SERVO_MAX_US);
    unoUsed++;
    return true;
  }

  void writeUs(uint8_t leg, uint8_t j, const JointConfig &jc, int us) {
    if (!jc.onUno) {
      pwm.writeMicroseconds(jc.channel, us);
      return;
    }
    int8_t h = unoHandle[leg][j];
    if (h >= 0) unoPool[h].writeMicroseconds(us);
  }

private:
  Adafruit_PWMServoDriver pwm;             // default I2C address 0x40
  static const uint8_t MAX_UNO_SERVOS = 3; // just the LR leg overflow
  Servo   unoPool[MAX_UNO_SERVOS];
  int8_t  unoHandle[NUM_LEGS][NUM_JOINTS];
  uint8_t unoUsed;
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
  backend.writeUs(li, COXA,  legs[li].joint[COXA],  jointToUs(legs[li].joint[COXA],  p.coxa));
  backend.writeUs(li, FEMUR, legs[li].joint[FEMUR], jointToUs(legs[li].joint[FEMUR], p.femur));
  backend.writeUs(li, TIBIA, legs[li].joint[TIBIA], jointToUs(legs[li].joint[TIBIA], p.tibia));
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

// Quadratic Bezier through p0 / control p1 / p2 — the swing-phase path,
// same approach as the single-leg test sketch.
LegPose bezierPose(const LegPose &p0, const LegPose &p1, const LegPose &p2, float t) {
  float u = 1.0f - t;
  LegPose r;
  r.coxa  = u * u * p0.coxa  + 2 * u * t * p1.coxa  + t * t * p2.coxa;
  r.femur = u * u * p0.femur + 2 * u * t * p1.femur + t * t * p2.femur;
  r.tibia = u * u * p0.tibia + 2 * u * t * p1.tibia + t * t * p2.tibia;
  return r;
}

// Eases velocity at the start/end of each phase instead of an instant
// start/stop, which is the main thing that shock-loads the servos.
float easeInOutCubic(float t) {
  return (t < 0.5f) ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
}

// ---- Gait engine ----
// One half-cycle: the swinging tripod lifts and arcs to its forward
// foothold along a quadratic Bezier curve (start -> lifted apex ->
// target) while the stance tripod stays down and pushes the body in a
// straight line. The Bezier plus the ease-in-out timing means the
// servos see no velocity discontinuities at lift-off or touch-down —
// that's what keeps shock loads off the gears. Every leg interpolates
// from wherever it currently is to its target, so starting from
// neutral, stopping, and changing direction at a half-cycle boundary
// are all smooth — no assumed start positions.
void runHalfCycle(bool aSwings, Motion m) {
  LegPose start[NUM_LEGS], target[NUM_LEGS], control[NUM_LEGS];
  for (uint8_t li = 0; li < NUM_LEGS; li++) {
    start[li]  = currentPose[li];
    target[li] = NEUTRAL;
    float dir = (TRIPOD_A[li] == aSwings) ? +1 : -1;  // swing lands front, stance pushes rear
    target[li].coxa += dir * strideFactor(m, li) * STRIDE_DEG;

    // Swing apex = midpoint of the move plus the lift. A plain quadratic
    // Bezier only gets pulled toward its control point, it doesn't pass
    // through it, so the control point is inflated
    // (2*apex - 0.5*(start+target)) to make the curve actually hit the
    // apex at t = 0.5 — same trick as the single-leg sketch.
    LegPose apex = lerpPose(start[li], target[li], 0.5f);
    apex.femur += LIFT_FEMUR;
    apex.tibia += LIFT_TIBIA;
    control[li].coxa  = 2 * apex.coxa  - 0.5f * (start[li].coxa  + target[li].coxa);
    control[li].femur = 2 * apex.femur - 0.5f * (start[li].femur + target[li].femur);
    control[li].tibia = 2 * apex.tibia - 0.5f * (start[li].tibia + target[li].tibia);
  }

  for (int i = 0; i <= GAIT_STEPS; i++) {
    float t = easeInOutCubic((float)i / GAIT_STEPS);
    for (uint8_t li = 0; li < NUM_LEGS; li++) {
      // Swinging feet arc through the air on the Bezier; stance feet are
      // loaded on the ground, so they move low and straight instead.
      LegPose p = (TRIPOD_A[li] == aSwings)
                    ? bezierPose(start[li], control[li], target[li], t)
                    : lerpPose(start[li], target[li], t);
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
//   < >      coarse nudge by +/-10 deg (for large horn-mount offsets)
//   j        wiggle the selected joint to identify it physically
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
  Serial.println(F("Calibrate: 0-5 leg | c/f/t joint | +/- nudge 2deg | </> 10deg"));
  Serial.println(F("           j wiggle joint | n neutral | p print offsets | h help"));
  Serial.println(F("Diagnose:  z wiggle every joint in sequence (finds dead wiring)"));
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

// Wiggles one joint +/-15 deg, then settles back — the quick way to
// check that a servo is plugged into the channel the legs[] table says
// it is.
void wiggleOne(uint8_t li, uint8_t j, int cycles) {
  LegPose base = currentPose[li];
  const int steps = 20;
  for (int cycle = 0; cycle < cycles; cycle++) {
    for (int i = 0; i <= steps; i++) {
      float wave = 15.0f * sin(2 * PI * i / steps);
      LegPose p = base;
      if      (j == COXA)  p.coxa  += wave;
      else if (j == FEMUR) p.femur += wave;
      else                 p.tibia += wave;
      applyLegPose(li, p);
      delay(STEP_MS * 2);
    }
  }
  applyLegPose(li, base);
}

void wiggleJoint() {
  printSelection();
  wiggleOne(selLeg, selJoint, 3);
}

// Wiggles every enabled joint in sequence, announcing each over Serial.
// This is the first thing to run when legs aren't moving: a joint that
// stays still here has a wiring/power/channel problem, not a gait
// problem — the exact pulse the gait would send is being sent to it.
void scanAllJoints() {
  Serial.println(F("Joint scan — watch each named servo wiggle:"));
  for (uint8_t li = 0; li < NUM_LEGS; li++) {
    if (!legs[li].enabled) {
      Serial.print(legs[li].name);
      Serial.println(F(" skipped (disabled)"));
      continue;
    }
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      const JointConfig &jc = legs[li].joint[j];
      Serial.print(legs[li].name);
      Serial.print(' ');
      Serial.print(jointName(j));
      Serial.print(jc.onUno ? F(" (Uno D") : F(" (CH"));
      Serial.print(jc.channel);
      Serial.println(')');
      wiggleOne(li, j, 1);
    }
  }
  Serial.println(F("Scan done. Any joint that stayed still: check its plug,"));
  Serial.println(F("channel number, and servo power — the code did drive it."));
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
  else if (c == '>') { nudgeOffset(+10); }
  else if (c == '<') { nudgeOffset(-10); }
  else if (c == 'j' || c == 'J') { wiggleJoint(); }
  else if (c == 'z' || c == 'Z') { scanAllJoints(); }
  else if (c == 'n' || c == 'N') {
    applyAllLegs(NEUTRAL);
    Serial.println(F("Neutral stance applied."));
  }
  else if (c == 'p' || c == 'P') { printOffsets(); }
  else if (c == 'h' || c == 'H') { printHelp(); }
  // anything else (newlines etc.) is ignored
}

// Reports every I2C device that acknowledges, so a missing or
// mis-addressed PCA9685 shows up at boot instead of the sketch silently
// writing pulses into nothing. Expected: 0x40 (the PCA9685) and 0x70
// (its all-call address). A board strapped to another address (solder
// jumpers A0-A5) shows up here too — that means 15 of the 18 joints are
// being sent to an address nothing is listening on.
void scanI2C() {
  Serial.print(F("I2C scan:"));
  bool pcaAt40 = false;
  int found = 0;
  for (uint8_t a = 8; a < 120; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F(" 0x"));
      Serial.print(a, HEX);
      if (a == 0x40) pcaAt40 = true;
      found++;
    }
  }
  if (found == 0) Serial.print(F(" nothing found"));
  Serial.println();
  if (!pcaAt40) {
    Serial.println(F("WARNING: no PCA9685 at 0x40 — every PCA channel is dead."));
    Serial.println(F("Check VCC/GND/SDA=A4/SCL=A5 and the address jumpers."));
  }
}

void setup() {
  Serial.begin(9600);
  backend.begin();
  scanI2C();

  int attached = 0, skipped = 0;
  for (uint8_t li = 0; li < NUM_LEGS; li++) {
    currentPose[li] = NEUTRAL;
    if (!legs[li].enabled) continue;
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
      int us = jointToUs(legs[li].joint[j], j == COXA ? NEUTRAL.coxa
                       : j == FEMUR ? NEUTRAL.femur : NEUTRAL.tibia);
      if (backend.attach(li, j, legs[li].joint[j], us)) attached++;
      else skipped++;
    }
  }

  // Build tag: bump this whenever the sketch changes, so a stale upload
  // (e.g. the IDE compiling an old buffer) is obvious in the Serial
  // Monitor without counting joints.
  Serial.println(F("Hexapod fw build 3 (I2C scan + z joint scan + stop fix)"));
  Serial.print(F("Hexapod ready (PCA9685 + Uno pins). Joints attached: "));
  Serial.println(attached);
  if (skipped > 0) {
    Serial.print(F("WARNING: "));
    Serial.print(skipped);
    Serial.println(F(" joints skipped — more Uno-pin joints in legs[]"));
    Serial.println(F("than the Servo.h fallback pool holds (3). Move"));
    Serial.println(F("them onto PCA9685 channels or raise MAX_UNO_SERVOS."));
  }
  printHelp();
}

bool aSwings = true;  // which tripod swings on the next half-cycle
bool settled = true;  // legs boot into the neutral stance

void loop() {
  while (Serial.available() > 0)
    handleCommand(Serial.read());

  if (motion == STOP) {
    // Settle back to neutral exactly once after a stop. (Checking for
    // STOP after runHalfCycle() never fired: commands are only read at
    // the top of loop(), so the robot froze in its last gait posture.)
    if (!settled) {
      returnToNeutral();
      settled = true;
    }
    return;
  }
  settled = false;

  // Latch the motion for this half-cycle so a direction change can't
  // yank the stride factors mid-air; new commands take effect at the
  // next half-cycle boundary, when all six feet are on the ground.
  Motion m = motion;
  runHalfCycle(aSwings, m);
  aSwings = !aSwings;
}
