#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>  // Adafruit PWM Servo Driver Library
#include <Servo.h>                    // LR leg on Uno pins D9-D11

// ============================================================
// Calibration reference: drives all 18 servos to the
// calibration pose on boot and holds it forever.
//
// The pose every leg should physically show:
//   coxa  : leg pointing straight out from its body corner
//           (check from directly above)
//   femur : hanging plumb straight down
//   tibia : dead straight in line with the femur (knee locked)
//
// Any joint not matching that picture has a wrong offset (or a
// horn mounted a spline tooth off). Tune offsets live with the
// Hexapod sketch's calibration UI ('v' pose, +/- keys), or edit
// the offsetDeg values below and re-upload until the pose is
// right — the table layout is identical to Hexapod.ino's
// legs[], so calibrated values paste straight across.
//
// !!! Prop the body >= 25 cm off the bench BEFORE powering up:
// the legs extend fully downward (feet ~245 mm below the hips).
// ============================================================

// ---- Servo mapping (identical to Hexapod.ino) ----
const int      SERVO_MIN_US     = 500;
const int      SERVO_MAX_US     = 2500;
const int      SERVO_MAX_DEG    = 270;
const int      SERVO_CENTER_DEG = 135;  // logical 0 deg maps here
const int      SERVO_FREQ_HZ    = 50;
const uint32_t PCA_OSC_HZ       = 27000000;

// ---- Calibration pose (logical degrees, Hexapod.ino convention) ----
const float CAL_COXA  = 0;    // straight out from the body corner
const float CAL_FEMUR = -90;  // plumb straight down
const float CAL_TIBIA = 0;    // in line with the femur

const int NUM_LEGS   = 6;
const int NUM_JOINTS = 3;

// ---- Per-leg configuration (keep in sync with Hexapod.ino) ----
// sign: legs are identical assemblies mounted radially, so only the
// coxa mirrors on the left side. (In this pose coxa is 0, so sign
// doesn't move anything — it's kept so the table matches Hexapod.ino.)
// offsetDeg: paste calibrated values here AND into Hexapod.ino.
struct JointConfig {
  uint8_t channel;    // PCA9685 channel, or Uno pin when onUno is set
  bool    onUno;
  int8_t  sign;
  int16_t offsetDeg;
};

struct LegConfig {
  const char* name;
  JointConfig joint[NUM_JOINTS];  // coxa, femur, tibia
};

LegConfig legs[NUM_LEGS] = {
  // name    coxa                 femur                tibia
  { "RF", {{ 0, false, +1, 0}, { 1, false, +1, 0}, { 2, false, +1, 0}} },
  { "RM", {{ 3, false, +1, 0}, { 4, false, +1, 0}, { 5, false, +1, 0}} },
  { "RR", {{ 6, false, +1, 0}, { 7, false, +1, 0}, { 8, false, +1, 0}} },
  { "LF", {{ 9, false, -1, 0}, {10, false, +1, 0}, {11, false, +1, 0}} },
  { "LM", {{12, false, -1, 0}, {13, false, +1, 0}, {14, false, +1, 0}} },
  { "LR", {{ 9, true,  -1, 0}, {10, true,  +1, 0}, {11, true,  +1, 0}} },
};

Adafruit_PWMServoDriver pwm;  // default I2C address 0x40
Servo unoServo[NUM_JOINTS];   // the LR leg's three joints

int jointToUs(const JointConfig &jc, float logicalDeg) {
  float servoDeg = SERVO_CENTER_DEG + jc.sign * logicalDeg + jc.offsetDeg;
  servoDeg = constrain(servoDeg, 0, SERVO_MAX_DEG);
  return map((int)round(servoDeg), 0, SERVO_MAX_DEG, SERVO_MIN_US, SERVO_MAX_US);
}

const char* jointName(int j) {
  return j == 0 ? "coxa" : (j == 1 ? "femur" : "tibia");
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  Wire.setClock(400000);

  Serial.println(F("Calibration pose — every leg should look identical:"));
  Serial.println(F("  coxa straight out | femur plumb down | tibia straight in line"));
  Serial.println(F("Joint -> pulse sent:"));

  int unoUsed = 0;
  for (int li = 0; li < NUM_LEGS; li++) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      const JointConfig &jc = legs[li].joint[j];
      float logicalDeg = j == 0 ? CAL_COXA : (j == 1 ? CAL_FEMUR : CAL_TIBIA);
      int us = jointToUs(jc, logicalDeg);

      if (!jc.onUno) {
        pwm.writeMicroseconds(jc.channel, us);
      } else {
        // write before attach so the first pulse is the target pose
        unoServo[unoUsed].writeMicroseconds(us);
        unoServo[unoUsed].attach(jc.channel, SERVO_MIN_US, SERVO_MAX_US);
        unoUsed++;
      }

      Serial.print(legs[li].name);
      Serial.print(' ');
      Serial.print(jointName(j));
      Serial.print(jc.onUno ? F(" (Uno D") : F(" (CH"));
      Serial.print(jc.channel);
      Serial.print(F(") -> "));
      Serial.print(us);
      Serial.println(F(" us"));
    }
  }

  Serial.println(F("Holding pose. Joints off-target need an offsetDeg tweak"));
  Serial.println(F("(here and in Hexapod.ino) or a horn re-mount."));
}

void loop() {
  // Nothing to do — the PCA9685 keeps emitting the last pulse widths
  // and Servo.h refreshes the Uno-pin servos automatically.
}
