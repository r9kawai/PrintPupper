#include <Servo.h>
#include <math.h>
#include <string.h>

/*
  PrintPupper kinematics / gait test
  serial command -> gait phase -> foot positions -> normal-leg IK
                 -> non-parallel knee linkage -> servo angles -> Servo.write()

  Select "Newline" in Serial Monitor and send: w, s, a, d, stop, stand, print
  FL alone is enabled initially. Add pins and enable legs one by one.
*/

enum LegId : uint8_t { FL = 0, FR = 1, BL = 2, BR = 3, LEG_COUNT = 4 };
enum AxisId : uint8_t { ABDUCTION = 0, UPPER_LEG = 1, KNEE = 2, AXIS_COUNT = 3 };
enum LegPhase : uint8_t { STANCE = 0, SWING = 1 };

struct Vec3 { float x, y, z; };
// Model radians. knee is the virtual interior angle, not the horn angle.
struct JointAngles { float abduction, upperLeg, knee; };
// Direct physical Servo.write() degrees.
struct ServoAngles { float hip, upperLeg, knee; };
struct GaitState { LegPhase phase[LEG_COUNT]; float phase01[LEG_COUNT]; };
struct FootPositions { Vec3 foot[LEG_COUNT]; };
struct MotionCommand { float vx, vy; };

// ------------------- wiring: set pins and enable one leg at a time ---------
const int8_t kServoPins[LEG_COUNT][AXIS_COUNT] = {
  /* FL */ { 3, 5, 6 },
  /* FR */ { -1, -1, -1 },
  /* BL */ { -1, -1, -1 },
  /* BR */ { -1, -1, -1 }
};
bool kLegEnabled[LEG_COUNT] = { true, false, false, false };
Servo gServos[LEG_COUNT][AXIS_COUNT];

// ---------------------------- geometry in metres --------------------------
const float kLegFrontBack = 0.100f;
const float kLegLeftRight = 0.045f;
const float kAbductionOffset = 0.045f;
const float kUpperLegLength = 0.110f;
const float kLowerLegLength = 0.120f;

// Non-parallel knee linkage: same dimensions as PrintPupper/src/Config.py.
const float kKneeLinkOnLowerLeg = 0.060f;
const float kKneeHornLength = 0.050f;
const float kKneeRodLength = 0.090f;
const float kKneeServoOffsetX = -0.005f;
const float kKneeServoOffsetY =  0.025f;

// This sketch uses FL, FR, BL, BR (unlike the Pi source's FR, FL, BR, BL).
const Vec3 kLegOrigins[LEG_COUNT] = {
  {  kLegFrontBack,  kLegLeftRight, 0.0f }, // FL
  {  kLegFrontBack, -kLegLeftRight, 0.0f }, // FR
  { -kLegFrontBack,  kLegLeftRight, 0.0f }, // BL
  { -kLegFrontBack, -kLegLeftRight, 0.0f }  // BR
};
const Vec3 kNominalFeet[LEG_COUNT] = {
  {  0.117f,  0.110f, -0.165f }, // FL
  {  0.117f, -0.110f, -0.165f }, // FR
  { -0.103f,  0.110f, -0.165f }, // BL
  { -0.103f, -0.110f, -0.165f }  // BR
};

// Pi calibration, reordered into FL, FR, BL, BR.
const float kServoMultiplier[LEG_COUNT][AXIS_COUNT] = {
  {  1.0f,  1.0f, -1.0f }, // FL
  {  1.0f, -1.0f,  1.0f }, // FR
  { -1.0f,  1.0f, -1.0f }, // BL
  { -1.0f, -1.0f,  1.0f }  // BR
};
const float kServoNeutralRad[AXIS_COUNT] = {
  0.0f, 45.0f * DEG_TO_RAD, -45.0f * DEG_TO_RAD
};
const float kServoNeutralDeg = 90.0f;

// ------------------------------ gait constants ----------------------------
const unsigned long kControlPeriodMs = 20;
const float kOverlapTime = 0.140f;
const float kSwingTime = 0.200f;
const float kStanceTime = 2.0f * kOverlapTime + kSwingTime; // 0.48 s
const float kGaitCycleTime = kStanceTime + kSwingTime;      // 0.68 s
const float kSwingClearance = 0.055f;
const float kForwardSpeed = 0.28f;
const float kBackwardSpeed = 0.11f;
const float kSideSpeed = 0.10f;
const float kStartRampTime = 0.40f;
// Same phase layout as Python: FL+BR swing, then FR+BL swing.
const float kSwingStart[LEG_COUNT] = { 0.140f, 0.480f, 0.480f, 0.140f };

MotionCommand gCommand = { 0.0f, 0.0f };
bool gWalking = false;
unsigned long gGaitStartMs = 0;
unsigned long gLastControlMs = 0;

float clampf(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}
float wrapPositive(float value, float period) {
  value = fmodf(value, period);
  return value < 0.0f ? value + period : value;
}
float acosClamped(float value) { return acosf(clampf(value, -1.0f, 1.0f)); }
float dot2(float ax, float ay, float bx, float by) { return ax * bx + ay * by; }
bool isRightLeg(LegId leg) { return leg == FR || leg == BR; }

// 1. Time -> individual stance/swing phase (0.0--1.0) for every foot.
GaitState gaitStateForTime(unsigned long elapsedMs) {
  GaitState result;
  const float t = elapsedMs * 0.001f;
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
    const float sinceSwingStart = wrapPositive(t - kSwingStart[leg], kGaitCycleTime);
    if (sinceSwingStart < kSwingTime) {
      result.phase[leg] = SWING;
      result.phase01[leg] = sinceSwingStart / kSwingTime;
    } else {
      result.phase[leg] = STANCE;
      result.phase01[leg] = (sinceSwingStart - kSwingTime) / kStanceTime;
    }
  }
  return result;
}

// 2. Phase + command -> all body-frame foot positions.
FootPositions footPositionsForGait(const GaitState &gait, const MotionCommand &command,
                                   unsigned long elapsedMs) {
  FootPositions result;
  const float ramp = clampf(elapsedMs * 0.001f / kStartRampTime, 0.0f, 1.0f);
  // A stance foot moves forward -> rear relative to the body. A swing foot
  // returns rear -> forward. This is the same open-loop convention as Python.
  const float stepX = command.vx * kStanceTime * ramp;
  const float stepY = command.vy * kStanceTime * ramp;
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
    const float p = gait.phase01[leg];
    float horizontalFraction;
    float zLift = 0.0f;
    if (gait.phase[leg] == SWING) {
      horizontalFraction = -0.5f + p;
      zLift = (p < 0.5f) ? (p / 0.5f) * kSwingClearance
                          : ((1.0f - p) / 0.5f) * kSwingClearance;
    } else {
      horizontalFraction = 0.5f - p;
    }
    result.foot[leg] = kNominalFeet[leg];
    result.foot[leg].x += horizontalFraction * stepX;
    result.foot[leg].y += horizontalFraction * stepY;
    result.foot[leg].z += zLift;
  }
  return result;
}

FootPositions nominalFootPositions() {
  FootPositions result;
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) result.foot[leg] = kNominalFeet[leg];
  return result;
}

// 3. Body-frame foot position -> ideal serial-leg IK.
// out.knee is the angle between upper and lower links, before linkage conversion.
bool inverseKinematics(LegId leg, const Vec3 &bodyFoot, JointAngles &out) {
  const Vec3 origin = kLegOrigins[leg];
  const float x = bodyFoot.x - origin.x;
  const float y = bodyFoot.y - origin.y;
  const float z = bodyFoot.z - origin.z;
  const float rBodyFootYZ = sqrtf(y * y + z * z);
  if (rBodyFootYZ <= kAbductionOffset) return false;
  const float rHipFootYZ2 = rBodyFootYZ * rBodyFootYZ
                           - kAbductionOffset * kAbductionOffset;
  if (rHipFootYZ2 <= 0.0f) return false;
  const float rHipFootYZ = sqrtf(rHipFootYZ2);

  const float signedOffset = isRightLeg(leg) ? -kAbductionOffset : kAbductionOffset;
  out.abduction = acosClamped(signedOffset / rBodyFootYZ) + atan2f(z, y);

  const float hipToFoot = sqrtf(rHipFootYZ * rHipFootYZ + x * x);
  const float minReach = fabsf(kUpperLegLength - kLowerLegLength);
  const float maxReach = kUpperLegLength + kLowerLegLength;
  if (hipToFoot <= minReach || hipToFoot >= maxReach) return false;

  const float theta = atan2f(-x, rHipFootYZ);
  const float upperTriangleAngle = acosClamped(
    (kUpperLegLength * kUpperLegLength + hipToFoot * hipToFoot
     - kLowerLegLength * kLowerLegLength) / (2.0f * kUpperLegLength * hipToFoot));
  out.upperLeg = theta + upperTriangleAngle;
  out.knee = acosClamped(
    (kUpperLegLength * kUpperLegLength + kLowerLegLength * kLowerLegLength
     - hipToFoot * hipToFoot) / (2.0f * kUpperLegLength * kLowerLegLength));
  return true;
}

// 4a. Ideal knee angle -> physical non-parallel knee horn angle.
bool nonParallelKneeHornAngle(float thetaLeg, float thetaKnee, bool mirror,
                               float &hornAngleOut) {
  if (mirror) { thetaLeg = -thetaLeg; thetaKnee = -thetaKnee; }
  const float x1 = kUpperLegLength * cosf(thetaLeg);
  const float y1 = kUpperLegLength * sinf(thetaLeg);
  const float lowerLegDirection = thetaLeg + (PI - thetaKnee);
  const float px = x1 + kKneeLinkOnLowerLeg * cosf(lowerLegDirection);
  const float py = y1 + kKneeLinkOnLowerLeg * sinf(lowerLegDirection);
  const float dx = px - kKneeServoOffsetX;
  const float dy = py - kKneeServoOffsetY;
  const float distance = sqrtf(dx * dx + dy * dy);
  if (distance < 0.000001f) return false;

  const float cosAtServo = (kKneeHornLength * kKneeHornLength + distance * distance
                           - kKneeRodLength * kKneeRodLength)
                          / (2.0f * kKneeHornLength * distance);
  if (cosAtServo < -1.0f || cosAtServo > 1.0f) return false;

  const float lineAngle = atan2f(dy, dx);
  const float angleAtServo = acosf(cosAtServo);
  const float candidate1 = lineAngle + angleAtServo;
  const float candidate2 = lineAngle - angleAtServo;
  // The circles have two intersections. Pick the assembled branch whose horn
  // direction is closer to the lower-leg direction.
  const float targetX = cosf(lowerLegDirection);
  const float targetY = sinf(lowerLegDirection);
  const float hornAngle = dot2(cosf(candidate1), sinf(candidate1), targetX, targetY)
                        > dot2(cosf(candidate2), sinf(candidate2), targetX, targetY)
                        ? candidate1 : candidate2;
  hornAngleOut = mirror ? -hornAngle : hornAngle;
  return true;
}

// 4b. Ideal 3-DOF angles -> direct three-servo angles in degrees.
// The lateral (hip) axis bypasses the linkage but still needs its calibration.
bool calculateServoAngles(LegId leg, const JointAngles &virtualAngles,
                          ServoAngles &out) {
  float modelAngle[AXIS_COUNT] = {
    virtualAngles.abduction, virtualAngles.upperLeg, virtualAngles.knee
  };
  const float upperAfterMirror = virtualAngles.upperLeg * kServoMultiplier[leg][UPPER_LEG];
  const float kneeAfterMirror = virtualAngles.knee * kServoMultiplier[leg][KNEE];
  const bool mirror = isRightLeg(leg);
  const float linkageUpper = mirror ? upperAfterMirror - PI * 0.5f
                                    : upperAfterMirror + PI * 0.5f;
  float kneeHornAngle;
  if (!nonParallelKneeHornAngle(linkageUpper, kneeAfterMirror, mirror, kneeHornAngle)) {
    return false;
  }
  // Return to the common model convention, then calibrate all three axes alike.
  modelAngle[KNEE] = kneeHornAngle * kServoMultiplier[leg][KNEE];

  float servoDeg[AXIS_COUNT];
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    servoDeg[axis] = kServoNeutralDeg
      + (modelAngle[axis] - kServoNeutralRad[axis])
        * kServoMultiplier[leg][axis] * RAD_TO_DEG;
    servoDeg[axis] = clampf(servoDeg[axis], 0.0f, 180.0f);
  }
  out.hip = servoDeg[ABDUCTION];
  out.upperLeg = servoDeg[UPPER_LEG];
  out.knee = servoDeg[KNEE];
  return true;
}

// 5. Apply direct physical servo angles. Kept independent for servo-only tests.
void applyServoAngles(LegId leg, const ServoAngles &angles) {
  if (!kLegEnabled[leg]) return;
  const float requested[AXIS_COUNT] = { angles.hip, angles.upperLeg, angles.knee };
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    if (kServoPins[leg][axis] >= 0) {
      gServos[leg][axis].write((int)lroundf(clampf(requested[axis], 0.0f, 180.0f)));
    }
  }
}

// One foot through the complete pipeline. false keeps its last command safely.
bool applyFootPosition(LegId leg, const Vec3 &bodyFoot) {
  JointAngles virtualAngles;
  ServoAngles servoAngles;
  if (!inverseKinematics(leg, bodyFoot, virtualAngles)) return false;
  if (!calculateServoAngles(leg, virtualAngles, servoAngles)) return false;
  applyServoAngles(leg, servoAngles);
  return true;
}

void applyFeet(const FootPositions &feet) {
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
    if (kLegEnabled[leg] && !applyFootPosition((LegId)leg, feet.foot[leg])) {
      Serial.print(F("IK/linkage unreachable on leg "));
      Serial.println(leg);
    }
  }
}

void commandStand() { applyFeet(nominalFootPositions()); }

void startWalking(float vx, float vy) {
  gCommand.vx = vx;
  gCommand.vy = vy;
  gWalking = true;
  gGaitStartMs = millis();
  Serial.print(F("walking: vx=")); Serial.print(vx, 3);
  Serial.print(F(" vy=")); Serial.println(vy, 3);
}

void stopWalking() {
  gWalking = false;
  gCommand.vx = 0.0f;
  gCommand.vy = 0.0f;
  commandStand();
  Serial.println(F("stopped: nominal stance commanded"));
}

void printFlSnapshot() {
  const unsigned long elapsed = gWalking ? millis() - gGaitStartMs : 0;
  const GaitState gait = gaitStateForTime(elapsed);
  const FootPositions feet = gWalking ? footPositionsForGait(gait, gCommand, elapsed)
                                      : nominalFootPositions();
  JointAngles ik;
  ServoAngles servo;
  Serial.print(F("FL foot [m] = "));
  Serial.print(feet.foot[FL].x, 4); Serial.print(F(", "));
  Serial.print(feet.foot[FL].y, 4); Serial.print(F(", "));
  Serial.println(feet.foot[FL].z, 4);
  if (!inverseKinematics(FL, feet.foot[FL], ik)) {
    Serial.println(F("FL IK: unreachable")); return;
  }
  Serial.print(F("FL IK [deg] = abduct ")); Serial.print(ik.abduction * RAD_TO_DEG, 1);
  Serial.print(F(", upper ")); Serial.print(ik.upperLeg * RAD_TO_DEG, 1);
  Serial.print(F(", knee interior ")); Serial.println(ik.knee * RAD_TO_DEG, 1);
  if (!calculateServoAngles(FL, ik, servo)) {
    Serial.println(F("FL knee linkage: unreachable")); return;
  }
  Serial.print(F("FL servo [deg] = hip ")); Serial.print(servo.hip, 1);
  Serial.print(F(", upper ")); Serial.print(servo.upperLeg, 1);
  Serial.print(F(", knee ")); Serial.println(servo.knee, 1);
}

void processSerialCommand(const char *line) {
  if      (strcmp(line, "w") == 0) startWalking( kForwardSpeed, 0.0f);
  else if (strcmp(line, "s") == 0) startWalking(-kBackwardSpeed, 0.0f);
  else if (strcmp(line, "a") == 0) startWalking(0.0f,  kSideSpeed);
  else if (strcmp(line, "d") == 0) startWalking(0.0f, -kSideSpeed);
  else if (strcmp(line, "stop") == 0) stopWalking();
  else if (strcmp(line, "stand") == 0) commandStand();
  else if (strcmp(line, "print") == 0) printFlSnapshot();
  else if (line[0] != '\0') Serial.println(F("commands: w s a d stop stand print"));
}

void readSerialCommands() {
  static char line[16];
  static uint8_t length = 0;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[length] = '\0';
      processSerialCommand(line);
      length = 0;
    } else if (length < sizeof(line) - 1) {
      line[length++] = c;
    } else {
      length = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t leg = 0; leg < LEG_COUNT; ++leg) {
    if (!kLegEnabled[leg]) continue;
    for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
      if (kServoPins[leg][axis] >= 0) {
        // Preserve the existing test's 90 degrees = 1500 us convention.
        gServos[leg][axis].attach(kServoPins[leg][axis], 1000, 2000);
      }
    }
  }
  delay(100);
  commandStand();
  Serial.println(F("PrintPupper gait test ready. Send w/s/a/d, stop, stand, or print."));
}

void loop() {
  readSerialCommands();
  const unsigned long now = millis();
  if (now - gLastControlMs < kControlPeriodMs) return;
  gLastControlMs = now;
  if (gWalking) {
    const unsigned long elapsed = now - gGaitStartMs;
    const GaitState gait = gaitStateForTime(elapsed);
    applyFeet(footPositionsForGait(gait, gCommand, elapsed));
  }
}
