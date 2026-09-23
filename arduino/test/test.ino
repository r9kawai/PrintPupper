#include <Servo.h>

/*
  PrintPupper: one-leg walking-motion test

  Wiring
    Servo 1 signal -> GPIO/D3
    Servo 2 signal -> GPIO/D5
    Servo 3 signal -> GPIO/D6

  The horns were fitted while every servo was at 90 degrees.  Therefore 90 is
  the mechanical neutral point, just as it is in the Raspberry Pi program.

  This sketch uses the same direction convention as PrintPupper's front-right
  leg in src/Config.py: servo 1 = +, servo 2 = -, servo 3 = +.
  If this is another leg, change only the three *_DIRECTION constants below.
*/

const byte SERVO_1_PIN = 3;
const byte SERVO_2_PIN = 5;
const byte SERVO_3_PIN = 6;

const int NEUTRAL_ANGLE = 90;

// PrintPupper front-right leg direction convention.
const int SERVO_1_DIRECTION = +1;  // coxa / left-right
const int SERVO_2_DIRECTION = -1;  // upper leg
const int SERVO_3_DIRECTION = +1;  // lower leg / knee

// Keep the test motion within a conservative range around mechanical neutral.
const int MAX_OFFSET = 35;

// Tune these two values first if a different stride or foot clearance is needed.
const int STEP_ANGLE = 20;  // fore/aft swing: +/- degrees from neutral
const int LIFT_ANGLE = 26;  // knee bend while the foot is off the ground

Servo servo1;
Servo servo2;
Servo servo3;

struct LegPose {
  int coxaOffset;
  int hipOffset;
  int kneeOffset;
};

LegPose currentPose = {0, 0, 0};

int clampOffset(int offset) {
  return constrain(offset, -MAX_OFFSET, MAX_OFFSET);
}

// Converts a logical joint offset into the physical Servo.h angle.
int servoAngle(int offset, int direction) {
  return constrain(NEUTRAL_ANGLE + clampOffset(offset) * direction, 0, 180);
}

void writePose(const LegPose &pose) {
  servo1.write(servoAngle(pose.coxaOffset, SERVO_1_DIRECTION));
  servo2.write(servoAngle(pose.hipOffset, SERVO_2_DIRECTION));
  servo3.write(servoAngle(pose.kneeOffset, SERVO_3_DIRECTION));
}

// Move gradually so the linkage never receives a step command.
void moveTo(const LegPose &target, unsigned long durationMs) {
  const unsigned long stepMs = 20;
  const int steps = max(1, (int)(durationMs / stepMs));
  const LegPose start = currentPose;

  for (int step = 1; step <= steps; ++step) {
    LegPose intermediate = {
      start.coxaOffset + (target.coxaOffset - start.coxaOffset) * step / steps,
      start.hipOffset  + (target.hipOffset  - start.hipOffset)  * step / steps,
      start.kneeOffset + (target.kneeOffset - start.kneeOffset) * step / steps
    };
    writePose(intermediate);
    delay(stepMs);
  }

  currentPose = target;
}

void setup() {
  // 1000--2000 us matches the 90-degree = 1500-us neutral used on the Pi.
  servo1.attach(SERVO_1_PIN, 1000, 2000);
  servo2.attach(SERVO_2_PIN, 1000, 2000);
  servo3.attach(SERVO_3_PIN, 1000, 2000);

  // Always begin at the assembled, safe neutral position.
  writePose(currentPose);
  delay(2000);
}

void loop() {
  /*
    One-leg gait, repeated:
      1. With the foot assumed on the ground, move it to the rear.
      2. Bend the knee to lift the foot.
      3. Swing the lifted foot to the front.
      4. Straighten the knee to place it down.

    A single free leg cannot translate the robot by itself, but this verifies
    the same lift / swing / place sequence that is used for a walking leg.
  */
  // The signs are chosen for forward motion with the actual one-leg model.
  // Swap the +/- STEP_ANGLE values below only if the physical leg is mounted
  // as the mirrored (left) side.
  const LegPose rearOnGround  = {0,  STEP_ANGLE, 0};
  const LegPose rearLifted    = {0,  STEP_ANGLE, LIFT_ANGLE};
  const LegPose frontLifted   = {0, -STEP_ANGLE, LIFT_ANGLE};
  const LegPose frontOnGround = {0, -STEP_ANGLE, 0};

  moveTo(rearOnGround, 500);
  delay(120);
  moveTo(rearLifted, 350);
  moveTo(frontLifted, 550);
  moveTo(frontOnGround, 350);
  delay(120);
}
