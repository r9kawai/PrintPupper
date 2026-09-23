#include <Servo.h>

/*
  PrintPupper: Front-Left (FL) Leg Walking Motion Test

    Wiring:
      Servo FL1 signal -> GPIO/D3
      Servo FL2 signal -> GPIO/D5
      Servo FL3 signal -> GPIO/D6

    The horns were installed while all servos were set to 90 degrees.
    Therefore, 90° is the mechanical neutral position, matching the Raspberry Pi program.

    This is a direct-command debug sketch for testing the mechanism.
    In loop(), each moveToPose parameter represents the physical angles
    sent to the FL1, FL2, and FL3 servos (from left to right).
*/

const byte SERVO_1_PIN = 3;
const byte SERVO_2_PIN = 5;
const byte SERVO_3_PIN = 6;

Servo servo1;
Servo servo2;
Servo servo3;

int servo1Angle = 90;
int servo2Angle = 90;
int servo3Angle = 90;

// Larger values make the motion slower.  5 ms is one degree per 5 ms.
const unsigned long STEP_DELAY_MS = 5;

// Move one servo to targetAngle one degree at a time.
void moveServoSlowly(Servo &servo, int &currentAngle, int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);

  if (targetAngle > currentAngle) {
    for (int angle = currentAngle + 1; angle <= targetAngle; ++angle) {
      servo.write(angle);
      delay(STEP_DELAY_MS);
    }
  } else if (targetAngle < currentAngle) {
    for (int angle = currentAngle - 1; angle >= targetAngle; --angle) {
      servo.write(angle);
      delay(STEP_DELAY_MS);
    }
  } else {
    // Also emit the initial 90-degree command after attach().
    servo.write(targetAngle);
  }

  currentAngle = targetAngle;
}

// The arguments are direct physical angles for servos 1, 2, and 3.
void moveToPose(int target1, int target2, int target3) {
  moveServoSlowly(servo1, servo1Angle, target1);
  moveServoSlowly(servo2, servo2Angle, target2);
  moveServoSlowly(servo3, servo3Angle, target3);
}

void setup() {
  // 1000--2000 us matches the 90-degree = 1500-us neutral used on the Pi.
  servo1.attach(SERVO_1_PIN, 1000, 2000);
  servo2.attach(SERVO_2_PIN, 1000, 2000);
  servo3.attach(SERVO_3_PIN, 1000, 2000);

  // Always begin at the assembled, safe neutral position.
  moveToPose(90, 90, 90);
  delay(2000);
}

void loop() {
  /*
    One-leg gait, repeated.  The three numbers in each step are direct servo
    angles: servo1, servo2, servo3.  Change one number at a time for debugging.
  */
  // 1. Foot at the rear, on the ground.
  moveToPose(90, 70, 90);
  delay(300);

  // 2. Bend the knee and lift the foot.
  moveToPose(90, 70, 116);
  delay(300);

  // 3. Move the lifted foot to the front.
  moveToPose(90, 110, 116);
  delay(300);

  // 4. Put the foot on the ground.
  moveToPose(90, 110, 90);
  delay(300);
}
