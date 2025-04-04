#include <Servo.h>
#include "DRV8825.h"
#include <math.h>

#define CLAW_PIN 5
#define WRIST_PIN 6
#define ELBOW_PIN 9

#define SHOULDER_DIRECTION_PIN 2
#define SHOULDER_STEP_PIN 3

#define CLAW_CODE 'C'
#define WRIST_CODE 'W'
#define ELBOW_CODE 'E'
#define SHOULDER_CODE 'S'

const byte numChars = 10;
char text[numChars];
bool newData = false;

Servo clawServo, wristServo, elbowServo;
DRV8825 shoulderStepper;
const byte stepsPerAngle = 45;
byte shoulderCurrentAngle = 0;

const float upperArmLength = 28.7; // Length in cm
const float forearmLength = 29.0; // Length in cm
const float clawLength = 20.2; // Length in cm

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  clawServo.attach(CLAW_PIN);
  wristServo.attach(WRIST_PIN);
  elbowServo.attach(ELBOW_PIN);
  shoulderStepper.begin(SHOULDER_DIRECTION_PIN, SHOULDER_STEP_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  readLine();
  executeInstruction();
  //setNumber();
}

void moveServo(Servo &servo, byte angle) {

  short currentAngle = servo.read(); //short to allow negative values for the second for loop
  const byte delayTime = 50;

  if (currentAngle < angle) {

    for (; currentAngle <= angle; currentAngle++) {
      servo.write(currentAngle);
      delay(delayTime);
    }
  }
  else {
    
    for (; currentAngle >= angle; currentAngle--) {
      servo.write(currentAngle);
      delay(delayTime);
    }
  }
}

void moveStepper(byte angle) {

  byte delta;
  byte sign;

  if (shoulderCurrentAngle < angle) {
      shoulderStepper.setDirection(DRV8825_CLOCK_WISE);
      delta = angle - shoulderCurrentAngle;
      sign = 2; //Positive
  }
  else {
    shoulderStepper.setDirection(DRV8825_COUNTERCLOCK_WISE);
    delta = shoulderCurrentAngle - angle;
    sign = 0; //Negative
  }

  for (; delta != 0; delta--) {
      for (byte i = 0; i < stepsPerAngle; i++) {
        shoulderStepper.step();
        delay(1);
      }

      shoulderCurrentAngle += sign - 1;
    }
}

// Function to convert (x, y, z) coordinates to the necessary angles. (Inverse kinematics)
// x - direction towards the length of the chessboard, towards the other player
// y - direction towards the width of the chessboard, to the left and right of both players
// z - direction perpendicular to the chessboard, the altitude
void coordsToAngles(float x, float y, float z, byte &baseAngle, byte &shoulderAngle, byte &elbowAngle, byte &wristAngle) {

  if (x == 0) {
    if (y > 0) {
      baseAngle = 180;
    }
    else if (y == 0) {
      baseAngle = 90;
    }
    else {
      baseAngle = 0;
    }
  }
  else {
    
    float baseAngleRadians = atan(y/x);
    baseAngle = (byte) (baseAngleRadians * 180 / M_PI + 90);
  }

  float d = sqrt(x*x + y*y);
  float L = sqrt(d*d + (z+clawLength)*(z+clawLength));

  float theta1Prime = atan((z+clawLength) / d);

  float s = (upperArmLength + forearmLength + L) / 2;
  float area = sqrt(s * (s-upperArmLength) * (s-forearmLength) * (s-L));
  float h = 2 * area / L;

  float theta1Second = asin(h/upperArmLength);

  shoulderAngle = (byte) ((theta1Prime + theta1Second) * 180 / M_PI);

  float theta2 = acos((upperArmLength*upperArmLength + forearmLength*forearmLength - L*L) / (2*upperArmLength*forearmLength));
  elbowAngle = (byte) (theta2 * 180 / M_PI);

  wristAngle = 270 - shoulderAngle - elbowAngle;
  
}

void readLine() {

  static byte i=0;
  char buff;

  if (!newData) {
    for (; i < numChars && Serial.available() > 0; i++) {

      buff = Serial.read();
      text[i] = buff;

      if (buff == '\n') {
        text[i] = '\0';
        newData = true;
        i=0;
        break;
      }
    }
  }
}

void executeInstruction() {

  if (newData) {

    int angle = atoi(text+1);

    switch(text[0]) {

      case CLAW_CODE:
        Serial.println("Moving claw");
        moveServo(clawServo, angle);
        break;
      case WRIST_CODE:
      Serial.println("Moving wrist");
        moveServo(wristServo, angle);
        break;
      case ELBOW_CODE:
      Serial.println("Moving elbow");
        moveServo(elbowServo, angle);
        break;
      case SHOULDER_CODE:
      Serial.println("Moving shoulder");
        moveStepper(angle);
        break;
    }

    newData = false;
  }
}

void showLine() {
  if (newData) {
    //angle = atoi(text);
    Serial.print("Text received: ");
    Serial.println(text);
    newData = false;
    //servo.write(angle);
  }
}
