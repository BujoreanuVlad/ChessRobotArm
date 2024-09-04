#include <Servo.h>
#include "DRV8825.h"

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
const byte stepsPerAngle = 100;
byte shoulderCurrentAngle = 0;

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

void moveServo(Servo &servo, int angle) {

  servo.write(angle);
}

void moveStepper(byte angle) {

  byte delta;

  if (shoulderCurrentAngle < angle) {
      shoulderStepper.setDirection(DRV8825_CLOCK_WISE);
      delta = angle - shoulderCurrentAngle;
  }
  else {
    shoulderStepper.setDirection(DRV8825_COUNTERCLOCK_WISE);
    delta = shoulderCurrentAngle - angle;
  }

  for (; delta != 0; delta--) {
      for (byte i = 0; i < stepsPerAngle; i++) {
        shoulderStepper.step();
        delay(1);
      }
    }
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
