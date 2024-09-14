#include "DRV8825.h"
#include <Servo.h>

const byte numChars = 10;
char text[numChars];
bool newData = false;

DRV8825 shoulder;

Servo rotator;
Servo elbow;
Servo wrist;
Servo grip;
const int stepsPerAngle = 100;
byte shoulderCurrentAngle = 0; 

void moveServo(Servo &servo, byte angle) {

  byte currentPosition = servo.read();
  char sign = angle > currentPosition ? -1 : 1;
  for (; currentPosition != angle; currentPosition += sign) {
     servo.write(currentPosition);
  }
}

void moveStepper(byte angle) {

    byte delta;
    
    if (shoulderCurrentAngle < angle) {
        shoulder.setDirection(DRV8825_CLOCK_WISE);
        delta = angle - shoulderCurrentAngle;
    }
    else {
      shoulder.setDirection(DRV8825_COUNTERCLOCK_WISE);
      delta = shoulderCurrentAngle - angle;
    }

    for (; delta != 0; delta--) {
      for (byte i = 0; i < stepsPerAngle; i++) {
        shoulder.step();
        delay(1);
      }
    }
}

void readNumber() {

  static byte i=0;
  char buff;

  if (!newData) {
    for (; i < numChars && Serial.available() > 0; i++) {

      buff = Serial.read();
      text[i] = buff;
      Serial.print(buff);

      if (buff == '\n') {
        text[i] = '\0';
        newData = true;
        i=0;
        break;
      }
    }
  }
}

void showNumber() {
  if (newData) {
    
    newData = false;
    byte angle = (byte) atoi(text+2);
    
    switch(text[0]) {
      case 'r': moveServo(rotator, angle); break;
      case 's': moveStepper(angle); break;
      case 'e': moveServo(elbow, angle); break;
      case 'w': moveServo(wrist, angle); break;
      case 'g': moveServo(grip, angle); break;
    }
  }
}


void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("DRV8825_LIB_VERSION: ");
  Serial.println(DRV8825_LIB_VERSION);

  shoulder.begin(2, 3);  //  set direction pin + step pin.
  rotator.attach(5);
  elbow.attach(6);
  wrist.attach(9);
  grip.attach(10);
}


void loop()
{
  readNumber();
  showNumber();
}
