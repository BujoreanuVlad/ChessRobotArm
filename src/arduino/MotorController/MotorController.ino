#include <Servo.h>

#define CLAW_PIN 5
#define WRIST_PIN 6
#define ELBOW_PIN 9

#define CLAW_CODE 'C'
#define WRIST_CODE 'W'
#define ELBOW_CODE 'E'
#define SHOULDER_CODE 'S'

const byte numChars = 10;
char text[numChars];
bool newData = false;

Servo clawServo, wristServo, elbowServo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  clawServo.attach(CLAW_PIN);
  wristServo.attach(WRIST_PIN);
  elbowServo.attach(ELBOW_PIN);
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

void moveStepper() {

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
        moveServo(clawServo, angle);
        break;
      case WRIST_CODE:
        moveServo(wristServo, angle);
        break;
      case ELBOW_CODE:
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
