#include <Servo.h>
#include <DRV8825.h>
#include <math.h>

#define SWITCH_PIN 4

#define CLAW_PIN 5
#define WRIST_PIN 6
#define ELBOW_PIN 9
#define BASE_PIN 10

#define SHOULDER_DIRECTION_PIN 2
#define SHOULDER_STEP_PIN 3

#define CLAW_CODE 'C'
#define WRIST_CODE 'W'
#define ELBOW_CODE 'E'
#define SHOULDER_CODE 'S'
#define BASE_CODE 'B'

#define CALIBRATE_CODE 'c'
#define VERTICAL_CODE 'v'
#define HORIZONTAL_CODE 'h'
#define INIT_CODE 'i'
#define BOARD_CODE 'b'

const byte numChars = 10;
char text[numChars];
bool newData = false;

Servo clawServo, wristServo, elbowServo, baseServo;
DRV8825 shoulderStepper;
const byte stepsPerAngle = 44;
byte shoulderCurrentAngle = 135;

const float upperArmLength = 28.7; // Length in cm
const float forearmLength = 29.0; // Length in cm
const float clawLength = 20.2; // Length in cm

float boardLength;
float boardXOffset;
float boardYOffset;
float boardHeight;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  clawServo.attach(CLAW_PIN);
  wristServo.attach(WRIST_PIN);
  elbowServo.attach(ELBOW_PIN);
  //baseServo.attach(BASE_PIN, 647, 2400);
  baseServo.attach(BASE_PIN);
  shoulderStepper.begin(SHOULDER_DIRECTION_PIN, SHOULDER_STEP_PIN);
  pinMode(SWITCH_PIN, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  readLine();
  executeInstruction();
  //setNumber();
}

void calibrate() {

  moveServo(baseServo, 90);

  shoulderStepper.setDirection(DRV8825_CLOCK_WISE);

  while (digitalRead(SWITCH_PIN) == 0) {
    shoulderStepper.step();
    delay(1);
  }

  shoulderCurrentAngle = 180;
  delay(75);

  moveStepper(135);
}

void moveServo(Servo &servo, byte angle) {

  short currentAngle = servo.read(); //short to allow negative values for the second for loop
  const byte delayTime = 25;

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

void anglesToCoords(byte baseAngle, byte shoulderAngle, byte elbowAngle, byte wristAngle, float &x, float &y, float &z) {

  float baseAngleRadians = (float) baseAngle * M_PI / 180;
  float shoulderAngleRadians = (float) shoulderAngle * M_PI / 180;
  float elbowAngleRadians = (float) elbowAngle * M_PI / 180;

  float d = cos(shoulderAngleRadians) * upperArmLength + sin(shoulderAngleRadians + elbowAngleRadians - M_PI_2) * forearmLength;
  z = sin(shoulderAngleRadians) * upperArmLength - cos(shoulderAngleRadians + elbowAngleRadians - M_PI_2) * forearmLength - clawLength;

  y = sin(baseAngleRadians - M_PI_2) * d;
  x = cos(baseAngleRadians - M_PI_2) * d;
}

bool checkValidAngles(const byte baseAngle, const byte shoulderAngle, const byte elbowAngle, const byte wristAngle) {

  return (baseAngle >=0 && baseAngle <= 180) &&
         (shoulderAngle >= 0 && shoulderAngle <= 180) &&
         (elbowAngle >= 55 && elbowAngle <= 180) &&
         (wristAngle >= 60 && wristAngle <= 180);
}

void moveVertical(float finalZ) {

  float x, y, z;

  byte baseAngle = baseServo.read();
  byte shoulderAngle = shoulderCurrentAngle;
  byte elbowAngle = elbowServo.read();
  byte wristAngle = wristServo.read();

  anglesToCoords(baseAngle, shoulderAngle, elbowAngle, wristAngle, x, y, z);

  const float granularity = 0.1;

  if (finalZ > z) {

    for (; z <= finalZ; z += granularity) {
      
      coordsToAngles(x, y, z, baseAngle, shoulderAngle, elbowAngle, wristAngle);

        
//      Serial.println(baseAngle);
//      Serial.println(shoulderAngle);
//      Serial.println(elbowAngle);
//      Serial.println(wristAngle);
      
      if (checkValidAngles(baseAngle, shoulderAngle, elbowAngle, wristAngle))
          {
            moveServo(baseServo, baseAngle);
            moveStepper(shoulderAngle);
            moveServo(elbowServo, elbowAngle);
            moveServo(wristServo, wristAngle);
      }
      else {
        return;
      }
    }
  }
  else {
    for (; z >= finalZ; z -= granularity) {
      
      coordsToAngles(x, y, z, baseAngle, shoulderAngle, elbowAngle, wristAngle);

  
//      Serial.println(baseAngle);
//      Serial.println(shoulderAngle);
//      Serial.println(elbowAngle);
//      Serial.println(wristAngle);
      
      if (checkValidAngles(baseAngle, shoulderAngle, elbowAngle, wristAngle))
          {
            Serial.println("Moving");
            moveServo(baseServo, baseAngle);
            moveStepper(shoulderAngle);
            moveServo(elbowServo, elbowAngle);
            moveServo(wristServo, wristAngle);
      }
      else {
        return;
      }
    }
  }
  
}

void moveHorizontal(float finalX, float finalY) {

  float x, y, z;

  byte baseAngle = baseServo.read();
  byte shoulderAngle = shoulderCurrentAngle;
  byte elbowAngle = elbowServo.read();
  byte wristAngle = wristServo.read();

  anglesToCoords(baseAngle, shoulderAngle, elbowAngle, wristAngle, x, y, z);

  const float granularity = 0.1;

//  if (checkValidAngles(baseAngle, shoulderAngle, elbowAngle, wristAngle)) {
//    moveServo(baseServo, baseAngle);
//  }
//  else
//    return;

  float distance = sqrt((finalX - x)*(finalX - x) + (finalY - y)*(finalY - y));
  float angle;

  if (finalX - x == 0) {

    if (finalY > y)
      angle = M_PI;
     else
      angle = 0;
  }
  else if (finalY - y == 0) {
    if (finalX > x)
      angle = M_PI_2;
    else
      angle = -M_PI_2;
  }
  else {
    
    angle = atan((finalX - x) / (finalY - y));

    if (finalX > x && finalY < y)
      angle += M_PI;
  }

  for (; distance >= 0; distance -= granularity) {

    x += granularity * sin(angle);
    y += granularity * cos(angle);
    
    coordsToAngles(x, y, z, baseAngle, shoulderAngle, elbowAngle, wristAngle);

      
//      Serial.println(baseAngle);
//      Serial.println(shoulderAngle);
//      Serial.println(elbowAngle);
//      Serial.println(wristAngle);
    
    if (checkValidAngles(baseAngle, shoulderAngle, elbowAngle, wristAngle))
        {
          moveServo(baseServo, baseAngle);
          moveStepper(shoulderAngle);
          moveServo(elbowServo, elbowAngle);
          moveServo(wristServo, wristAngle);
    }
    else {
      return;
    }
  }
}

void liftArm() {
  moveVertical(10);
}

void lowerArm() {
  moveVertical(-3);
}

void openClaw() {
  moveServo(clawServo, 85);
}

void closeClaw() {
  moveServo(clawServo, 95);
}

void defaultPosition() {

  liftArm();
  moveHorizontal(3, 0);
  moveVertical(5);
}

void makeMove(byte col1, byte line1, byte col2, byte line2) {

  liftArm();

  float boardCellLength = boardLength / 8;

  float x = boardXOffset + boardCellLength / 2 + boardCellLength * (line1-1);
  float y = boardYOffset + boardCellLength / 2 + boardCellLength * (col1-1);

  moveHorizontal(x, y);

  openClaw();
  lowerArm();
  closeClaw();
  liftArm();

  x = boardXOffset + boardCellLength / 2 + boardCellLength * (line2-1);
  y = boardYOffset + boardCellLength / 2 + boardCellLength * (col2-1);

  moveHorizontal(x, y);

  lowerArm();
  openClaw();
  liftArm();
  closeClaw();

  defaultPosition();
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

    if (isLowerCase(text[0])) {

      char* buff;
      float x, y, z;
    
      switch(text[0]) {
        
        case CALIBRATE_CODE:
          Serial.println("Calibrating");
          calibrate();
          break;
        case VERTICAL_CODE:
          z = (float) atof(text+1);
          moveVertical(z);
          break;
        case HORIZONTAL_CODE:
          buff = strtok(text+1, ";\n");
          x = atof(buff);
          
          buff = strtok(NULL, ";\n");
          y = atof(buff);

          moveHorizontal(x, y);
          
          break;
          
        case INIT_CODE:

          buff = strtok(text+1, ";\n");
          boardLength = atof(buff);
          
          buff = strtok(NULL, ";\n");
          boardXOffset = atof(buff);
          
          buff = strtok(NULL, ";\n");
          boardYOffset = atof(buff);
          
          buff = strtok(NULL, ";\n");
          boardHeight = atof(buff);

          break;
          
        case BOARD_CODE:
        
          byte col1 = (byte) text[1];
          byte line1 = (byte) text[2];

          byte col2 = (byte) text[3];
          byte line2 = (byte) text[4];

          makeMove(col1, line1, col2, line2);
          
          break;
      }
    }
    else {
  
      byte angle = (byte) atoi(text+1);
  
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
        case BASE_CODE:
          Serial.println("Moving base");
          moveServo(baseServo, angle);
          break;
      }
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
