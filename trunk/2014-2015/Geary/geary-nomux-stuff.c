#include "../drivers-3x/hitechnic-superpro.h"
#include "../drivers-3x/lego-light.h"
#include "../drivers/lego-ultrasound.h"

// NXT sensor ports:
// 1 - controller chain
// 2 - left sonar
// 3 - protoboard
// 4 - gyro or right sonar or PS2 controller


// ********************************* globals ************************************

float currHeading = 0;  // Current heading of the robot

const int SonarTolerance = 2;
const int ServoDelta = 2;
const int ServoSteps = 3;
const int ServoFreq = 100; // millisecs

const int DeadZone = 15;
const int Forward = -1;
const int Backward = 1;
const int TurnTimeLimit = 3500; // millisecs

const int GREEN = 0;
const int RED = 1;

int DriveByButtonSpeed = 10;

// ********************************* procedures ************************************

void LightLed (int num) {
	HTSPBwriteIO(HTSPB, 1 << (num));
}

void LightBothLeds () {
	HTSPBwriteIO(HTSPB, 0x3);
}

void LightsOff () {
	HTSPBwriteIO(HTSPB, 0x0); // indicate not turning
}

// prepare i/o on protoboard for two lights
void InitProtoboardLights () {
  // Set B0 and B1 as output
  HTSPBsetupIO(HTSPB, 0x3);

  // say we are working correctly
  LightLed(GREEN);
  //HTSPBwriteIO(HTSPB, 0x1);
  wait1Msec(500);
  LightLed(RED);
  //HTSPBwriteIO(HTSPB, 0x2);
  wait1Msec(500);
  LightLed(GREEN);
  wait1Msec(500);
  LightLed(RED);
  wait1Msec(500);
  LightsOff();
  //HTSPBwriteIO(HTSPB, 0x0);
}

void StalkRight (void) {
  servoChangeRate[servo1] = ServoDelta;          // Slow the Servo Change Rate down to only 'delta' positions per update.
  short currPos = ServoValue[servo1];
  if (currPos < 255)    {
    HTSPBwriteIO(HTSPB, 0x1); // red means port, i.e. turn right, was... nxtDisplayTextLine(1, "turn Right");
    servo[servo1] = currPos + ServoSteps;
    }
}

void StalkLeft (void) {
  servoChangeRate[servo1] = ServoDelta;          // Slow the Servo Change Rate down to only 'delta' positions per update.
  short currPos = ServoValue[servo1];
  if (currPos > 0)    {
    HTSPBwriteIO(HTSPB, 0x2); // green means starboard, i.e. turn left, was... nxtDisplayTextLine(1, "turn Left");
    servo[servo1] = currPos - ServoSteps;
    }
}


void OrientStalkServo (void) {
    if(ServoValue[servo1] < 128)              // If servo1 is closer to 0 (than 255):
    {
      while(ServoValue[servo1] < 255)               // While the ServoValue of servo1 is less than 255:
      {
        servo[servo1] = 255;                              // Move servo1 to position to 255.
      }
    }
    wait1Msec(1000);                          // Wait 1 second.
    if(ServoValue[servo1] >= 128)             // If servo1 is closer to 255 (than 0):
    {
      while(ServoValue[servo1] > 0)                 // While the ServoValue of servo1 is greater than 0:
      {
        servo[servo1] = 0;                                // Move servo1 to position to 0.
      }
    }
    wait1Msec(1000);                          // Wait 1 second.

    servo[servo1] = 128; // go to center?
    wait1Msec(1000);
}

void RetractLightServo (void) {
	servoChangeRate[servo2] = ServoDelta;
  while(ServoValue[servo2] > 0) {
    servo[servo2] = 0;
    }
}

void ExtendLightServo (void) {
	servoChangeRate[servo2] = ServoDelta;
	while(ServoValue[servo2] < 110)    {
    servo[servo2] = 110;
    }
}

void TrackBySonar (void) {
	  int range2 = SensorValue[sonar2];
	  int range4 = SensorValue[sonar4];
	  nxtDisplayCenteredBigTextLine(1, "%3d cm ", range2);
		nxtDisplayCenteredBigTextLine(6, "%3d cm ", range4);
    // move servo
		if ((range2 - range4) > SonarTolerance) {
      StalkRight();
    } else if ((range4 - range2) > SonarTolerance) {
      StalkLeft();
    }
    wait1Msec(ServoFreq);
}

void TurnLeft (int forwardOrBackward) {
          nxtDisplayTextLine(2, "turning left");
          HTSPBwriteIO(HTSPB, 0x1); // green means starboard, i.e. turn left, was... nxtDisplayTextLine(1, "turn Left");
          motor[left] = DriveByButtonSpeed * 1 / 100 * forwardOrBackward;
          motor[right] = DriveByButtonSpeed * forwardOrBackward;
}

void TurnLeftGeary (int forwardOrBackward) {
          nxtDisplayTextLine(2, "turning left");
          HTSPBwriteIO(HTSPB, 0x1); // green means starboard, i.e. turn left, was... nxtDisplayTextLine(1, "turn Left");
          motor[left] = DriveByButtonSpeed * 1 / 8 * forwardOrBackward;
          motor[right] = DriveByButtonSpeed * 10 * forwardOrBackward;
}

void TurnRight (int forwardOrBackward) {
          nxtDisplayTextLine(2, "turning right");
          HTSPBwriteIO(HTSPB, 0x2); // red means port, i.e. turn right, was... nxtDisplayTextLine(1, "turn Right");
          motor[left] = DriveByButtonSpeed * forwardOrBackward;
          motor[right] =  DriveByButtonSpeed* 1 / 8 * forwardOrBackward;
}

void TurnRightGeary (int forwardOrBackward) {
          nxtDisplayTextLine(2, "turning right");
          HTSPBwriteIO(HTSPB, 0x2); // red means port, i.e. turn right, was... nxtDisplayTextLine(1, "turn Right");
          motor[left] = DriveByButtonSpeed * 10 * forwardOrBackward;
          motor[right] =  DriveByButtonSpeed* 1 / 10 * forwardOrBackward;
}

void RotateLeft (int speed) {
          nxtDisplayTextLine(2, "rotating left");
          motor[left] = speed * Backward;
          motor[right] = speed * Forward * 3/2;
}

void RotateRight (int speed) {
          nxtDisplayTextLine(2, "rotating right");
          motor[left] = speed * Forward * 3/2;
          motor[right] = speed * Backward;
}

void RotateLeftGeary (int speed) {
          nxtDisplayTextLine(2, "rotating left");
          motor[left] = 0; // speed * 2 / 5 * Backward;
          motor[right] = speed * Forward;
}

void RotateRightGeary (int speed) {
          nxtDisplayTextLine(2, "rotating right");
          motor[left] = speed * Forward;
          motor[right] = 0; // speed* 2 / 5 * Backward;
}

void GoStraight (int forwardOrBackward) {
          nxtDisplayTextLine(2, "going straight");
          LightsOff();
          motor[left] = DriveByButtonSpeed * forwardOrBackward;
          motor[right] = DriveByButtonSpeed * forwardOrBackward;
}

void GoStraightGeary (int forwardOrBackward) {
          nxtDisplayTextLine(3, "going straight");
          LightsOff();
          motor[left] = DriveByButtonSpeed * forwardOrBackward * 7/4;
          motor[right] = DriveByButtonSpeed * forwardOrBackward;
}

void StopMotors (void) {
          motor[left] = 0;
          motor[right] = 0;
          LightsOff();
}

task limitMotorTime() {
    ClearTimer(T2);
    while (time1[T2] < TurnTimeLimit) { wait10Msec(2); }
    StopMotors();
    PlayTone(1000,200);
    wait10Msec(100);
    StopAllTasks();
}

// ********************************* tasks ************************************

task AvoidWalls() {
    while(SensorValue[sonar2] > 10)   // While the Sonar Sensor readings are less than the specified, 'distance_in_cm':
      { wait1Msec(300); }
    StopMotors();
    PlayTone(1000,200);
    wait1Msec(500);
    StopAllTasks();
}

/* mux version
task AvoidWalls() {
    while((USreadDist(LEGOUS2) > 10) && (USreadDist(LEGOUS3) > 10)) { wait1Msec(300); }
    StopMotors();
    PlayTone(1000,200);
    wait1Msec(500);
    StopAllTasks();
}
*/
