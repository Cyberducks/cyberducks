#include "../drivers-3x/hitechnic-sensormux.h"  // needs to come before gyro.h
#include "../drivers-3x/hitechnic-superpro.h"
#include "../drivers-3x/hitechnic-gyro.h"
#include "../drivers-3x/lego-light.h"
#include "../drivers-3x/lego-ultrasound.h"

// NXT sensor ports:
// 1 - controller chain
// 2 - multiplexer
// 3 - protoboard
// 4 - color
//
// Multiplexer ports:
// 1 - light
// 2 - left sonar
// 3 - right sonar
// 4 - gyro

// ********************************* globals ************************************

float currHeading = 0;  // Current heading of the robot

const int SonarTolerance = 2;
const int HeadingTolerance = 4;
const int GyroFrequency = 30; // millisecs
int GyroSensitivity = 3;  // min. degrees per sec that means we're accelerating, 3 is really sensitive
const int GyroTolerance = 1;  // degrees, difference means adjust motors
const int ServoDelta = 2;
const int ServoSteps = 3;
const int ServoFreq = 100; // millisecs

const int DeadZone = 15;
const int Forward = -1;
const int Backward = 1;
const int TurnTimeLimit = 3500; // millisecs

int DriveByButtonSpeed = 10;

// see xander's gyro mux example: http://botbench.com/driversuite/hitechnic-gyro-_s_m_u_x-test1_8c-example.html
const tMUXSensor HTGYRO = msensor_S2_4;
// see xander's mux sample code:  http://botbench.com/driversuite/lego-ultrasound-_s_m_u_x-test1_8c-example.html
const tMUXSensor LEGOUS2 = msensor_S2_2;
const tMUXSensor LEGOUS3 = msensor_S2_3;
// see xander's light mux sample: http://botbench.com/driversuite/lego-light-_s_m_u_x-test1_8c-example.html
const tMUXSensor LEGOLS = msensor_S2_1;

const int GREEN = 0;
const int RED = 1;

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

// heading is never negative, but it can flip from 1 to 359, so do modulus 180
float SubtractFromCurrHeading (float x) {
  float result = 0.0;
  float ch = currHeading;
  float diff = abs(ch - x);
  if (diff >= 180) { // more than 180deg apart, so flip
    result = 360.0 - diff;
    if (x < 180) { result = -result; }
  } else {
    result = ch - x;
  }
  return result;
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

void PointStraightByGyro (float straightHead) {
      	// stay on heading from before button was pressed
        //nxtDisplayTextLine(1, "Offset: %4d", HTGYROreadCal(HTGYRO));
        nxtDisplayTextLine(2, "servo: %d", ServoValue[servo1]);
        nxtDisplayTextLine(3, "head: %3.0f", currHeading);
        nxtDisplayTextLine(4, "target: %3.0f", straightHead);
        nxtDisplayTextLine(5, "diff: %3.0f", SubtractFromCurrHeading (straightHead));
        if (SubtractFromCurrHeading (straightHead) > HeadingTolerance) {
        	StalkLeft();
        } else if (SubtractFromCurrHeading (straightHead) < -HeadingTolerance) {
          StalkRight();
        } else HTSPBwriteIO(HTSPB, 0x0); // indicate not turning
      wait1Msec(ServoFreq);
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
   while(ServoValue[servo2] > 20) {
    servo[servo2] = 20;
    }
}

void ExtendLightServo (void) {
	servoChangeRate[servo2] = ServoDelta;
	while(ServoValue[servo2] < 190)    {
    servo[servo2] = 190;
    }
}

void TrackBySonar (void) {
	  int range2 = USreadDist(LEGOUS2);
	  int range3 = USreadDist(LEGOUS3);
	  nxtDisplayCenteredBigTextLine(1, "%3d cm ", range3);
		nxtDisplayCenteredBigTextLine(6, "%3d cm ", range2);
    // move servo
		if ((range3 - range2) > SonarTolerance) {
      StalkLeft();
    } else if ((range2 - range3) > SonarTolerance) {
      StalkRight();
    }
    wait1Msec(ServoFreq);
}


void TurnLeft (int forwardOrBackward) {
          nxtDisplayTextLine(2, "turning left");
          HTSPBwriteIO(HTSPB, 0x1); // green means starboard, i.e. turn left, was... nxtDisplayTextLine(1, "turn Left");
          motor[left] = DriveByButtonSpeed * 1 / 8 * forwardOrBackward;
          motor[right] = DriveByButtonSpeed * 10 * forwardOrBackward;
}

void TurnRight (int forwardOrBackward) {
          nxtDisplayTextLine(2, "turning right");
          HTSPBwriteIO(HTSPB, 0x2); // red means port, i.e. turn right, was... nxtDisplayTextLine(1, "turn Right");
          motor[left] = DriveByButtonSpeed * 10 * forwardOrBackward;
          motor[right] =  DriveByButtonSpeed* 1 / 10 * forwardOrBackward;
}

void RotateLeft (int speed) {
          nxtDisplayTextLine(3, "rotating left");
          motor[left] = 0; // speed * 2 / 5 * Backward;
          motor[right] = speed * Forward;
}

void RotateRight (int speed) {
          nxtDisplayTextLine(3, "rotating right");
          motor[left] = speed * Forward;
          motor[right] = 0; // speed* 2 / 5 * Backward;
}

void GoStraight (int forwardOrBackward) {
          nxtDisplayTextLine(3, "going straight");
          LightBothLeds();
          motor[left] = DriveByButtonSpeed * forwardOrBackward * 7/4;
          motor[right] = DriveByButtonSpeed * forwardOrBackward;
}

void StopMotors (void) {
          motor[left] = 0;
          motor[right] = 0;
}

void GoStraightByGyro (float straightHead) {
      	// stay on heading from before button was pressed
        nxtDisplayTextLine(3, "head: %3.0f", currHeading);
        nxtDisplayTextLine(4, "target: %3.0f", straightHead);
        nxtDisplayTextLine(5, "diff: %3.0f", SubtractFromCurrHeading (straightHead));
        if (SubtractFromCurrHeading (straightHead) > GyroTolerance) {
          TurnLeft(Forward);
      } else if (SubtractFromCurrHeading (straightHead) < -GyroTolerance) {
          TurnRight(Forward);
      } else  {
          GoStraight(Forward);
      }
}

void GoStraightWithoutGyro () {
      	// stay on heading from before button was pressed
          TurnRight(Forward);
}

void GoBackByGyro (float straightHead) {
      	// stay on heading from before button was pressed
        if (SubtractFromCurrHeading (straightHead) < GyroTolerance) {
        	TurnLeft(Backward);
      } else if (SubtractFromCurrHeading (straightHead) > GyroTolerance) {
          TurnRight(Backward);
      } else  {
          GoStraight(Backward);
      }
}

task limitMotorTime() {
    ClearTimer(T2);
    while (time1[T2] < TurnTimeLimit) { wait10Msec(2); }
    StopMotors();
    PlayTone(1000,200);
    wait10Msec(100);
    StopAllTasks();
}

void Turn180 (void) {
	LightsOff();
	float startHead = currHeading;
	StartTask (limitMotorTime);
	wait1Msec(100);
	// currHeading starts at zero and goes up as we turn right
	// startHead is zero, so SubtractFromCurrHeading(startHead) == CurrHeading
	do {
		RotateRight (15);
		wait1Msec (10);
	} while (SubtractFromCurrHeading (startHead) < 90);
	do {
		RotateRight (15);
		wait1Msec (10);
	} while (SubtractFromCurrHeading (startHead) > 0); // goes 178,179,180,-1, so flips negative after 180
	do {
		RotateLeft (18);
		wait1Msec (10);
	} while (SubtractFromCurrHeading (startHead) < 0); // back to zero
	do {
		RotateRight (18);
		wait1Msec (10);
	} while (SubtractFromCurrHeading (startHead) > 0); // back to zero
  StopMotors();
	StopTask (limitMotorTime);
}

void VerifyMux (void) {
	  if (HTSMUXreadPowerStatus(S2)) {
	  	PlaySound(soundDownwardTones);
	  	StopMotors();
      wait1Msec(500);
      StopAllTasks();
    }
}

// ********************************* tasks ************************************


task AvoidWalls() {
    while((USreadDist(LEGOUS2) > 10) && (USreadDist(LEGOUS3) > 10)) { wait1Msec(300); }
    StopMotors();
    PlayTone(1000,200);
    wait1Msec(500);
    StopAllTasks();
}

// Task to keep track of the current heading using the HT Gyro, by Xander
task getHeading () {
	float delTime = 0;
	float prevHeading = 0;
	float curRate = 0;

  HTGYROstartCal(HTGYRO);
  while (true) {
    time1[T1] = 0;
    curRate = HTGYROreadRot(HTGYRO);
    if (abs(curRate) > GyroSensitivity) {  // seems to give spurious readings when stationary
      prevHeading = currHeading;
      currHeading = prevHeading + curRate * delTime;
      if (currHeading > 360) currHeading -= 360;
      else if (currHeading < 0) currHeading += 360;
    }
    wait1Msec(GyroFrequency);
    delTime = ((float)time1[T1]) / 1000;
  }
}
