#include "../drivers-3x/hitechnic-gyro.h"

// NXT sensor ports:
// 1 - controller chain
// 2 - left sonar
// 3 - protoboard
// 4 - gyro or right sonar or PS2 controller

// ********************************* globals ************************************

const int HeadingTolerance = 4;
const int GyroFrequency = 30; // millisecs
const int GyroSensitivity = 3;  // min. degrees per sec that means we're accelerating, 3 is really sensitive
const int GyroTolerance = 1;  // degrees, difference means adjust motors

// ********************************* tasks ************************************

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

// ********************************* procedures ************************************

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
          GoStraight(Forward);
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

void Turn180Geary (void) {
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

void Turn180 (int tolerance) {
	LightsOff();
	float startHead = currHeading;
	StartTask (limitMotorTime);
	wait1Msec(100);
	// currHeading starts at zero and goes up as we turn right
	// startHead is zero, so SubtractFromCurrHeading(startHead) == CurrHeading
	do {
		//nxtDisplayTextLine(3, "head: %3.0f", currHeading);
		//nxtDisplayTextLine(4, "target: %3.0f", startHead);
		//nxtDisplayTextLine(5, "diff: %3.0f", SubtractFromCurrHeading (startHead));
		RotateRight (30);
		wait1Msec (10);
	} while (SubtractFromCurrHeading (startHead) < (180-tolerance) && SubtractFromCurrHeading (startHead) >= (0-tolerance));
  StopMotors();
	StopTask (limitMotorTime);
}
