// ********************************* globals ************************************
#include "drivers/HTGYRO-driver.h"
float currHeading = 0;  // Current heading of the robot
const int SonarTolerance = 2;
const int ServoDelta = 2;
const int ServoSteps = 3;
const int ServoFreq = 30; // millisecs

// Task to keep track of the current heading using the HT Gyro, by Xander
task getHeading () {
	float delTime = 0;
	float prevHeading = 0;
	float curRate = 0;

  HTGYROstartCal(GYRO);
  //PlayStartSound();
  while (true) {
    time1[T1] = 0;
    curRate = HTGYROreadRot(GYRO);
    if (abs(curRate) > 3) {
      prevHeading = currHeading;
      currHeading = prevHeading + curRate * delTime;
      if (currHeading > 360) currHeading -= 360;
      else if (currHeading < 0) currHeading += 360;
    }
    wait1Msec(5);
    delTime = ((float)time1[T1]) / 1000;
    //delTime /= 1000;
  }
}
/*commenting these out becasue stalk is referenced
void TurnRight (void) {
  servoChangeRate[stalk] = ServoDelta;          // Slow the Servo Change Rate down to only 'delta' positions per update.
  short currPos = ServoValue[stalk];
  if (currPos < 255)    {
    nxtDisplayTextLine(1, "turn Right");
    servo[stalk] = currPos + ServoSteps;
    }
}

void TurnLeft (void) {
  servoChangeRate[stalk] = ServoDelta;          // Slow the Servo Change Rate down to only 'delta' positions per update.
  short currPos = ServoValue[stalk];
  if (currPos > 0)    {
    nxtDisplayTextLine(1, "turn Left");
    servo[stalk] = currPos - ServoSteps;
    }
}


void OrientStalkServo (void) {
    if(ServoValue[stalk] < 128)              // If stalk is closer to 0 (than 255):
    {
      while(ServoValue[stalk] < 255)               // While the ServoValue of stalk is less than 255:
      {
        servo[stalk] = 255;                              // Move stalk to position to 255.
      }
    }
    wait1Msec(1000);                          // Wait 1 second.
    if(ServoValue[stalk] >= 128)             // If stalk is closer to 255 (than 0):
    {
      while(ServoValue[stalk] > 0)                 // While the ServoValue of stalk is greater than 0:
      {
        servo[stalk] = 0;                                // Move stalk to position to 0.
      }
    }
    wait1Msec(1000);                          // Wait 1 second.

    servo[stalk] = 128; // go to center?
    wait1Msec(1000);
}*/

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

bool CurrHeadingIsLessThan (float x) {
  float delta = SubtractFromCurrHeading(x);
  return (delta < 0);
}

/*void TrackBySonar (void) {

	  int range2 = SensorValue[sonar2];
	  int range3 = SensorValue[sonar3];
	  nxtDisplayCenteredBigTextLine(1, "%3d cm ", range2);
		nxtDisplayCenteredBigTextLine(6, "%3d cm ", range3);
    // move servo
		if ((range2 - range3) > SonarTolerance) {
      TurnRight();
    } else if ((range3 - range2) > SonarTolerance) {
      TurnLeft();
    }
    wait1Msec(ServoFreq);
}*/
