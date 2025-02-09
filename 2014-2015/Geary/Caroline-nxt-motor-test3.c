#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S2,     sonar2,         sensorSONAR)
#pragma config(Sensor, S3,     HTSPB,          sensorI2CCustom9V)
#pragma config(Sensor, S4,     HTGYRO,         sensorAnalogInactive)
#pragma config(Motor,  motorA,          hopperNXT,     tmotorNXT, PIDControl, reversed, encoder)
#pragma config(Motor,  motorB,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     left,          tmotorTetrix, PIDControl, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     right,         tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     hopper,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define sonar4 0
#define servo1 0
#define servo2 0

#include "geary-nomux-stuff.c"

//  nMotorTargetEncoder is coupled with the nMotorRunState function.
//    The nMotorRunState function watches the "state" of the motor in three conditions:
//       1. "runStateRunning" - While the motor is running
//       2. "runStateHoldPosition" - While the motor is begining to slow down.
//          nMotorEncoderTarget sets the condition when the motor begins to slow down.
//       3. "runStateIdle" - Reports back to the controller that the motor is in an
//           idle position; this is when the motor is stopped.


#define HOPPERTOLERANCE 10 // NXT encoder count slop around each position

void MoveHopper (int hopperTarget) {
  int motorUpSpeed = 30;
  int motorDownSpeed = -10;
  while (abs(nMotorEncoder[hopperNXT]-hopperTarget) > HOPPERTOLERANCE) {
 	  bool moveUp = nMotorEncoder[hopperNXT] < hopperTarget;
 	  bool ledColor = moveUp ? GREEN : RED;
 	  int motorSpeed = moveUp ? motorUpSpeed : motorDownSpeed;
   	motor[hopper] = motorSpeed;
   	LightLed(ledColor);
  }
  motor[hopper] = 0;
  LightsOff();
}

task main()
{
	InitProtoboardLights();  LightsOff();
  nMotorEncoder[hopperNXT] = 0;  //clear the LEGO motor encoders
  while (true) {
    MoveHopper (50);
    wait1Msec(1000);
  }

}
