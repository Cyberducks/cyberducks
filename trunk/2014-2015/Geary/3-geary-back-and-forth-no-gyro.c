#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S3,     HTSPB,                sensorI2CCustom9V)
#pragma config(Motor,  mtr_S1_C1_1,     left,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     right,         tmotorTetrix, PIDControl, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "geary-mux-stuff.c"

//////////////////////////////////////////////////////////////////////////////
//
//      MAIN
//
/////////////////////////////////////////////////////////////////////////////


task
main ()
{
  //int powerA, powerC;
  //int d_left_X, d_left_Y;
  //int d_right_X, d_right_Y;

  //bool go_forward, go_back, go_left, go_right, trigger_pressed;

  //float straightHead;

  InitProtoboardLights();
  wait1Msec(1000);

  StartTask(getHeading);
  wait1Msec(300);
  //StartTask(AvoidWalls);
  wait1Msec(300);


  float startHeading = currHeading;
  float flipHeading = startHeading + 180;
  const int onewayDistance = 6000;

  //for (int count = 1; count <= 1; count++) { // turn back and forth several times

	 // for (int i = 1; i < 200; i++) { // this timing will be done by encoder after I put an encoder on motor
		//  GoStraightByGyro(startHeading);
		//  wait1Msec(GyroFrequency*4);
		//}

    nMotorEncoder[right] = 0;
    while (nMotorEncoder[right] < onewayDistance) {
		  GoStraightWithoutGyro();
		}

	  Turn180();

	  nMotorEncoder[right] = 0;
	  while (nMotorEncoder[right] < onewayDistance) {
		  GoStraightWithoutGyro();
		}


  StopAllTasks ();
}
