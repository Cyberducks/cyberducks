#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTServo,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     rightFront,    tmotorTetrix, PIDControl, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rightBack,     tmotorTetrix, PIDControl, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     leftBack,      tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S1_C2_2,     leftFront,     tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S2_C2_1,     beaterBar,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     lift,          tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    backRightServo,       tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    backLeftServo,        tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    rightServo,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    leftServo,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)

task main()
{
	nMotorEncoder[lift] = 0;
	while(true){
		nxtDisplayTextLine(3, "head: %3.0f", nMotorEncoder[lift]/4);
		if(nNxtButtonPressed != -1){
			if(nNxtButtonPressed == 1){
				motor[lift] = 50;
				wait1Msec(200);

			}else if(nNxtButtonPressed == 2){
				motor[lift] = -50;
				wait1Msec(200);
			}else if(nNxtButtonPressed == 3){

			}
		}
		motor[lift] = 0;
	}


}
