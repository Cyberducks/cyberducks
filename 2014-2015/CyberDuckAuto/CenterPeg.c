#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTServo,  HTMotor,  none,     none)
#pragma config(Sensor, S4,     HTSMUX,         sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     rightFront,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rightBack,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     leftBack,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     leftFront,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     beaterBar,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     lift,          tmotorTetrix, openLoop, encoder)
#pragma config(Servo,  srvo_S2_C1_1,    backRightServo,       tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    backLeftServo,        tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    topServo,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_4,    fghjk,                tServoStandard)
#pragma config(Servo,  srvo_S2_C1_5,    sdfgbhnm,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_6,    sdfghj,               tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "../drivers-3x/hitechnic-sensormux.h"
#include "../drivers-3x/hitechnic-irseeker-v2.h"
#include "../drivers-3x/hitechnic-gyro.h"
const tMUXSensor IRS1 = msensor_S4_1;
const tMUXSensor IRS2 = msensor_S4_2;
tMUXSemsor Gyro
task main()
{



}
