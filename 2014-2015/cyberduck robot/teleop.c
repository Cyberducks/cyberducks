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
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//all naming is this program are based of positive y being forwards and positive x being right
#include "JoystickDriver.c"

short speedMod = 2;
short straifMod = 2;
short turnMod = 2;
short backMod = 2;
short frontMod = 2;
short zero = 6;
short liftPosition = 0;
bool lc = true;

void initializeRobot(){
	nMotorEncoder[lift] = 0;
  return;
}
void ySet(short speed){
	speed /= speedMod;
	motor[leftFront] = speed;
	motor[leftBack] = speed;
	motor[rightFront] = speed;
	motor[rightBack] = speed;
}
void xSet(short speed){
	speed /= straifMod;
	motor[leftFront] = -speed/frontMod;
	motor[leftBack] = speed/backMod;
	motor[rightFront] = speed/frontMod;
	motor[rightBack] = -speed/backMod;
}
void turn(short speed){
	speed /= turnMod;
	motor[leftFront] = speed;
	motor[leftBack] = speed;
	motor[rightFront] = -speed;
	motor[rightBack] = -speed;
}

task main(){
  initializeRobot();

  // wait for start of tele-op phase
  waitForStart();
  liftCheck();

  //actual control of the robot


  while (true){
		getJoystickSettings(joystick);
		//code for robot panning controlled by the left joystick
		if(joystick.joy1_y1 > zero){ //forwards
			ySet(joystick.joy1_y1);
		}else if(joystick.joy1_y1 < -zero){ //backwards
			ySet(joystick.joy1_y1);
		}else if(joystick.joy1_x1 < -zero){ //left
			xSet(joystick.joy1_x1);
		}else if(joystick.joy1_x1 > zero){ //right
			xSet(joystick.joy1_x1);
		}else if(joystick.joy1_x2 < -zero){ //left turn
			turn(joystick.joy1_x2);
		}else if(joystick.joy1_x2 > zero){ //right turn
			turn(joystick.joy1_x2);
		}else{
			ySet(0);
		}

		//button actions
		switch(joystick.joy1_Buttons){
			case 1:
				servo[backRightServo] = 50;
				servo[backLeftServo] = 190;
				break;
			case 2:
				servo[backRightServo] = 190;
	    	servo[backLeftServo] = 30;
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:

				break;
			case 6:

				break;
			case 7:
				break;
			case 8:
				break;
			default:
				break;
		}

		if(joystick.joy1_TopHat != -1){
    	if(joystick.joy1_TopHat == 0){
    		motor[beaterBar] = 50;
    	}else if(joystick.joy1_TopHat == 1){
    	}else if(joystick.joy1_TopHat == 2){
    	}else if(joystick.joy1_TopHat == 3){
    	}else if(joystick.joy1_TopHat == 4){
    		motor[beaterBar] = 0;
    	}else if(joystick.joy1_TopHat == 6){
      }else if(joystick.joy1_TopHat == 5){
    	}else if(joystick.joy1_TopHat == 7){
    	}
    }

    // the secound controller that does the 80-20 lifts and other attachments that are not the back goal clips

		if(joystick.joy2_y1 > zero){ //increment up

		}else if(joystick.joy2_y1 < -zero){ //increment down

		}

		}
	}
}
