#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     rightFront,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     rightBack,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     leftBack,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     leftFront,     tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

task main()
{
	 while (true)
  {
    getJoystickSettings(joystick);
    if(joystick.joy1_x1 > 50 && joystick.joy1_x2 > 50){
   	  motor[leftFront] = -abs(joystick.joy1_x1);
			motor[rightFront] = abs(joystick.joy1_x1);
			motor[leftBack] = abs(joystick.joy1_x1);
			motor[rightBack] = -abs(joystick.joy1_x1);
    }else if(joystick.joy1_x1 < -50 && joystick.joy1_x2 < -50){
    	motor[leftFront] = abs(joystick.joy1_x2);
			motor[rightFront] = -abs(joystick.joy1_x2);
			motor[leftBack] = -abs(joystick.joy1_x2);
			motor[rightBack] = abs(joystick.joy1_x2);
    }else if(joystick.joy1_TopHat != -1){
    	if(joystick.joy1_TopHat == 0){
    		motor[leftFront] = 20;
				motor[rightFront] = 20;
				motor[leftBack] = 20;
				motor[rightBack] = 20;
    	}else if(joystick.joy1_TopHat == 1){
				motor[rightFront] = 20;
				motor[leftBack] = 20;
    	}else if(joystick.joy1_TopHat == 2){
    		motor[leftFront] = -20;
				motor[rightFront] = 20;
				motor[leftBack] = 20;
				motor[rightBack] = -20;
    	}else if(joystick.joy1_TopHat == 3){
    		motor[leftFront] = -20;
				motor[rightBack] = -20;
    	}else if(joystick.joy1_TopHat == 4){
    		motor[leftFront] = -20;
				motor[rightFront] = -20;
				motor[leftBack] = -20;
				motor[rightBack] = -20;
    	}else if(joystick.joy1_TopHat == 5){
    		motor[rightFront] = -20;
				motor[leftBack] = -20;
    	}else if(joystick.joy1_TopHat == 6){
    		motor[leftFront] = 20;
				motor[rightFront] = -20;
				motor[leftBack] = -20;
				motor[rightBack] = 20;
    	}else if(joystick.joy1_TopHat == 7){
    		motor[leftFront] = 20;
				motor[rightBack] = 20;
    	}
    }else{
    	motor[leftFront] = joystick.joy1_y1;
			motor[rightFront] = joystick.joy1_y2;
			motor[leftBack] = joystick.joy1_y1;
			motor[rightBack] = joystick.joy1_y2;
    }

  }


}
