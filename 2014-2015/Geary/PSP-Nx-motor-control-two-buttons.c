/************************************************************************/
/*                                                                      */
/* Program Name: PSP-Nx-demo.c                                       */
/* ===========================                                          */
/*                                                                      */
/* Copyright (c) 2008 by mindsensors.com                                */
/* Email: info (<at>) mindsensors (<dot>) com                           */
/*                                                                      */
/* This program is free software. You can redistribute it and/or modify */
/* it under the terms of the GNU General Public License as published by */
/* the Free Software Foundation; version 3 of the License.              */
/* Read the license at: http://www.gnu.org/licenses/gpl.txt             */
/*                                                                      */
/************************************************************************/

/*
 * History
 * ------------------------------------------------
 * Author     Date      Comments
 * Deepak     08/01/14  Changed PSP-Nx-lib to '.h'.
 */
const ubyte Addr = 0x02;
const tSensors SensorPort = S1;        // Connect PSPNX sensorto this port!!
#include "PSP-Nx-lib.h"

int nLeftButton = 0;
int nRightButton = 0;
int nEnterButton = 0;
int nExitButton = 0;

//////////////////////////////////////////////////////////////////////////////
//
//      Display the  Gamepad buttons and joystick status  on Nxt screen
//
/////////////////////////////////////////////////////////////////////////////

task
main ()
{
  int powerA, powerC;
  int d_left_X, d_left_Y;
  int d_right_X, d_right_Y;

	psp currState;

  //
  // Note: program cannot be terminated if we hijack the 'exit' button. So there has to be an escape sequence
  //       that will return buttons to system control! We'll use a triple click
  //
  nNxtExitClicks = 3;                // Triple clicking EXIT button will terminate program
  //
  nI2CBytesReady[SensorPort] = 0;

  //SensorType[SensorPort] = sensorI2CCustom9V;        // or perhaps use 'Lowspeed_9V_Custom0'??
  SensorType[SensorPort] = sensorI2CMuxController;
  wait10Msec (100);

  nxtDisplayTextLine (0, "mindsensors.com");
  nxtDisplayTextLine (1, "PSP-NX");

  while ( true )
    {
      wait1Msec (5);

      PSP_ReadButtonState(SensorPort, Addr, currState);

      d_left_X = (int)currState.l_j_x;
      d_left_Y = (int)currState.l_j_y;
      d_right_X = (int)currState.r_j_x;
      d_right_Y = (int)currState.r_j_y;

      /*
      * Determine power for each motor.
       * in doing so, take the Y component of left joystick
       * and X component of right joystick
      */
      powerA = d_left_Y + d_right_X / 2;
      powerC = d_left_Y - d_right_X / 2;

		  motor[motorA] = powerA;
		  motor[motorC] = powerC;

    nxtDisplayTextLine(1,"motorA: %d", powerA);
    nxtDisplayTextLine(2,"motorC: %d", powerC);

    }

  wait10Msec (100);

  StopAllTasks ();
}
