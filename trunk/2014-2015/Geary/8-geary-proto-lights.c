#pragma config(Sensor, S3,     HTSPB,                sensorI2CCustom9V)

// see examples for hitechnic-superpro.h at http://botbench.com/driversuite/examples.html

//#include "../drivers-3x/hitechnic-superpro.h"
#define sonar2 0
#define sonar4 0
#define left 0
#define right 0
#define servo1 0
#define servo2 0
#include "geary-nomux-stuff.c"

task main() {

  // Set B0 and B1 as output
  // HTSPBsetupIO(HTSPB, 0x3);
  InitProtoboardLights();

  while(true) {
  	//for (int pin = 0; pin <= 1; pin++) {
   //   outputdata = 1 << pin;
      LightLed(GREEN); // HTSPBwriteIO(HTSPB, 0x1);
      wait1Msec(1000);
      LightLed(RED);  //HTSPBwriteIO(HTSPB, 0x2);
      wait1Msec(1000);
  }
}
