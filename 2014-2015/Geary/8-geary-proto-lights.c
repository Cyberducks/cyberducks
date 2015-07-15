#pragma config(Sensor, S3,     HTSPB,                sensorI2CCustom9V)

// see examples for hitechnic-superpro.h at http://botbench.com/driversuite/examples.html

#include "../drivers-3x/hitechnic-superpro.h"

task main() {

  // Set B0 and B1 as output
  HTSPBsetupIO(HTSPB, 0x3);

  while(true) {
  	//for (int pin = 0; pin <= 1; pin++) {
   //   outputdata = 1 << pin;
      HTSPBwriteIO(HTSPB, 0x1);
      wait1Msec(1000);
      HTSPBwriteIO(HTSPB, 0x2);
      wait1Msec(1000);
  }
}
