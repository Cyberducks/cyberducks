#pragma config(Sensor, S1,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  motorB,          leftMotor,     tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          rightMotor,    tmotorNXT, PIDControl, encoder)

task main()
{

	int degreesToTurn = 90;
	float degreesSoFar = 0;
	int initialTurnReading = SensorValue[gyro];

	while(degreesSoFar < degreesToTurn)
	{
		wait1Msec(10);
		int currentGyroReading = SensorValue[gyro] - initialTurnReading;
		degreesSoFar = degreesSoFar + currentGyroReading * .01;
	}
	motor[motorB] = 0;

}
