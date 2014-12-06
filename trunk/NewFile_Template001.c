
task main()
{
	motor[motorB] = -100;
  motor[motorC] = 100;

  wait1Msec(1000);

  motor[motorB] = 0;
  motor[motorC] = 0;

}
