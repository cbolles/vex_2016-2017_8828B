#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorTouch)
#pragma config(Sensor, dgtl4,  wallSonar,      sensorSONAR_inch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           topLeft,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           FR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           LE,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           RI,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           BK,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           topRight,      tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
	12/3/2016
	Collin Bolles
	Knocks starts off of wall
	Dumps Pre-load
*/
int degreesToTicks(int degrees)
{
	return (int)(degrees / 0.078);//0079 degrees per tick
}

void driveForward(int speed)
{
	motor[RI] = -speed;
	motor[LE] = speed;
}

void driveSideways(char direction, int speed)
{
	if(direction == 'R')
	{
		motor[FR] = -speed;
		motor[BK] = speed;
	}
	else
	{
		motor[FR] = speed;
		motor[BK] = -speed;
	}
}

void moveShooter(int speed)
{
	motor[topRight] = speed;
	motor[bottomRight] = speed;
	motor[topLeft] = speed;
	motor[bottomLeft] = speed;
}

void moveShooterDegree(float degrees, int speed)
{
	nMotorEncoder[topRight] = 0;
	if(degrees < 0)
	{
		while(nMotorEncoder[topRight] > degreesToTicks(degrees))
		{
			moveShooter(-speed);
		}
	}
	else
	{
		while(nMotorEncoder[topRight] < degreesToTicks(degrees))
		{
			moveShooter(speed);
		}
	}
	moveShooter(0);
}

task main()
{
	nMotorEncoder[topRight] = 0;

	driveForward(75);//Knock off prload
	wait1Msec(750);

	moveShooterDegree(15, 50); //Lock stand offs into place
	moveShooterDegree(-10, 30);
	wait1Msec(250);
	while(SensorValue[wallSonar] > 1.5) //Goes up to wall
	{
		driveForward(110);
	}
	driveForward(0);

	moveShooterDegree(120, 75);//Dump Stars

	driveSideways('R', 50);//Knock off stars on wall
	wait1Msec(1000);
	driveSideways('L', 50);
	wait1Msec(2000);
	driveSideways('R', 0);
}
