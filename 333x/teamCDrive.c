#pragma config(Sensor, dgtl1,  armEncoder,     sensorQuadEncoder)
#pragma config(Motor,  port1,           leftArm2,      tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           backLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           frontLeft,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           leftArmWired,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           leftArm1,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rightArmWired, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           rightArm1,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           backRight,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           frontRight,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          rightArm2,     tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


void baseDrive()
{
	motor[backLeft] = vexRT[Ch3];
	motor[backRight] = vexRT[Ch2];

	motor[frontLeft] = vexRT[Ch3];
	motor[frontRight] = vexRT[Ch2];
}

void moveArm(int speed)
{
	motor[rightArm1] = speed;
	motor[rightArm2] = speed;
	motor[rightArmWired] = speed;
	motor[leftArm1] = speed;
	motor[leftArm2] = speed;
	motor[leftArmWired] = speed;
}

void controlArm()
{
	int maxSpeed = 90;
	if(vexRT[Btn6U])
	{
		moveArm(maxSpeed);
	}
	else if(vexRT[Btn6D])
	{
		moveArm(-maxSpeed);
	}
	else
	{
		moveArm(0);
	}
}

task main()
{
	while(true)
	{
		baseDrive();
		controlArm();
	}
}