#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  topLimit,       sensorTouch)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorTouch)
#pragma config(Sensor, dgtl3,  atWall,         sensorSONAR_inch)
#pragma config(Sensor, dgtl5,  indicateWall1,  sensorLEDtoVCC)
#pragma config(Sensor, dgtl6,  indicateWall2,  sensorLEDtoVCC)
#pragma config(Sensor, dgtl7,  indicateWall3,  sensorLEDtoVCC)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           topLeft,       tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port4,           FL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           FR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           BR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           BL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           topRight,      tmotorVex393_MC29, openLoop, reversed)

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
#define C1LX vexRT[Ch4]
#define C1LY vexRT[Ch3]
#define C1RX vexRT[Ch1]

int lockArmPosition = nMotorEncoder[topLeft];
int threshold = 15;

int getControlValues(int controllerInput)
{
	if(abs(controllerInput) > threshold)
	{
		return controllerInput;
	}
	return 0;
}

void moveShooter(int speed)
{
	motor[bottomLeft] = speed;
	motor[bottomRight] = speed;
	motor[topLeft] = speed;
	motor[topRight] = speed;
}

void driveControl()
{
	motor[FL] = getControlValues(C1LY) - getControlValues(C1LX) + getControlValues(C1RX);
	motor[FR] = -getControlValues(C1LY) - getControlValues(C1LX) + getControlValues(C1RX);
	motor[BR] =  -getControlValues(C1LY) + getControlValues(C1LX) + getControlValues(C1RX);
	motor[BL] = getControlValues(C1LY) + getControlValues(C1LX) + getControlValues(C1RX);
}

void indicateAtWall()
{
	if(SensorValue[atWall] <=2)
	{
		SensorValue[indicateWall1] = 1;
		SensorValue[indicateWall2] = 1;
		SensorValue[indicateWall2] = 1;
	}
	else
	{
		SensorValue[indicateWall1] = 0;
		SensorValue[indicateWall2] = 0;
		SensorValue[indicateWall2] = 0;
	}
}

//Basic up down moving, when not suppose to move, adds power to make sure it wont move
void dumpControl()
{
	if(vexRT[Btn6U] && !SensorValue[topLimit])
	{
		moveShooter(75);

		lockArmPosition = nMotorEncoder[topLeft];
	}
	else if(vexRT[Btn6D] && !SensorValue[bottomLimit])
	{
		moveShooter(-75);

		lockArmPosition = nMotorEncoder[topLeft];
	}
	else
	{
		if(nMotorEncoder[topLeft] > lockArmPosition)
		{
			moveShooter(-10);
		}
		else if(nMotorEncoder[topLeft] < lockArmPosition)
		{
			moveShooter(10);
		}
		else
		{
			moveShooter(0);
		}
	}
}
void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	// ..........................................................................
	// Insert user code here.
	// ..........................................................................

	// Remove this function call once you have "real" code.
	AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	// User control code here, inside the loop

	while (true)
	{
		driveControl();
		dumpControl();
		indicateAtWall();
		// This is the main execution loop for the user control program.
		// Each time through the loop your program should update motor + servo
		// values based on feedback from the joysticks.

		// ........................................................................
		// Insert user code here. This is where you use the joystick values to
		// update your motors, etc.
		// ........................................................................

		// Remove this function call once you have "real" code.
	}
}
