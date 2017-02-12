#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    rightPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    leftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    leftFollower,   sensorLineFollower)
#pragma config(Sensor, in4,    rightFollower,  sensorLineFollower)
#pragma config(Sensor, dgtl1,  autoSensor,     sensorTouch)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorNone)
#pragma config(Sensor, dgtl3,  autoSensor,     sensorNone)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           rightPincer,   tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           topLeft,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           frontRight,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           frontLeft,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           backRight,     tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port7,           backLeft,      tmotorVex393_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           topRight,      tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_3)
#pragma config(Motor,  port10,          leftPincer,    tmotorVex393_HBridge, openLoop)

#include "autoInclude.h"

task main()
{
	setToDefault();
	startTask(odometry);

	basicAuto();
	openPincers();
	moveArmDegree(10, 127);

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	driveForward(90, 127);


	wait1Msec(1500);

	closePincers();
	wait1Msec(750);

	driveBackward(90, 127);
	moveArmDegree(70, 127);
	lockArm = false;

	moveArmDegree(-100, 127);


}
