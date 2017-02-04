#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    rightPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    leftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    autoControl,    sensorNone)
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

/*--------------------------------Odometry Variables--------------------------------*/
float WHEEL_BASE = 76;
float LEFT_CLICKS_PER_CM = 12.28;
float RIGHT_CLICKS_PER_CM = 12.28;
float theta = 270;                    /* bot heading */
float X_pos=0;                    /* bot X position in inches */
float Y_pos=0;                    /* bot Y position in inches */
float traveled = 0;								//distanced traveled from set point
float currentVelocity = 0;
/*----------------------------------------------------------------------------------*/

/*-------------------------Advanced Calculations Variables--------------------------*/
float frictionForce = 0;
/*----------------------------------------------------------------------------------*/

/*--------------------------------Arm control Variables-----------------------------*/
int additionalPower = 0;
int lockArmPosition = 0;
bool lockArm = false;
int lockArmPositionUser = 0;
/*----------------------------------------------------------------------------------*/

/*----------------------------Arm movement functions--------------------------------*/
void moveArm(int speed)
{
	motor[topRight] = speed;
	motor[bottomRight] = speed;
	motor[topLeft] = speed;
	motor[bottomLeft] = speed;
	lockArmPosition = nMotorEncoder[topRight];
	lockArmPositionUser = nMotorEncoder[topRight];
}

int degreesToTicks(int degrees)
{
	return (int)(degrees / 0.078);//0079 degrees per tick
}

void moveArmDegree(float degrees, int speed)
{
	nMotorEncoder[topRight] = 0;
	if(degrees < 0)
	{
		while(nMotorEncoder[topRight] > degreesToTicks(degrees))
		{
			moveArm(-speed);
		}
	}
	else
	{
		while(nMotorEncoder[topRight] < degreesToTicks(degrees))
		{
			moveArm(speed);
		}
	}
	moveArm(0);
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;
}
/*----------------------------------------------------------------------------------*/

/*---------------------------Drive base functions-----------------------------------*/
void driveForward(int speed)
{
	motor[backRight] = speed;
	motor[backLeft] = speed;

	motor[frontRight] = speed;
	motor[frontLeft] = speed;
}

void driveBackward(int speed)
{
	motor[backRight] = -speed;
	motor[backLeft] = -speed;

	motor[frontRight] = -speed;
	motor[frontLeft] = -speed;
}

void driveLeftward(int speed)
{
	motor[frontRight] = speed;
	motor[frontLeft] = speed;

	motor[backRight] = -speed;
	motor[backLeft] = -speed;
}

void driveRightward(int speed)
{
	motor[frontRight] = -speed;
	motor[frontLeft] = -speed;

	motor[backRight] = speed;
	motor[backLeft] = speed;
}

void turnRight(int speed)
{
	motor[frontRight] = -speed;
	motor[frontLeft] = speed;

	motor[backRight] = -speed;
	motor[backLeft] = speed;
}

void turnLeft(int speed)
{
	motor[frontRight] = speed;
	motor[frontLeft] = -speed;

	motor[backRight] = speed;
	motor[backLeft] = -speed;
}

void stopBase()
{
	motor[frontRight] = 0;
	motor[frontLeft] = 0;

	motor[backRight] = 0;
	motor[backLeft] = 0;
}

float calculateStoppingDistance()
{
	float massOfRobot = 0;
	return (massOfRobot*pow(currentVelocity,2))/(2*frictionForce);
}

void driveForward(int targetDistance, int speed)
{
	traveled = 0;
	float stoppingDistance = 0;
	float pastVelocity = currentVelocity;
	while(traveled < targetDistance-stoppingDistance)
	{
		if(currentVelocity > pastVelocity)
		{
			stoppingDistance = calculateStoppingDistance();
		}
		driveForward(speed);
	}
	stopBase();
}

void driveBackward(int targetDistance, int maxSpeed)
{
	//Linear function to slow down robot
	float slope = -maxSpeed/targetDistance;
	int currentSpeed = maxSpeed;
	traveled = 0;
	while(traveled < targetDistance)
	{
		driveBackward(currentSpeed);
		currentSpeed = slope*traveled+maxSpeed;
	}
	stopBase();
}

bool inRange(float value, float target, float tolerance)
{
	return abs(value-target) <= tolerance;
}

void turnRight(float targetTheta, int speed)
{
	float targetThetaRadians = degreesToRadians(targetTheta);
	while(!inRange(theta, targetThetaRadians, 10))
	{
		turnRight(speed);
	}
	stopBase();
}

void turnLeft(float targetTheta, int speed)
{
	float targetThetaRadians = degreesToRadians(targetTheta);
	while(!inRange(theta, targetThetaRadians, 10))
	{
		turnLeft(speed);
	}
	stopBase();
}

void goToPoint(float x, float y, int speed)
{
	float xDistance = x - X_pos+.01;
	float yDistance = y - Y_pos;
	float angleToPoint = radiansToDegrees(atan2(yDistance,xDistance));
	float distanceToPoint = sqrt(pow(X_pos - x, 2) + pow(Y_pos - y, 2));
	if(angleToPoint < 0)
	{
		angleToPoint += 360;
	}
	writeDebugStreamLine("Angle_To_Point: %f",angleToPoint);
	if(theta - angleToPoint > 5)
	{
		turnRight(angleToPoint, speed);
	}
	else if(theta - angleToPoint < -5)
	{
		turnLeft(angleToPoint, speed);
	}
	driveForward(distanceToPoint, speed);
}
/*----------------------------------------------------------------------------------*/

task odometry()
{
	nMotorEncoder[backRight] = 0;
	nMotorEncoder[backLeft] = 0;
	int lsamp = 0;
	int rsamp = 0;
	int L_ticks = 0;
	int R_ticks = 0;
	int last_left = 0;
	int last_right = 0;
	float x1,y1;
	float left_cm, right_cm, cm;
	while (true)
	{
		lsamp = nMotorEncoder[backRight];
		rsamp = nMotorEncoder[backLeft];

		//Change in encoder values
		L_ticks = lsamp - last_left;
		R_ticks = rsamp - last_right;

		//Update past encodervalues for next pass
		last_left = lsamp;
		last_right = rsamp;

		//Calculate the distance each wheel has travled
		left_cm = (float)L_ticks/LEFT_CLICKS_PER_CM;
		right_cm = (float)R_ticks/RIGHT_CLICKS_PER_CM;

		//calculate distance the total robot has travled
		cm = (left_cm + right_cm) / 2.0;

		//change the angle of the robot
		float changeInTheta = (left_cm - right_cm) / WHEEL_BASE;
		theta += (changeInTheta*180)/PI;

		//Keeps the theta value within 0 and 2*PI radians
		if(theta >= 360)
		{
			theta = theta - 360;
		}

		else if (theta < 0)
		{
			theta =  theta + 360;
		}

		y1 = Y_pos;
		x1 = X_pos;
		/* now calculate and accumulate our position in inches */
		Y_pos += cm * sin(degreesToRadians(theta));
		X_pos += cm * cos(degreesToRadians(theta));

		float distanceTraveled = sqrt(pow(X_pos-x1,2) + pow(Y_pos-y1,2));
		traveled += distanceTraveled;
		currentVelocity = (distanceTraveled*1000)/60;
		if(lockArm)
		{
			if(lockArmPositionUser > nMotorEncoder[topRight]+3)
			{
				additionalPower += 2; //Increment by 2
			}
			if(lockArmPosition > nMotorEncoder[topRight] + 100)
			{
				lockArmPositionUser = nMotorEncoder[topRight];
			}
			moveArm(additionalPower); //Zero if locked it on its own
		}
		wait1Msec(60);//Allow for new values to come in before updating
	}
}

void setToDefault()
{
	nMotorEncoder[topRight] = 0;
	nMotorEncoder[backLeft] = 0;
	nMotorEncoder[backRight] = 0;
	X_pos = 0;
	Y_pos = 0;
	theta = 270;
	additionalPower = 0;
	traveled= 0;
}

/*---------------------------Auto Control Functions-----------------------------------*/
/*
In here will be functions to tell which value a potentiometer is at to determine which
auto to run
*/
/*------------------------------------------------------------------------------------*/
void calculateFrictionForce()
{
	traveled = 0;
	clearTimer(T1);
	while(traveled < 51)
	{
		driveForward(127);
	}
	stopBase();
	float finalVelocity = currentVelocity;
	traveled = 0;
	wait1Msec(750); //Wait for robot to completly stop
	float stoppingDistance = traveled;
	float massOfRobot = 0;
	frictionForce = (massOfRobot*pow(finalVelocity, 2))/(2*stoppingDistance);
	writeDebugStreamLine("friction_Force: %f", stoppingDistance);
}
/*

/*---------------------------Autos----------------------------------------------------*/
void basicAuto()
{
	//Drop Preload
	driveBackward(18, 127);
	moveArmDegree(10, 90);

	moveArmDegree(30, 60);

	turnLeft(280, 50);

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	calculateFrictionForce();
	//Drive up to wall
	driveBackward(25, 127);
	lockArm = false;

	//Dump Star
	moveArmDegree(95, 90);

	//Drop star on wall
	motor[leftPincer] = 127;
	motor[rightPincer] = 127;
	wait1Msec(1000);
	motor[leftPincer] = 0;
	motor[rightPincer] = 0;

	moveArmDegree(-100, 75);
}

void backStars()
{
	basicAuto();
	motor[leftPincer] = -127;
	motor[rightPincer] = -127;
	wait1Msec(750);
	motor[leftPincer] = 0;
	motor[rightPincer] = 0;

	driveForward(114, 127);
	theta = 270;
	turnLeft(327, 127);
	driveForward(35, 127);
	moveArmDegree(50, 100);
	theta = 327;

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	driveBackward(76, 127);
	turnRight(260, 75);
	driveBackward(102, 127);
	moveArmDegree(90, 127);
	lockArm = false;
}
/*------------------------------------------------------------------------------------*/
