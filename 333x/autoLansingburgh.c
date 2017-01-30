#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    rightPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    leftPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  topLimit,       sensorTouch)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorTouch)
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

/*
Collin Bolles
1/24/2017

*/
/*--------------------------------Odometry Variables--------------------------------*/
float WHEEL_BASE = 30;
float LEFT_CLICKS_PER_INCH = 27;
float RIGHT_CLICKS_PER_INCH = 27;
float theta = PI/2;                    /* bot heading */
float X_pos=0;                    /* bot X position in inches */
float Y_pos=0;                    /* bot Y position in inches */
float traveled = 0;								//distanced traveled from set point
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

void driveForward(int targetDistance, int speed)
{
	float startPositionX = X_pos;
	float startPositionY = Y_pos;
	float distance = 0;
	while(distance < targetDistance)
	{
		driveForward(speed);
		distance = sqrt(pow(X_pos - startPositionX, 2) + pow(Y_pos - startPositionY, 2));
		wait10Msec(2);
	}
	stopBase();
}

void driveBackward(int targetDistance, int speed)
{
	float startPositionX = X_pos;
	float startPositionY = Y_pos;
	float distance = 0;
	while(distance < targetDistance)
	{
		driveBackward(speed);
		distance = sqrt(pow(X_pos - startPositionX, 2) + pow(Y_pos - startPositionY, 2));
		wait10Msec(2);
	}
	stopBase();
}

bool inRange(float value, float target, float tolerance)
{
	return abs(value-target) <= tolerance;
}

void turnRight(float targetTheta, int speed)
{
	while(!inRange(theta, targetTheta, 0.09))
	{
		turnRight(speed);
	}
	stopBase();
}

void turnLeft(float targetTheta, int speed)
{
	while(!inRange(theta, targetTheta, 0.09))
	{
		turnLeft(speed);
	}
	stopBase();
}

void goToPoint(float x, float y, int speed)
{
	float xDistance = x - X_pos+.01;
	float yDistance = y - Y_pos;
	float angleToPoint = atan2(yDistance,xDistance);
	float distanceToPoint = sqrt(pow(X_pos - x, 2) + pow(Y_pos - y, 2));
	if(angleToPoint < 0)
	{
		angleToPoint += 2*PI;
	}
	writeDebugStreamLine("%f:",angleToPoint);
	if(theta - angleToPoint > .09)
	{
		turnRight(angleToPoint, speed);
	}
	else if(theta - angleToPoint < -0.09)
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
	float left_inches, right_inches, inches;
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
		left_inches = (float)L_ticks/LEFT_CLICKS_PER_INCH;
		right_inches = (float)R_ticks/RIGHT_CLICKS_PER_INCH;

		//calculate distance the total robot has travled
		inches = (left_inches + right_inches) / 2.0;

		//change the angle of the robot
		theta += (left_inches - right_inches) / WHEEL_BASE;

		//Keeps the theta value within 0 and 2*PI radians
		if(theta >= 2*PI)
		{
			theta = theta - 2 * PI;
		}

		else if (theta < 0)
		{
			theta =  theta + 2 * PI;
		}

		y1 = Y_pos;
		x1 = X_pos;
		/* now calculate and accumulate our position in inches */
		Y_pos += inches * sin(theta);
		X_pos += inches * cos(theta);

		traveled += sqrt(pow(X_pos-x1,2) + pow(Y_pos-y1,2));//distance formula, used like a resetable sensor

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
		wait1Msec(50);//Allow for new values to come in before updating
	}
}

void setToDefault()
{
	nMotorEncoder[topRight] = 0;
	nMotorEncoder[backLeft] = 0;
	nMotorEncoder[backRight] = 0;
	X_pos = 0;
	Y_pos = 0;
	theta = 3*PI/2;
	additionalPower = 0;
}

/*---------------------------Auto Control Functions-----------------------------------*/
/*
In here will be functions to tell which value a potentiometer is at to determine which
auto to run
*/
/*------------------------------------------------------------------------------------*/

/*---------------------------Autos----------------------------------------------------*/
void basicAuto()
{
	//Drop Preload
	driveBackward(7, 127);
	moveArmDegree(10, 90);

	moveArmDegree(30, 60);

	turnLeft(3*PI/2+.17, 50);

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	//Drive up to wall
	driveBackward(30, 127);
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

	driveForward(45, 127);
	theta = 3*PI/2;
	turnLeft(5.7, 127);
	driveForward(35, 127);
	moveArmDegree(50, 100);
	theta = 5.85;

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	driveBackward(30, 127);
	turnRight(3*PI/2-.17, 75);
	driveBackward(40, 127);
	moveArmDegree(90, 127);
	lockArm = false;
}
/*------------------------------------------------------------------------------------*/

task main()
{
	clearTimer(T1);
	startTask(odometry);
	setToDefault();
	backStars();
	writeDebugStreamLine("%d:",time1(T1));
}
