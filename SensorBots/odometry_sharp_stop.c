#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    rightPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    leftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    autoControl,    sensorNone)
#pragma config(Sensor, in4,    accelRight,     sensorAnalog)
#pragma config(Sensor, in5,    accelLeft,      sensorAnalog)
#pragma config(Sensor, in6,    accelRightZ,    sensorAnalog)
#pragma config(Sensor, in7,    accelLeftZ,     sensorAnalog)
#pragma config(Sensor, dgtl1,  autoSensor,     sensorTouch)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorNone)
#pragma config(Sensor, dgtl3,  autoSensor,     sensorNone)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           frontLeft,     tmotorVex393_HBridge, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_3)
#pragma config(Motor,  port9,           backRight,     tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_4)
#pragma config(Motor,  port10,          backLeft,      tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
float WHEEL_BASE = 30;
float LEFT_CLICKS_PER_INCH = 27;
float RIGHT_CLICKS_PER_INCH = 27;
float theta = PI/2;                    /* bot heading */
float X_pos=0;                    /* bot X position in inches */
float Y_pos=0;                    /* bot Y position in inches */
float traveled = 0;								//distanced traveled from set point


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

//Using y=mx+b to calculate how to stop the robot
void driveForward(int targetDistance, int maxSpeed)
{
	//Linear Function
	float slope = -maxSpeed/targetDistance;

	float startPositionX = X_pos;
	float startPositionY = Y_pos;
	float distance = 0;
	int currentSpeed = maxSpeed;
	while(distance < targetDistance)
	{
		driveForward(currentSpeed);
		distance = sqrt(pow(X_pos - startPositionX, 2) + pow(Y_pos - startPositionY, 2));
		currentSpeed = slope*distance+maxSpeed;
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
		wait1Msec(60);//Allow for new values to come in before updating
	}
}

void setDefault()
{
	theta = PI/2;
	X_pos = 0;
	Y_pos = 0;
	nMotorEncoder[backLeft] = 0;
	nMotorEncoder[backRight] = 0;
}

task main()
{
	setDefault();
	startTask(odometry);
	driveForward(15, 127);

}
