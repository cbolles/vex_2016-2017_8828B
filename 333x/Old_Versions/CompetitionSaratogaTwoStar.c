
#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    rightPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    leftPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  topLimit,       sensorTouch)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_5,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           rightPincer,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           topLeft,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           FR,            tmotorVex393_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port5,           LE,            tmotorVex393_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port6,           RI,            tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port7,           BK,            tmotorVex393_MC29, openLoop, encoderPort, I2C_4)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           topRight,      tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_5)
#pragma config(Motor,  port10,          leftPincer,    tmotorVex393_HBridge, openLoop, reversed)
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

string direction = ""; //Direction the robot is moving, updated in real time

//Position values for robot, updated real time
float posX = 0;
float posY = 0;
float theta = PI / 2;

//Variables to control arm
int additionalPower = 0;
int lockArmPosition = 0;
bool lockArm = false;
int lockArmPositionUser = 0;

/*
<summary>
Takes in a degree value and returns the required ticks to move the
robot arm that given degrees
</summary>
<param id=degrees> Degree value in degrees not radians</param>
*/
int degreesToTicks(int degrees)
{
	return (int)(degrees / 0.078);//0079 degrees per tick
}

/*
<summary>
Moves all of the arm motors at one PWM value
</summary>
<param id=speed> PWM value, positive and negative matter </param>
*/
void moveArm(int speed)
{
	motor[topRight] = speed;
	motor[bottomRight] = speed;
	motor[topLeft] = speed;
	motor[bottomLeft] = speed;
	lockArmPosition = nMotorEncoder[topRight];
	lockArmPositionUser = nMotorEncoder[topRight];
}

/*
<summary>
Moves the robot arm based off of a given degree and speed
</summary>
<param id=degrees>
Value in degrees, represents a value that the arm moves, not
a location the arm move to</param>
<param id = speed> PWM value, positive values only!</param>
*/
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
	lockArmPositionUser = nMotorEncoder[topRight];
	additionalPower = 0;
}

/*
<summary>
Has the motors move at a given PWM speed forward only
</summary>
<param id=speed> PWM value, only positive!!</param>
*/
void driveForward(int speed)
{
	motor[RI] = speed;
	motor[LE] = -speed;
	direction = "forward";
}

/*
<summary>
Has the motors move at a given PWM speed backwards only
</summary>
<param id=speed> PWM speed, positive only!</param>
*/
void driveBackward(int speed)
{
	motor[RI] = -speed;
	motor[LE] = speed;
	direction = "backward";
}

/*
<summary>
Moves the robot left sideways(not turning)
</summary>
<param id = speed> PWM value, positive only!</param>
*/
void driveLeftward(int speed)
{
	motor[BK] = -speed;
	motor[FR] = speed;
}

/*
<summary>
Moves the robot right sideways(not turning)
</summary>
<param id = speed> PWM value, positive only!</param>
*/
void driveRightward(int speed)
{
	motor[BK] = speed;
	motor[FR] = -speed;
}

/*
<summary>
Spins the robot right
</summary>
<param id = speed> PWM value, positive only!</param>
*/
void turnRight(int speed)
{
	motor[BK] = speed;
	motor[FR] = speed;
	motor[LE] = speed;
	motor[RI] = speed;
	direction = "right";
}

/*
<summary>
Spins the robot left
</summary>
<param id = speed> PWM value, positive only!</param>
*/
void turnLeft(int speed)
{
	motor[BK] = -speed;
	motor[FR] = -speed;
	motor[LE] = -speed;
	motor[RI] = -speed;
	direction = "left";
}

/*
<summary>
Stops all motors on the base
</summary>
*/
void stopBase()
{
	motor[BK] = 0;
	motor[FR] = 0;
	motor[LE] = 0;
	motor[RI] = 0;
	direction = "stopped";
}

/*
<summary>
Calculates the distance one wheel has travled since last reading
</summary>
<param id=motorName> Name of motor that you want the distance travled of</param>
<return> the linear distance the robot has moved </return>
*/
float getDistanceTravled(short motorName)
{
	float wheelDiameter = 4;
	float circumfrence = PI * wheelDiameter;
	float axelRotation = abs(nMotorEncoder[motorName]) / 627.2; //627.2 represents the number of motor encoder ticks per rotation of axel
	nMotorEncoder[motorName] = 0;
	return circumfrence * axelRotation;
}

/*
<summary>
Uses the getDistanceTravled to convert the linear distance of a wheel into degrees the robot
has turned
</summary>
<param id = motorName> Name of motor that you want to get the angle of rotation from</param>
<return> Returns the angle the wheel has turned through</return>
*/
float getDegreeTravled(short motorName)
{
	float robotRadius = 7.25;
	return getDistanceTravled(motorName) / robotRadius;
}

/*
<summary>
Determines current angle the robot is pointed in after motion
</summary>
<param id=directionTurned>Either "Right" or "Left" used to proporly change theta</param>
<param "motors">All the motors to get angle from</param>
*/
void changeTheta(string directionTurned, short motor1, short motor2, short motor3, short motor4)
{
	float sumDegree = getDegreeTravled(motor1) + getDegreeTravled(motor2) + getDegreeTravled(motor3) + getDegreeTravled(motor4);
	float averageDegree = sumDegree / 4;
	if(directionTurned == "Right")
	{
		theta-=averageDegree;
	}
	else if(directionTurned == "Left")
	{
		theta+=averageDegree;
	}

	//Keep theta within 2 PI
	if(theta > 2*PI)
	{
		theta = theta - 2*PI;
	}
	else if(theta < 0)
	{
		theta += 2 *PI;
	}
}

/*
<summary>
Determes and adjusts the x y coordinate of the robot
</summary>
<param id = motors>Since a linear change is a two motor act, two motors are given as params</param>
*/
void changePosition(string direction, short motor1, short motor2)
{
	float sumDistance = getDistanceTravled(motor1) + getDistanceTravled(motor2);
	float averageDistance = sumDistance/2;
	if(direction == "forward")
	{
		posX += averageDistance * cos(theta);
		posY += averageDistance * sin(theta);
	}
	else if(direction == "backward")
	{
		posX -= averageDistance * cos(theta);
		posY -= averageDistance * sin(theta);
	}
}

/*
<summary>
Keeps track of the x,y,and theta of the robot as each commmand is given
</summary>
*/
task coordinate()
{
	while(true)
	{
		if(direction == "forward")
		{
			changePosition(direction, RI, LE);
		}
		else if(direction == "backward")
		{
			changePosition(direction, RI, LE);
		}
		else if(direction == "left")
		{
			string directionTurn = "Left";
			changeTheta(directionTurn, LE, RI, BK, FR);
		}
		else if(direction == "right")
		{
			string directionTurn = "Right";
			changeTheta(directionTurn, LE, RI, BK, FR);
		}

		if(lockArm == true)
		{
			if(lockArmPosition > nMotorEncoder[topRight]+10)
			{
				additionalPower += 2; //Increment by 2
			}
			else if(lockArmPosition < nMotorEncoder[topRight] -10)
			{
				additionalPower -= 1;
				if(additionalPower < 0)
				{
					additionalPower = 0;
				}
			}
			else
			{
				additionalPower = 0;
			}
			moveArm(additionalPower); //Zero if locked it on its own
		}
		else
		{
			moveArm(0);
		}
		wait10Msec(2); //Information can only be transfered every 20 ms
	}
}

/*
<summary>
Turns the robot a specific angle to the right
</summary>
<param id = targetTheta>Angle the robot should turn to IN RADIANS</param>
<param id = speed> PWM value, positive only!</param>
*/
void turnRightDegree(int targetTheta, int speed)
{
	while(theta > targetTheta)
	{
		turnRight(speed);
	}
	stopBase();
}

/*
<summary>
Turns the robot a specific angle to the left
</summary>
<param id = targetTheta>Angle the robot should turn to IN RADIANS</param>
<param id = speed> PWM value, positive only!</param>
*/
void turnLeftDegree(int targetTheta, int speed)
{
	while(theta < targetTheta)
	{
		turnLeft(speed);
	}
	stopBase();
}

/*
<summary>
Moves the robot a specific number of inches forward
</summary>
<param id= targetDistance> Distance the robot will travle in inches</param>
<param id= speed> PWM value, positive only!</param>
*/
void driveForwardInches(int targetDistance, int speed)
{
	float startPositionX = posX;
	float startPositionY = posY;
	float distance = 0;
	while(distance < targetDistance)
	{
		driveForward(speed);
		distance = sqrt(pow(posX - startPositionX, 2) + pow(posY - startPositionY, 2));
		wait10Msec(2);
	}
	stopBase();
}


/*
<summary>
Moves the robot a specific number of inches backwards
</summary>
<param id= targetDistance> Distance the robot will travle in inches</param>
<param id= speed> PWM value, positive only!</param>
*/
void driveBackwardInches(int targetDistance, int speed)
{
	float startPositionX = posX;
	float startPositionY = posY;
	float distance = 0;
	while(distance < targetDistance)
	{
		driveBackward(speed);
		distance = sqrt(pow(posX - startPositionX, 2) + pow(posY - startPositionY, 2));
		wait10Msec(2);
	}
	stopBase();
}

/*
<summary>
Tells the robot to move to a specific point on the field
</summary>
<param id=x> Target x position</param>
<param id=y> Target y position</param>
<param id=speed> PWM value, positive only!</param>
*/
void goToPoint(float x, float y, int speed)
{
	float xDistance = x - posX
	float yDistance = y - posY;
	float angleToPoint = acos(yDistance/xDistance);

	float distanceToPoint = sqrt(pow(posX - x, 2) + pow(posY - y, 2));

	if(theta - angleToPoint > 0)
	{
		turnRightDegree(theta - angleToPoint, speed);
	}
	else if(theta - angleToPoint < 0)
	{
		turnLeftDegree(angleToPoint - theta, speed);
	}
	driveForwardInches(distanceToPoint, speed);
}

void openPincer(int speed)
{
	int zeroRightPot = 180; //Values to set the potentiometer value to zero
	//int zeroLeftPot = 400;

	int positionFarRight = 280;
	//int positionFarLeft = 790;

	int tolorance = 100;

	while(SensorValue[rightPot] - zeroRightPot > positionFarRight)
	{
		motor[rightPincer] = speed;
		motor[leftPincer] = -speed;
	}

}

void closePincer(int speed)
{
	int zeroRightPot = 180; //Values to set the potentiometer value to zero
	//int zeroLeftPot = 400;

	int positionFarRight = 280;
	//int positionFarLeft = 790;

	int tolorance = 100;

	while(SensorValue[rightPot] - zeroRightPot < positionFarRight-300)
	{
		motor[rightPincer] = -speed;
		motor[leftPincer] = speed;
	}

}

task autonomous()
{
	nMotorEncoder[BK] = 0;
	nMotorEncoder[FR] = 0;
	nMotorEncoder[LE] = 0;
	nMotorEncoder[RI] = 0;
	nMotorEncoder[topRight] = 0;
	theta = PI/2;
	posX = 0;
	posY = 0;
	startTask(coordinate);

	//Drop Preload
	driveBackwardInches(10, 127);
	moveArmDegree(10, 60);
	lockArm = true;

	//Pick up preload
	driveForwardInches(6, 75);
	moveArmDegree(30, 60);

	//Drive up to wall
	driveBackwardInches(35, 127);

	//Dump Star
	moveArmDegree(60, 90);

	//Drop star on wall
	motor[rightPincer] = -75;
	wait1Msec(750);
	motor[rightPincer] = 0;

	moveArmDegree(-100, 75);

	wait1Msec(5000);
	stopTask(coordinate);
}

//Gets Joystick Values with dead zone
int * getJoystick()
{
	int joystickValues[] = {0,0,0};
	int minVal = 30;

	//Geting C1LX or ch4
	int currentVal = abs(vexRT[Ch4]);
	if(currentVal >= minVal)
	{
		joystickValues[0] = vexRT[Ch4];
	}
	else
	{
		joystickValues[0]= 0;
	}

	//Getting C1LY of ch3
	currentVal = abs(vexRT[Ch3]);
	if(currentVal >= minVal)
	{
		joystickValues[1] = vexRT[Ch3];
	}
	else
	{
		joystickValues[1] = 0;
	}

	//Getting C1Rx or ch1
	currentVal = abs(vexRT[Ch1]);
	if(currentVal >= minVal)
	{
		joystickValues[2] = vexRT[Ch1];
	}
	else
	{
		joystickValues[2] = 0;
	}
	return joystickValues;
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
void driveControl()
{
	int *joystickValues = getJoystick();
	int C1LX = joystickValues[0];
	int C1LY = joystickValues[1];
	int C1RX = joystickValues[2];

	motor[FR] = C1LX + C1RX;
	motor[LE] =  -C1LY + C1RX;
	motor[RI] =  C1LY + C1RX;
	motor[BK] = -C1LX + C1RX;
}

//Basic open and close of pincher claws
void movePincher()
{
	int motorSpeed = 50;
	if(vexRT[Btn8L])
	{
		motor[leftPincer] = motorSpeed;
	}
	else if(vexRT[Btn8R])
	{
		motor[leftPincer] = -motorSpeed;
	}
	else
	{
		motor[leftPincer] = 0;
	}

	if(vexRT[Btn7R])
	{
		motor[rightPincer] = -motorSpeed;
	}
	else if (vexRT[Btn7L])
	{
		motor[rightPincer] = motorSpeed;
	}
	else
	{
		motor[rightPincer] = 0;
	}
}

void pincherOpenClose()
{
	int zeroRightPot = 180; //Values to set the potentiometer value to zero
	int zeroLeftPot = 400;

	int positionOpenRight = 1700; //Potentiometer readings for each of the three positions and each arm
	int positionOpenLeft = 1700;
	int positionCloseRight = 3066;
	int positionCloseLeft = 3300;
	int positionFarRight = 280;
	int positionFarLeft = 790;

	int pincerSpeed = 80;

	int tolorance = 100;

	//Open Position control
	if(vexRT[Btn5D])
	{
		//right pincher
		if(SensorValue[rightPot]-zeroRightPot > positionOpenRight + tolorance)
		{
			motor[rightPincer] = pincerSpeed;
		}
		else if(SensorValue[rightPot]-zeroRightPot < positionOpenRight-tolorance)
		{
			motor[rightPincer] = -pincerSpeed;
		}
		else
		{
			motor[rightPincer] = 0;
		}

		//left pincher
		if(SensorValue[leftPot]-zeroLeftPot < positionOpenLeft-tolorance)
		{
			motor[leftPincer] = pincerSpeed;
		}
		else if(SensorValue[leftPot]-zeroLeftPot > positionOpenLeft+tolorance)
		{
			motor[leftPincer] = -pincerSpeed;
		}
		else
		{
			motor[leftPincer] = 0;
		}
	}

	//Close control
	else if(vexRT[Btn5U])
	{
		//right pincer
		if(SensorValue[rightPot]-zeroRightPot > positionCloseRight+tolorance)
		{
			motor[rightPincer] = pincerSpeed;
		}
		else if(SensorValue[rightPot]-zeroRightPot < positionCloseRight-tolorance)
		{
			motor[rightPincer] = -pincerSpeed;
		}
		else
		{
			motor[rightPincer] = 0;
		}

		//left pincher
		if(SensorValue[leftPot]-zeroLeftPot < positionCloseLeft-tolorance)
		{
			motor[leftPincer] = pincerSpeed;
		}
		else if(SensorValue[leftPot]-zeroLeftPot > positionCloseLeft+tolorance)
		{
			motor[leftPincer] = -pincerSpeed;
		}
		else
		{
			motor[leftPincer] = 0;
		}
	}
	else if(vexRT[Btn8D])//Getting into tight corners
	{
				//right pincer
		if(SensorValue[rightPot]-zeroRightPot > positionFarRight+tolorance)
		{
			motor[rightPincer] = pincerSpeed;
		}
		else if(SensorValue[rightPot]-zeroRightPot < positionFarRight-tolorance)
		{
			motor[rightPincer] = -pincerSpeed;
		}
		else
		{
			motor[rightPincer] = 0;
		}

		//left pincher
		if(SensorValue[leftPot]-zeroLeftPot < positionFarLeft-tolorance)
		{
			motor[leftPincer] = pincerSpeed;
		}
		else if(SensorValue[leftPot]-zeroLeftPot > positionFarLeft+tolorance)
		{
			motor[leftPincer] = -pincerSpeed;
		}
		else
		{
			motor[leftPincer] = 0;
		}
	}
}
void lockArmUser()
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
//Basic up down moving, when not suppose to move, adds power to make sure it wont move
void dumpControl()
{
	if(!vexRT[Btn7U]) //While not touching bottom sensor
	{
		if(vexRT[Btn6U])
		{
			additionalPower = 0;
			moveArm(75);
		}
		else if(vexRT[Btn6D])
		{
			additionalPower = 0;
			moveArm(-75);
		}
		else
		{
			lockArmUser();
		}
	}
	else //If bottom sensor pressed
	{
		moveArmDegree(1, 50);
		additionalPower = 0;
		lockArmPositionUser =nMotorEncoder[topRight];
	}
}

task usercontrol()
{
	nMotorEncoder[topRight] = 0;
	lockArmPositionUser = nMotorEncoder[topRight];
	additionalPower = 0;
	stopBase();
	moveArm(0);
	stopTask(coordinate);
	while(true)
	{
		driveControl();
		dumpControl();
		movePincher();
		pincherOpenClose();
		wait10Msec(2); //Motors can only be updated every 20ms
	}
}
