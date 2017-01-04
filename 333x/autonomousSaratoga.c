#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    rightPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    leftPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  topLimit,       sensorTouch)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           rightPincer,   tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           topLeft,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           FR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           LE,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           RI,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           BK,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           topRight,      tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port10,          leftPincer,    tmotorVex393_HBridge, openLoop, reversed)

//XY coordinates system control variables
string direction = ""; //Direction the robot is moving, updated in real time

//Position values for robot, updated real time
float posX = 0;
float posY = 0;
float theta = PI / 2;

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

/*
	<summary>
	Has the motors move at a given PWM speed forward only
	</summary>
	<param id=speed> PWM value, only positive!!</param>
*/
void driveForward(int speed)
{
	motor[RI] = speed;
	motor[LE] = speed;
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
	motor[LE] = -speed;
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
	motor[BK] = speed;
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
	motor[BK] = -speed;
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
	motor[BK] = speed;
	motor[FR] = speed;
	motor[LE] = speed;
	motor[RI] = speed;
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
	float wheelDiameter = 0;
	float circumfrence = PI * wheelDiameter;
	float axelRotation = nMotorEncoder[motorName] / 627.2; //627.2 represents the number of motor encoder ticks per rotation of axel
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
	float robotRadius = 0;
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
}

/*
	<summary>
	Determes and adjusts the x y coordinate of the robot
	</summary>
	<param id = motors>Since a linear change is a two motor act, two motors are given as params</param>
*/
void changePosition(short motor1, short motor2)
{
	float sumDistance = getDistanceTravled(motor1) + getDistanceTravled(motor2);
	float averageDistance = sumDistance/2;
	posX += averageDistance * cos(theta);
	posY += averageDistance * sin(theta);
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
		if(direction == "forward" || direction == "backward")
		{
			changePosition(RI, LE);
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
	while(theta < targetTheta)
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
	while(theta > targetTheta)
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
	int distance = 0;
	while(distance < targetDistance)
	{
		driveForward(speed);
		distance += (getDistanceTravled(RI) + getDistanceTravled(LE)) / 2;
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
	int distance = 0;
	while(distance < targetDistance)
	{
		driveBackward(speed);
		distance += (getDistanceTravled(RI) + getDistanceTRavled(LE)) / 2;
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
	float xDistance = posX - x;
	float yDistance = posY - y;
	float angleToPoint = acos(yDistance/xDistance);

	float distanceToPoint = sqrt(pow(posX - x, 2) + pow(posY - y, 2);

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

task main()
{
//Likly about 3 different autos will be programmed here


}
