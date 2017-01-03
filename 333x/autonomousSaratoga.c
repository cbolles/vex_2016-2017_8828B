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
string direction = "";

float posX = 0;
float posY = 0;
float theta = PI / 2;
float robotRadius = 0;

int degreesToTicks(int degrees)
{
	return (int)(degrees / 0.078);//0079 degrees per tick
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

void driveForward(int speed)
{
	motor[RI] = speed;
	motor[LE] = speed;
	direction = "forward";
}

void driveBackward(int speed)
{
	motor[RI] = -speed;
	motor[LE] = -speed;
	direction = "backward";
}

void driveLeftward(int speed)
{
	motor[BK] = speed;
	motor[FR] = speed;
}

void driveRightward(int speed)
{
	motor[BK] = -speed;
	motor[FR] = -speed;
}

void turnRight(int speed)
{
	motor[BK] = speed;
	motor[FR] = speed;
	motor[LE] = speed;
	motor[RI] = speed;
	direction = "right";
}

void turnLeft(int speed)
{
	motor[BK] = speed;
	motor[FR] = speed;
	motor[LE] = speed;
	motor[RI] = speed;
	direction = "left";
}

void stopBase()
{
	motor[BK] = 0;
	motor[FR] = 0;
	motor[LE] = 0;
	motor[RI] = 0;
	direction = "stopped";
}

float getDistanceTravled(short motorName)
{
	float wheelDiameter = 0;
	float circumfrence = PI * wheelDiameter;
	float axelRotation = nMotorEncoder[motorName] / 627.2; //627.2 represents the number of motor encoder ticks per rotation of axel
	return circumfrence * axelRotation;
}

float getDegreeTravled(short motorName)
{
	return getDistanceTravled(motorName) / robotRadius;
}

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

	//Clear theta
	if(theta > 2*PI)
	{
		theta = theta - 2*PI;
	}
}

void changePosition(short motor1, short motor2)
{
	float sumDistance = getDistanceTravled(motor1) + getDistanceTravled(motor2);
	float averageDistance = sumDistance/2;
	posX += averageDistance * cos(theta);
	posY += averageDistance * sin(theta);
}


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

void turnRightDegree(int targetTheta, int speed)
{
	while(theta < targetTheta)
	{
		turnRight(speed);
	}
	stopBase();
}

task main()
{



}
