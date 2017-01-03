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

double posX = 0;
double posY = 0;
double theta = 0;
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
}

void driveBackward(int speed)
{
	motor[RI] = -speed;
	motor[LE] = -speed;
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
}

void turnLeft(int speed)
{
	motor[BK] = speed;
	motor[FR] = speed;
	motor[LE] = speed;
	motor[RI] = speed;
}

double getDistanceTravled(short motorName)
{
	double wheelDiameter = 0;
	double circumfrence = PI * wheelDiameter;
	double axelRotation = nMotorEncoder[motorName] / 627.2; //627.2 represents the number of motor encoder ticks per rotation of axel
	return circumfrence * axelRotation;
}

task coordinate()
{
	while(true)
	{
		if(direction == "forward")
		{

		}
		else if(direction == "backward")
		{
		}
		else if(direction == "left")
		{
		}
		else if(direction == "right")
		{
		}
		else if(direction == "turnLeft")
		{
		}
		else if(direction == "turnRight")
		{
		}
	}
}

task main()
{



}
