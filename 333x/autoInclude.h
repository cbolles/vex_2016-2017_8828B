/**
*@author Collin Bolles
*This header file cointains all the code for the autonomous period of the robot. Included is code to 
*generate an x,y coordinate of the robot as it moves across the field. This allows for easier control 
*over the robot. Instead of having the robot move for an arbitary length of time, the robot moves for a 
*distance. Four autonomous periods are in this file.

*/
/*--------------------------------Odometry Variables--------------------------------*/
float WHEEL_BASE = 66;
float LEFT_CLICKS_PER_CM = 10.63;
float RIGHT_CLICKS_PER_CM = 10.63;
float theta = 270;                   ///robot heading in degres
float X_pos=0;                    	 //robot x position in cm
float Y_pos=0;                    	 // robot y position in cm
float traveled = 0;					 //
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

/**
*Runs all six VEX motors on the arm at a set PWM value
*@param speed PWM value, positive or negetive
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

/**
*Converts the target degrees of the arm to motor encoder ticks
*@param degrees Degress the arm should move
*/
int degreesToTicks(int degrees)
{
	return (int)(degrees / 0.078);//0078 degrees per tick
}

/**
*Has the arm move to a specific angle at a given PWM value
*@param degrees Degrees the arm should move, does not represent a position
*@param speed PWM value, positive only
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
	additionalPower = 0;
}
/*----------------------------------------------------------------------------------*/

/*---------------------------Drive base functions-----------------------------------*/
/**
*Has the robot drive forward at a given speed
*@param speed PWM value, positive only
*/
void driveForward(int speed)
{
	motor[backRight] = speed;
	motor[backLeft] = speed;

	motor[frontRight] = speed;
	motor[frontLeft] = speed;
}

/**
*Has the robot drive backward at a given speed
*@param speed PWM value, positive only
*/
void driveBackward(int speed)
{
	motor[backRight] = -speed;
	motor[backLeft] = -speed;

	motor[frontRight] = -speed;
	motor[frontLeft] = -speed;
}

/**
*Has the robot drive towards the left at a given speed
*@param speed PWM value, positive only
*/
void driveLeftward(int speed)
{
	motor[frontRight] = speed;
	motor[frontLeft] = speed;

	motor[backRight] = -speed;
	motor[backLeft] = -speed;
}

/**
*Has the robot drive towards the right at a given speed
*@param speed PWM value, positive only
*/
void driveRightward(int speed)
{
	motor[frontRight] = -speed;
	motor[frontLeft] = -speed;

	motor[backRight] = speed;
	motor[backLeft] = speed;
}

/**
*Turn the robot to to the right at a given speed
*@param speed PWM value, positive only
*/
void turnRight(int speed)
{
	motor[frontRight] = speed;
	motor[frontLeft] = -speed;

	motor[backRight] = speed;
	motor[backLeft] = -speed;
}

/**
*Turn the robot to to the left at a given speed
*@param speed PWM value, positive only
*/
void turnLeft(int speed)
{
	motor[frontRight] = -speed;
	motor[frontLeft] = speed;

	motor[backRight] = -speed;
	motor[backLeft] = speed;
}

/**
*Stops all base motors
*/
void stopBase()
{
	motor[frontRight] = 0;
	motor[frontLeft] = 0;

	motor[backRight] = 0;
	motor[backLeft] = 0;
}

/**
*Calculates the distance required for friction alone to stop the robot as it travles a t a given velocity.
*The equations uses the law of conservation of energy to calculate the stopping distance. The equation used is  below
*Change_In_Kinetic_Energy=Work_Done_On_System
*M=mass of robot, Vf=final velocity, V0=initial velocity, F=friction force, D=stopping distance.
*1/2MVf^2-1/2MV0^2=F*D In which Vf is zero (when it stops moving)
*D=(M*V0^2)/(2*F)
*/
float calculateStoppingDistance()
{
	float massOfRobot = 7.35;
	if(frictionForce == 0)
	{
		return 0;
	}
	return (massOfRobot*pow(currentVelocity,2))/(2*frictionForce);
}

/**
*Has the robot drive forward for a given distance at a given speed
*@param targetDistance Distance in cm the robot has to go
*@param speed PWM value, positive only
*/
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

/**
*Has the robot drive backward for a given distance at a given speed
*@param targetDistance Distance in cm the robot has to go
*@param speed PWM value, positive only
*/
void driveBackward(int targetDistance, int speed)
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
		driveBackward(speed);
	}
	stopBase();
}

/**
*Determines if the robot is within a target theta by a given tolerance
*@param value Current theta of the robot in degrees
*@param target Target theta in degrees
*@param tolerance How close the robot has to be to the target value (+-)
*@return True if the robot is within the tolerance of the target theta
*/
bool inRange(float value, float target, float tolerance)
{
	return abs(value-target) <= tolerance;
}

/**
*Has the robot turn right to a given angle at a given speed and tolerance
*@param targetTheta Target theta in degrees for robot to turn to
*@param speed PWM value, positive only
*@param tolerance Tolerance the robot theta can be to the target theta
*/
void turnRight(float targetTheta, int speed, int tolerance)
{
	while(!inRange(theta, targetTheta, tolerance))
	{
		turnRight(speed);
	}
	stopBase();
}

/**
*Has the robot turn left to a given angle at a given speed and tolerance
*@param targetTheta Target theta in degrees for robot to turn to
@param speed PWM value, positive only
@param tolerance Tolerance the robot theta can be to the target theta
*/
void turnLeft(float targetTheta, int speed, int tolerance)
{
	while(!inRange(theta, targetTheta, tolerance))
	{
		turnLeft(speed);
	}
	stopBase();
}

/*
	<summary>
	Has the robot move to a set coordinate on the field measured in cm
	</summary>
	<param id=x>Target x coordinate</param>
	<param id=y>Target y coordiante</param>
	<param id=speed>PWM value, positive only</param>
*/
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
		turnRight(angleToPoint, speed, 10);
	}
	else if(theta - angleToPoint < -5)
	{
		turnLeft(angleToPoint, speed, 10);
	}
	driveForward(distanceToPoint, speed);
}
/*----------------------------------------------------------------------------------*/

/**
*Calculates the x and y coordinates in cm and theta in degrees of the robot.
*Task has to be running to call base control functions
*/
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
		lsamp = nMotorEncoder[backLeft];
		rsamp = nMotorEncoder[backRight];

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

/**
*Has the right pincer open to a fixed position
*/
task openPincer()
{
	float positionOpen = 1960;
	int speed = 127;
	while(SensorValue[pincerPot] > positionOpen)
	{
		motor[rightPincer] = -speed;
		motor[leftPincer] = -speed;
	}
	motor[rightPincer] = 0;
	motor[leftPincer] = 0;
}

/**
*Has the right pincer close to a fixed position at a fixed speed
*/
task closePincer()
{
	float positionOpen = 4000;
	int speed = 127;
	while(SensorValue[pincerPot] < positionOpen)
	{
		motor[rightPincer] = speed;
		motor[leftPincer] = speed;
	}
	motor[rightPincer] = 0;
	motor[leftPincer] = 0;
}

/**
*Has the right pincer open as far as possible at a fixed speed
*/
task farPincer()
{
	float position = 0;
	int speed = 127;
	while(SensorValue[pincerPot] > position)
	{
		motor[rightPincer] = -speed;
		motor[leftPincer] = -speed;
	}
	motor[rightPincer] = 0;
	motor[leftPincer] = 0;
}

void farPincers()
{
	startTask(farPincer);
}

void openPincers()
{
	startTask(openPincer);
}

void closePincers()
{
	startTask(closePincer);
}

/**
*Has the right wheel align itself over the white midline at a fixed speed using a light sensor to check for the
*white electric tape midline.
*/
bool doneRight = false;
bool doneLeft = false;
int motorSpeed = 40;
task midLineRight()
{
	doneRight = false;
	int sensorValueDark = SensorValue[rightFollower];
	int light = sensorValueDark - 500;
	while(SensorValue[rightFollower] > light)
	{
		motor[frontRight] = motorSpeed;
		motor[backRight] = motorSpeed;
	}
	motor[frontRight] = 0;
	motor[backRight] = 0;
	wait1Msec(500);
	while(SensorValue[rightFollower] > light)
	{
		motor[frontRight] = -50;
		motor[backRight] = -50;
	}
	motor[frontRight] = 0;
	motor[backRight] = 0;
	doneRight = true;
}

/**
*Has the left wheel align itself over the white midline at a fixed speed using a light sensor to check for the
*white electric tape midline.
*/
task midLineLeft()
{
	doneLeft = false;
	int sensorValueDark = SensorValue[leftFollower];
	int light = sensorValueDark - 500;
	while(SensorValue[leftFollower] > light)
	{
		motor[frontLeft] = motorSpeed;
		motor[backLeft] = motorSpeed;
	}
	motor[frontLeft] = 0;
	motor[backLeft] = 0;
	wait1Msec(500);
	while(SensorValue[leftFollower] > light)
	{
		motor[frontLeft] = -50;
		motor[backLeft] = -50;
	}
	motor[frontLeft] = 0;
	motor[backLeft] = 0;
	doneLeft = true;
}

void goPastStartTile()
{
	int initalValue = SensorValue[threeStarLine];
	while(SensorValue[threeStarLine] > initalValue - 200)
	{
		driveBackward(127);
	}
	stopBase();
}

/**
*Has both the right and left wheels align themselves over the white electric tape midline. Since
*the robot bounces off the VEX wall at some unknown angle. This function realigns the robot
* to a known theta.
*/
void goToMidLine()
{
	startTask(midLineRight);
	startTask(midLineLeft);
	while(!doneRight || !doneLeft)//Wait for both tasks to end
	{

	}
}

/**
*Sets the default global varaibles. Must be called before any other function!
*/
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
/**
*Estimates the force of friction that exits between the tiles and the VEX wheels by appling the
*same equation mentions in calculateStoppingDistance() except the equation is calcualted for 
* force of friction.
*/
void calculateFrictionForce()
{
	traveled = 0;
	clearTimer(T1);
	while(traveled < 30)
	{
		driveBackward(127);
	}
	stopBase();
	float finalVelocity = currentVelocity;
	traveled = 0;
	wait1Msec(750); //Wait for robot to completly stop
	float stoppingDistance = traveled;
	float massOfRobot = 7.35;
	frictionForce = (massOfRobot*pow(finalVelocity, 2))/(2*stoppingDistance);
	writeDebugStreamLine("friction_Force: %f", stoppingDistance);
}

/**
*Function which controls how much additional power has to be sent to the robot to arm to keep it set at a given
*postion. As soon as input is no longer sent to the arm with in the form of an auto function call or a driver
*input, a lock position is set based off of a VEX motor encoder value. The function then works to keep the arm
*close to this lock position by adding power to the arm each time the arm falls below the tolerated position. 
*This allows the robot to hold weight without dropping the load.
*/
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
/*

/*---------------------------Autos----------------------------------------------------*/
/**
*Drops star onto claw, drive up to near wall. Dumps preload and opens pincers to knock all stars off the near wall
*/
void basicAuto(bool right)
{
	//Drop Preload
	driveBackward(18, 127);
	moveArmDegree(10, 90);

	moveArmDegree(30, 60);

	if(right)
	{
		turnLeft(290, 40, 15);
	}
	else
	{
		turnRight(250, 40, 15);
	}

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	//calculateFrictionForce();
	//Drive up to wall
	driveBackward(85, 127);
	lockArm = false;

	//Dump Star
	moveArmDegree(95, 90);

	//Drop star on wall
	motor[leftPincer] = 127;
	motor[rightPincer] = 127;
	wait1Msec(750);
	motor[leftPincer] = 0;
	motor[rightPincer] = 0;
	driveForward(20, 127);
	driveBackward(20, 127);
	moveArmDegree(-100, 75);
}

/**
*Runs basic auto, then goes to midline to realign then picks up cube, dumping it on the middle fence
*/
void cube(bool right)
{
	basicAuto(right);
	wait1Msec(750);
	openPincers();

	//Realign robot
	goToMidLine();
	theta = 270;

	//Turn to cube
	if(right)
	{
		turnLeft(350, 50, 10);
	}
	else
	{
		turnRight(180, 50, 10);
	}
	driveForward(75, 127);

	//Pick up cube
	closePincers();
	wait1Msec(500);
	moveArmDegree(40, 127);
	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	//Turn to wall
	if(right)
	{
		turnRight(270, 100, 30);
	}
	else
	{
		turnLeft(270, 100, 30);
	}
	driveBackward(40, 127);

	moveArmDegree(80, 127);
	lockArm = false;

	openPincers();
	wait1Msec(1000);
}

/**
*Picks up the back three stars and dump them on the near wall
*/
void backStars(bool right)
{
	theta = 0;

	driveBackward(30, 127);
	driveForward(90, 127);
	moveArmDegree(60, 127);

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	driveBackward(90, 127);
	if(right)
	{
		turnRight(300, 50, 10);
	}
	else
	{
		turnLeft(120, 50, 10);
	}
	driveBackward(100, 127);

	moveArmDegree(60, 127);
	lockArm = false;

	closePincers();
	wait1Msec(750);

}

/**
*Picks up the preload and the back single star and dumps them on the near wall
*/
void backTwo(bool right)
{
	theta = 180;

	driveBackward(60, 127);
	driveForward(85, 127);
	moveArmDegree(60, 127);

	lockArm = true;
	lockArmPosition = nMotorEncoder[topRight];
	additionalPower = 0;

	if(right)
	{
		turnLeft(280, 50, 10);
	}
	else
	{
		turnRight(100, 50, 10);
	}

	driveBackward(135, 127);
	moveArmDegree(75, 127);
	lockArm = false;
	closePincers();
	wait1Msec(750);
	openPincers();
	wait1Msec(500);
	moveArmDegree(-100, 127);
}
/*------------------------------------------------------------------------------------*/
