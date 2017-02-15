
/*--------------------------------Odometry Variables--------------------------------*/
float WHEEL_BASE = 66;
float LEFT_CLICKS_PER_CM = 10.63;
float RIGHT_CLICKS_PER_CM = 10.63;
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

/*
	<summary>
	Runs the arm at a given value
	</summary
	<param id=speed>PWM value, positive or negetive</param>
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
	Converts the target degrees of the arm to motor encoder ticks
	</summary>
	<param id=degrees> Degress the arm should move</param>
*/
int degreesToTicks(int degrees)
{
	return (int)(degrees / 0.078);//0079 degrees per tick
}

/*
	<summary>
	Has the arm move to a specific angle at a given speed
	</summary>
	<param id=degrees>Degrees the arm should move, does not represent a position</param>
	<param id=speed>PWM value, positive only</param>
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
/*
	<summary>
	Has the robot drive forward at a given speed
	</summary>
	<param id=speed>PWM value, positive only</param>
*/
void driveForward(int speed)
{
	motor[backRight] = speed;
	motor[backLeft] = speed;

	motor[frontRight] = speed;
	motor[frontLeft] = speed;
}

/*
	<summary>
	Has the robot drive backward at a given speed
	</summary>
	<param id=speed>PWM value, positive only</param>
*/
void driveBackward(int speed)
{
	motor[backRight] = -speed;
	motor[backLeft] = -speed;

	motor[frontRight] = -speed;
	motor[frontLeft] = -speed;
}

/*
	<summary>
	Has the robot drive towards the left at a given speed
	</summary>
	<param id=speed>PWM value, positive only</param>
*/
void driveLeftward(int speed)
{
	motor[frontRight] = speed;
	motor[frontLeft] = speed;

	motor[backRight] = -speed;
	motor[backLeft] = -speed;
}

/*
	<summary>
	Has the robot drive towards the right at a given speed
	</summary>
	<param id=speed>PWM value, positive only</param>
*/
void driveRightward(int speed)
{
	motor[frontRight] = -speed;
	motor[frontLeft] = -speed;

	motor[backRight] = speed;
	motor[backLeft] = speed;
}

/*
	<summary>
	Turn the robot to to the right at a given speed
	</summary>
	<param id=speed>PWM value, positive only</param>
*/
void turnRight(int speed)
{
	motor[frontRight] = -speed;
	motor[frontLeft] = speed;

	motor[backRight] = -speed;
	motor[backLeft] = speed;
}

/*
	<summary>
	Turn the robot to to the left at a given speed
	</summary>
	<param id=speed>PWM value, positive only</param>
*/
void turnLeft(int speed)
{
	motor[frontRight] = speed;
	motor[frontLeft] = -speed;

	motor[backRight] = speed;
	motor[backLeft] = -speed;
}

/*
	<summary>
	Stops all base motors
	</summary>
*/
void stopBase()
{
	motor[frontRight] = 0;
	motor[frontLeft] = 0;

	motor[backRight] = 0;
	motor[backLeft] = 0;
}

/*
	<summary>
	Calulates how far the robot has to go for friction to entierly stop the robot.
	Requires calculateFrictionForce to run once before hand
	</summary>
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

/*
	<summary>
	Has the robot drive forward for a given distance at a given speed
	</summary>
	<param id=targetDistance>Distance in cm the robot has to go</param>
	<param id=speed>PWM value, positive only</param>
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

/*
	<summary>
	Has the robot drive backward for a given distance at a given speed
	</summary>
	<param id=targetDistance>Distance in cm the robot has to go</param>
	<param id=speed>PWM value, positive only</param>
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

/*
	<summary>
	Determines if the robot is within a target theta by a given tolerance
	</summary>
	<param id=value>Current theta of the robot in degrees</param>
	<parma id=target>Target theta in degrees</param>
	<parma id=tolerance>How close the robot has to be to the target value (+-)</param>
*/
bool inRange(float value, float target, float tolerance)
{
	return abs(value-target) <= tolerance;
}

/*
	<summary>
	Has the robot turn right to a given angle at a given speed and tolerance
	</summary>
	<param id=targetTheta>Target theta in degrees for robot to turn to</param>
	<param id=speed>PWM value, positive only</parma>
	<param id=tolerance>Tolerance the robot theta can be to the target theta</param>
*/
void turnRight(float targetTheta, int speed, int tolerance)
{
	while(!inRange(theta, targetTheta, tolerance))
	{
		turnRight(speed);
	}
	stopBase();
}

/*
	<summary>
	Has the robot turn left to a given angle at a given speed and tolerance
	</summary>
	<param id=targetTheta>Target theta in degrees for robot to turn to</param>
	<param id=speed>PWM value, positive only</parma>
	<param id=tolerance>Tolerance the robot theta can be to the target theta</param>
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

/*
	<summary>
	Calculates the x and y coordinates in cm and theta in degrees of the robot.
	Task has to be running to call base control functions
	</summary>
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

/*
	<summary>
	Has the right pincer open to a fixed position
	</summary>
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

/*
	<summary>
	Has the right pincer close to a fixed position at a fixed speed
	</summary>
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

/*
	<summary>
	Has the right pincer open as far as possible at a fixed speed
	</summary>
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
/*
	<summary>
	Calls the open pincer tasks
	</summary>
*/
void openPincers()
{
	startTask(openPincer);
}

/*
	<summary>
	Calls the close pincer tasks
	</summary>
*/
void closePincers()
{
	startTask(closePincer);
}

/*
	<summary>
	Has the right wheel align itself over the white midline at a fixed speed
	</summary>
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

/*
	<summary>
	Has the left wheel align itself over the white midline at a fixed speed
	</summary>
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

/*
	<summary>
	Has both wheels align themselves over the white midline
	</summary>
*/
void goToMidLine()
{
	startTask(midLineRight);
	startTask(midLineLeft);
	while(!doneRight || !doneLeft)//Wait for both tasks to end
	{

	}
}

/*
	<summary>
	Sets the default global varaibles. Must be called
	before any other function!
	</summary>
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
/*
	<summary>
	Calculates the friction present on the robot
	</summary>
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

/*
	<summary>
	Allows the user to lock the arm at a given position
	</summary>
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
/*
	<summary>
	Drops star onto claw, drive up to near wall. Dumps preload
	and opens pincers to knock all stars off the near wall
	</summary>
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

/*
	<summary>
	Runs basic auto, then goes to midline to realign then
	picks up cube, dumping it on the middle fence
	</summary>
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

/*
	<summary>
	Picks up the back three stars and dump them on the near wall
	</summary>
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

/*
	<summary>
	Picks up the preload and the back single star and dumps
	them on the near wall
	</summary>
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
