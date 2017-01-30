#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  topLimit,       sensorTouch)
#pragma config(Sensor, dgtl2,  bottomLimit,    sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           bottomRight,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           topLeft,       tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port4,           FL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           FR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           BR,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           BL,            tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           bottomLeft,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           topRight,      tmotorVex393_MC29, openLoop, reversed)
task main()
{

	while(true)
	{
		if(vexRT[Btn7D])
		{
			motor[BR] = 75;
		}
		else if(vexRT[Btn7L])
		{
			motor[BL] = 75;
		}
		else if(vexRT[Btn7U])
		{
			motor[FL] = 75;
		}
		else if(vexRT[Btn7R])
		{
			motor[FR] = 75;
		}
		else
		{
			motor[FR] = 0;
			motor[FL] = 0;
			motor[BR] = 0;
			motor[BL] = 0;
		}
	}


}
