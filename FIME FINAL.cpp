#pragma config(Sensor, S1,     soundSensor,    sensorSoundDBA)

const float RADIUS = 2.75;
const int ONE_ROT_TICKS = 763; // see main
const int DIST = 600;
bool fire = false;
bool falseAlarm = true;

bool moveForward(int dist);
void turnLeft();
void turnAround();
void turnRight();
void turnBy(int ticks);
void rotateToBeacon();
void extinguish();

int arr[1000]; // holds all distances travelled, if negative represents a rotation by that amount clockwise
int top = -1; // holds array index
void addElement(int a)
{
	top++;
	arr[top] = a;
}
void backToStart()
{
	turnAround();
	for (int i = top; i >= 0; i--)
	{
		if (arr[top] > 0)
			moveForward(arr[i]);
		else
			turnBy(arr[i] * -1);
	}
}
// Check for presence of IR beacon by wiggling the robot to check if the IR 
// beacon is still on wiggleSpeed determines the direction of turning and 
// wiggleAngle determines how much to turn
bool checkForIR(int wiggleSpeed, int wiggleAmount)
{
	bool present = false;

	motor[motorA] = motor[motorD] = 0;

	motor[motorA] = wiggleSpeed;
	motor[motorD] = -wiggleSpeed;
	nMotorEncoder[motorA] = 0;
	while (fabs(nMotorEncoder[motorA]) < wiggleAmount)
	{
		if (SensorValue[S2] != 0)
			present = true;
	}
	motor[motorA] = -wiggleSpeed;
	motor[motorD] = wiggleSpeed;
	while (fabs(nMotorEncoder[motorA]) > 0)
	{
		if (SensorValue[S2] != 0)
			present = true;
	}
	motor[motorA] = motor[motorD] = 0;
	return present;
}
// Operate motor for a certain amount of repetitions
void operateMotor(int repetitions, int motorSpeed, int motorRotationAmount, bool motorBBool)
{
	int motorNum = 0;
	// Depending on motorBBool, operate a different motor (true -> fire 
	// extinguisher arm, false -> water sprayer)
	if (motorBBool)
		motorNum = 1;
	else
		motorNum = 2;
	for (int count = 0; count < repetitions; count ++)
	{
		nMotorEncoder[motorNum] = 0;
		// Move the arm up and down
		if (motorSpeed > 0)
		{
			motor[motorNum] = -motorSpeed;
			while (nMotorEncoder[motorNum] > -motorRotationAmount)
			{}
			motor[motorNum] = motorSpeed;
			while (nMotorEncoder[motorNum] < 0)
			{}
		}
		else
		{
			motor[motorNum] = -motorSpeed;
			while (nMotorEncoder[motorNum] < motorRotationAmount)
			{}
			motor[motorNum] = motorSpeed;
			while (nMotorEncoder[motorNum] > 0)
			{}
		}
	}
	motor[motorNum] = 0;
}
void turnRight()
{
	int temp = nMotorEncoder[motorD];
	motor[motorA] = -20;
	motor[motorD] = 20;
	while(nMotorEncoder[motorD] < temp + (ONE_ROT_TICKS)/(float)(4)) 
	{
		if (getButtonPress(buttonEnter))
		{
			motor[motorD] = motor[motorA] = 0;
			rotateToBeacon();
			extinguish();
		}
	}
	motor[motorA] = motor[motorD] = 0;

	int netRotation = nMotorEncoder[motorD] - temp;
	if (netRotation > 0) // array stores negative numbers for rotations, positive is for moving forward
		netRotation = ONE_ROT_TICKS - netRotation;
	addElement(netRotation); // should just be -270
}
void turnLeft()
{
	int temp = nMotorEncoder[motorD];
	motor[motorA] = 20;
	motor[motorD] = -20;

	while(nMotorEncoder[motorD] > temp - (ONE_ROT_TICKS)/(float)(4))
	{
		if (getButtonPress(buttonEnter))
		{
			motor[motorD] = motor[motorA] = 0;
			rotateToBeacon();
			extinguish();
		}
	}
	motor[motorA] = motor[motorD] = 0;
	int netRotation = nMotorEncoder[motorD] - temp;
	if (netRotation > 0) // array stores negative numbers for rotations, positive is for moving forward
		netRotation = ONE_ROT_TICKS - netRotation;
	addElement(netRotation); // should just be -90
}
void turnAround()
{
	int temp = nMotorEncoder[motorD];
	motor[motorA] = -20;
	motor[motorD] = 20;

	while(nMotorEncoder[motorD] < temp + (ONE_ROT_TICKS)/(float)(2))
	{
		if (getButtonPress(buttonEnter))
		{
			motor[motorD] = motor[motorA] = 0;
			rotateToBeacon();
			extinguish();
		}
	}
	motor[motorA] = motor[motorD] = 0;

	int netRotation = nMotorEncoder[motorD] - temp;
	if (netRotation > 0) // array stores negative numbers for rotations, positive is for moving forward
		netRotation = ONE_ROT_TICKS - netRotation;
	addElement(netRotation); // should just be -180
}
void turnBy(int ticks)
{
	int temp = nMotorEncoder[motorD];

	motor[motorA] = -20;
	motor[motorD] = 20;
	while(nMotorEncoder[motorD] < temp + ticks)
	{}
	motor[motorA] = motor[motorD] = 0;

	int netRotation = nMotorEncoder[motorD] - temp;
	if (netRotation > 0) // array stores negative numbers for rotations, positive is for moving forward
		netRotation = ONE_ROT_TICKS - netRotation;
	addElement(netRotation);
}
bool moveForward(int dist)
{
	int temp = nMotorEncoder[motorD];
	int temp2 = 0;
	motor[motorA] = motor[motorD] = 10;
	while(nMotorEncoder[motorD] - temp < dist)
	{
		if (getButtonPress(buttonEnter))
		{
			motor[motorD] = motor[motorA] = 0;
			temp2 = nMotorEncoder[motorD];
			int displacement = (temp2 - temp);
			addElement(displacement);
			rotateToBeacon();
			extinguish();
			return false;
		}
		if (SensorValue[S3] < 10)
		{
			motor[motorA] = motor[motorD] = 0;
			temp2 = nMotorEncoder[motorD];
			int displacement = (temp2 - temp);
			addElement(displacement);
			return false;
		}
	}

	int displacement = (temp2 - temp);
	addElement(displacement); // displacement will not equal dist if it stopped for a wall

	return true;
}
void rotateToBeacon()
{
	time1[T1] = 0;
	wait1Msec(2000);
	int c = false;
	int temp = SensorValue[S4];
	// configure sensors

	if (SensorValue[S2] < 0)
	{
		motor[motorA] = 20;
		motor[motorD] = -20;
	}
	else
	{
		motor[motorA] = -20;
		motor[motorD] = 20;
	}

	while (fabs(SensorValue[S4]) <= 360 && !c)
	{
		if (SensorValue[S2] <= 1 && SensorValue[S2] >= -1)
		{
			c = checkForIR(10,50) || checkForIR(-10,50);
			// Unexpected case: If a loud noise is detected but the IR beacon is off, search for 20 seconds before setting fire to false
			if (falseAlarm && time1[T1] > 3000)
			{
				fire = false;
				c = true;
			}
		}

		if (getButtonPress(buttonEnter))
		{
			motor[motorD] = motor[motorA] = 0;
			rotateToBeacon();
			extinguish();
		}
	}
	motor[motorA] = motor[motorD] = 0;

	int netRotation = SensorValue[S4] - temp;
	if (netRotation > 0) // array stores negative numbers for rotations, positive is for moving forward
		netRotation = 360 - netRotation;
	addElement(netRotation);
}
void extinguish()
{
	// Check both ways to make sure the IR beacon is on
	bool tempPresent = checkForIR(10,50);
	bool present = checkForIR(-10,50);
	if (present || tempPresent)
		present = true;
	motor[motorB] = motor[motorC] = 0;

	falseAlarm = false;
	if (present)
		rotateToBeacon();
	bool motorBBool = true;
	// While the beacon is on, alternate between using the water spray and using
	// the arm, then check if the beacon is still on
	while (present)
	{
		if (present)
		{
			if (motorBBool)
				operateMotor(5,50,80, motorBBool);
			else
				operateMotor(10,-90,58, motorBBool);
			tempPresent = checkForIR(10,50);
			present = checkForIR(-10,50);

			if (present || tempPresent)
				present = true;
			else
				present = false;

			motorBBool = !motorBBool;
		}
	}
	fire = false;
	backToStart();
}

void search()
{

	bool tempPresent, present;
	bool obstacle = false;
	while (fire)
	{
		rotateToBeacon();

		tempPresent = checkForIR(10,50);
		present = checkForIR(-10,50);

		if (present || tempPresent)
			present = true;
		if (present)
		{
			if (!moveForward(250)) // goes until hits a wall or until 1 meter
			{
				obstacle = true;
				while (obstacle)
				{
					// basically tries to go around whatever is infront
					turnLeft();
					if (SensorValue[S3] < 10)
					{
						turnAround();
						// Unexpected case: If the robot has tried to rotate left and right 90 degrees and the obstacle is still there, then 
						// the robot assumes there is no path to the beacon, and the robot returns to its original position.
						if (SensorValue[S3] < 10)
						{
							obstacle = false;
							fire = false;
							backToStart();
						}
						else
						{
							moveForward(DIST);
							turnLeft();
						}
					}
					else
					{
						moveForward(DIST);
						turnRight();
					}
					wait1Msec(1000);

					if (SensorValue[S3] > 10)
						obstacle = false;
				}
			}
		}
		else
		{
			obstacle = false;
			fire = false;
			backToStart();
		}
	}
}
task main()
{
	nMotorEncoder[motorD] = 0;
	SensorType[S2] = sensorEV3_IRSensor;
	SensorType[S3] = sensorEV3_Ultrasonic;
	SensorType[S4] = sensorEV3_Gyro;

	wait1Msec(2000);

	SensorMode[S2] = modeEV3IR_Seeker;
	SensorMode[S4] = modeEV3Gyro_RateAndAngle;

	// Waits for a loud noise before operating
	while(SensorValue[S1] <= 90)
	{}

	fire = true;

	search();
	
	if (fire || falseAlarm)
		displayString(0, "Mission failed");
	else
		displayString(0, "Mission success");

	// Wait for a button press before ending the program
	while(!getButtonPress(buttonAny))
	{}
}
