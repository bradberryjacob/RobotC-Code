/*
* This file was written by Jacob Bradberry from the Gwinnett School of Mathematics, Science, and Technology for his 2012 - 2013 Senior Project
* Please give credit where credit is due
* If you have any problems or comments you can reach me at the email address jake1.gsmst@gmail.com
*/

#define maxPower 127


/* FUNCTION PROTOTYPES
* ALSO A LIST OF THE FUNCTIONS IN THIS CODE
*/
void setWatchdogTimerInterval(float interval);
void printWatchdogTimerInterval();
float getWatchdogTimerInterval();
bool watchdogResetOccured();

void DriveMotors(int leftPower, int rightPower);
void timeMove(int time,int power, bool brake);
void lineFollow(int startingPower, int encoder, int sensitivity);
void lineFollower(int startingPower, int time, int sensitivity);

void setupPID(float pe, float ie, float de, int integralLim, string system);
void changeKp(string system, float n);
void changeKi(string system, float n);
void changeKd(string system, float n);
void changeIntegralLimit(string system, float n);
float getKp(string system);
float getKi(string system);
float getKd(string system);
int getIntegralLimit(string system);
bool systemSetup(string system);
void eraseSystem(string system);
void eraseCustomSystem();
void changeIntakeTarget(int newTarget);
void changeIntakeSensorValue(int q);
void intakePID(bool constant);
void DrivePID(int sensorVal, int target);
void eraseIntakeSystem();
void eraseArmSystem();
void eraseDriveSystem();

void doNothing(int time);
void delayMsec(int time);
void delayTenMsec(int time);
int systemTime();
int getTime(int timer, int msecInterval);
int getTimeMsec(int timer);
int getTimeTenMsec(int timer);
void clearTime(int num);
void clearAllTimers();

void printString(string s);
void printInt(int y);
void printFloat(float y);
void printNewLine();
void printBatteryVoltageContinuous(int frequency);
void clearScreen();
void printAverageBatteryVoltage();
void printCurrentBatteryVoltage();
void printBackupBatteryVoltage();
int getBatteryVoltage();
int getBackupBatteryVoltage();

void setMotorArrayNum(int time, int motorNum, int power);
void setAllMotorsHiSpeed();
void setMotorHighSpeed(int port);
void setAllMotors(int power);
void setMotor(int time, int motorNum, int power);
int motorValueArrayNum(int portNum);
int motorValue(int num);
int motorPower(tMotor t);
void maximizeAllMotors();
void zeroAllMotors();
void printAllMotorValues();
void printMotorValueArrayNum(int motorNum);
void printMotorValue(int motorNum);
bool isMotorReversedArrayNum(int port);
bool isMotorReversed(int port);
void setMotorReversedArrayNum(bool reversed, int port);
void setMotorReversed(bool reversed, int port);
void flipMotorArrayNum(int port);
void flipMotor(int port);
void flipMotor(tMotor t);
void setMotor(tMotor t, int power);
void setMotorType(tMotor t, int type);
void setMotorTypeHighSpeed(tMotor t);
bool isMotorReversed(tMotor t);

void setupPotentiometerCalculations(int ground, bool reverse);
void changePotentiometerReversed();
void changePotentiometerGround(int new);
int getPotentiometerGround();
int calculatePotentiometerValues(int angle);
int potSelector(int regions, int potValue);
bool exceedsLimits(tSensors t);
bool closeToExceedingLimits(tSensors t, int sensitivity);
void setExceedingLimitSensitivity(int sensitivity);
bool limitSensitivitySet();
int getExceedingLimitSensitivity();
bool closeToLimits(tSensors t);
bool closeToLowLimit(tSensors t, int sensitivity);
bool closeLowLimit(tSensors t);
bool closeToHighLimit(tSensors t, int sensitivity);
bool closeHighLimit(tSensors t);
void setPotentiometerLowLimit(int lim);
void setPotentiometerHighLimit(int lim);
task monitorForPotentiometerDanger();

void setupEncoder(int revPerRot, int wheelDiameter, string name);
void printCurrentEncoderValues();
bool isRightEncoder(string name);
int getRevPerPot();
int getWheelDiameter();
bool isEncoderSetup();
int encoderValues(int desiredDistance);

void setupGyro(int port, int bias);
void setupGyroPort(int port);
int getGyroValue(int port);
bool checkSetup(int port);
void rotate(int degrees, int bias, int port, int leftPower, int rightPower);

void setupTouch(int port);
int returnValue(int port);
void waitForPress();
void waitForPressPort(int port);
bool isTouchPressedPort(int port);
bool isTouchPressed();

void clearLCD();
void LCDClearLine(int num);
void LCDClearLineOne();
void LCDClearLineTwo();
void LCDLineOne(string a);
void LCDLineTwo(string a);
void setPosition(int line, int pos);
void displayFileLineOne();
void displayFileLineTwo();
void LCDdisplayStringHere(string x);
void LCDdisplayNumberHere(int x);
bool stringFitOnLine(string x);
bool lcdButtonPressed();
int  LCDButtonPressed();
bool LCDLeftPressed();
bool LCDCenterPressed();
bool LCDRightPressed();
bool LCDBacklightStatus();
void turnLCDBacklightOff();
void turnLCDBacklightOn();

/* SPECIAL FUNCTIONS */

// Triangle Functions
void setAngleOne(float a);
bool angleTwoDefined();
void setAngleTwo(float a);
bool angleOneDefined();
void setSideOne(float a);
bool sideOneDefined();
void setSideTwo(float a);
bool sideTwoDefined();
void setSideHypotunuse(float a);
bool hypotunuseDefined();
void solveSidesWithTwo();
bool twoSidesDefinedCheck();
void solveAnglesWithTwo();
bool triangleSolveable();
bool triangleComplete();
void printTriangleResults();
float getSideOne();
float getSideTwo();
float getHypotunuse();
float getAngleOne();
float getAngleTwo();

// Circle/ARC  Functions
void driveCircle(); // MAY REQUIRE YOU TO USE THE PID LOOPS IF SO DO THE ONE BELOW WILL REQUIRE
void setRadius(float radius);
void setDiameter(float diameter);
void setCircumference(float circumference);
void solveCircle();
float getArcLength(float degrees);
float getRadius();
float getDiameter();
float getArea();
bool diameterDefined();
bool radiusDefined();
bool circumferenceDefined();
bool areaDefined();
bool circleSolveable();
void printCircle();



/* CHANGE TO THE USER CONTROL WORLD */
void setupChannelDeadzone(int channelNumber, int deadZoneLimit); // Partner marked by 5,6,7,8
void setupUniversalDeadzone(int deadzoneLimit);
void setupDriveScaling(int scaleFactor);
int getDriveScaleFactor();
int getDeadzoneLimit();
int getDeadzoneLimitChannel(int channelNumber);
int getChannelOneDedzone();
int getChannelOnePartnerDeadzone();
int getChannelTwoDeadzone();
int getChannelTwoPartnerDeadzone();
int getChannelThreeDeadzone();
int getChannelThreePartnerDeadzone();
int getChannelFourDeadzone();
int getChannelFourPartnerDeadzone();
bool channelOneDeadzoneActive();
bool channelTwoDeadzoneActive();
bool channelThreeDeadzoneActive();
bool channelFourDeadzoneActive();
bool channelOnePartnerDeadzoneActive();
bool channelTwoPartnerDeadzoneActive();
bool channelThreePartnerDeadzoneActive();
bool channelFourPartnerDeadzoneActive();

/* Channels and Buttons */
int channelOneValue();
int channelTwoValue();
int channelThreeValue();
int channelFourValue();
int button8U();
int button8D();
int button8L();
int button8R();
int button7U();
int button7D();
int button7L();
int button7R();
int button6U();
int button6D();
int button5U();
int button5D();
int xAcceler();
int yAcceler();
int zAcceler();
bool button8UPressed();
bool button8DPressed();
bool button8LPressed();
bool button8RPressed();
bool button7UPressed();
bool button7DPressed();
bool button7LPressed();
bool button7RPressed();
bool button6UPressed();
bool button6DPressed();
bool button5UPressed();
bool button5DPressed();

// PARTNER JOYSTICK STUFF
bool partnerJoystickActive();
int partnerChannelOneValue();
int partnerChannelTwoValue();
int partnerChannelThreeValue();
int partnerChannelFourValue();
int partnerButton8U();
int partnerButton8D();
int partnerButton8L();
int partnerButton8R();
int partnerButton7U();
int partnerButton7D();
int partnerButton7L();
int partnerButton7R();
int partnerButton6U();
int partnerButton6D();
int partnerButton5U();
int partnerButton5D();
int partnerXAccel();
int partnerYAccel();
int partnerZAccel();
bool partnerButton8UPressed();
bool partnerButton8DPressed();
bool partnerButton8LPressed();
bool partnerButton8RPressed();
bool partnerButton7UPressed();
bool partnerButton7DPressed();
bool partnerButton7LPressed();
bool partnerButton7RPressed();
bool partnerButton6UPressed();
bool partnerButton6DPressed();
bool partnerButton5UPressed();
bool partnerButton5DPressed();

bool anyControlsPressed();
bool anyMainPressed();
bool anyPartnerPressed();

task LCDMotorFlipScreen();

void runMechanum(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack);
void runStandardTank(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack);
void runStandardArcade(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack);
void runSixMotorTank(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack);
void runSixMotorArcade(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack);

void runMechanumHalf(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack);
void runStandardTankHalf(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack);
void runStandardArcadeHalf(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack);
void runSixMotorTankHalf(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack);
void runSixMotorArcadeHalf(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack);

void runMechanumMotor(int LF, int RF, int LB, int RB);
void runStandardTankMotor(int LF, int RF, int LB, int RB);
void runStandardArcadeMotor(int LF, int RF, int LB, int RB);
void runSixMotorTankMotor(int LF, int LM, int LB, int RF, int RM, int RB );
void runSixMotorArcadeMotor(int LF, int LM, int LB, int RF, int RM, int RB);

void runMechanumMotorHalf(int LF, int RF, int LB, int RB);
void runStandardTankMotorHalf(int LF, int RF, int LB, int RB);
void runStandardArcadeMotorHalf(int LF, int RF, int LB, int RB);
void runSixMotorTankMotorHalf(int LF, int LM, int LB, int RF, int RM, int RB );
void runSixMotorArcadeMotorHalf(int LF, int LM, int LB, int RF, int RM, int RB);

bool halfPowerEnabled();
int returnHalfPowerCount();
void checkHalfPowerButton(int button);

/*
SCORE KEEPERS
*/

//Vex Sack Attack

// Vex Tossup
void redAutonomousBonus();
void blueAutonomousBonus();
bool autonomousWinner();
void addBBMiddleRed();
void addBBMiddleBlue();
void addBBGoalRed();
void addBBGoalBlue();
void addBBCylinderRed();
void addBBCylinderBlue();
void addLBMiddleRed();
void addLBMiddleBlue();
void addLBGoalRed();
void addLBGoalBlue();
void addLBCylinderRed();
void addLBCylinderBlue();
void addRedLowHang();
void addBlueLowHang();
void addRedHighHang();
void addBlueHighHang();
void addRedHangingBall();
void addBlueHangingBall();

void setRedHanging(int low, int high);
void setBlueHanging(int low, int high);
void setHanging(int redLow, int redHigh, int blueLow, int blueHigh);
void setHangingBalls(int red, int blue);

void setRedMiddle(int BB, int LB);
void setBlueMiddle(int BB, int LB);
void setMiddle(int redBB, int blueBB, int redLB, int blueLB);
int getRedMiddleScore();
int getBlueMiddleScore();
int redMiddleBBs();
int redMiddleLBs();
int blueMiddleBBs();
int blueMiddleLBs();

void setRedGoal(int BB, int LB);
void setBlueGoal(int BB, int LB);
void setGoal(int redBB, int redLB, int blueBB, int blueLB);
int redGoalBBs();
int redGoalLBs();
int redGoalScore();
int blueGoalBBs();
int blueGoalLBs();
int blueGoalScore();

void setRedCylinder(int BB, int LB);
void setBlueCylinder(int BB, int LB);
void setCylinders(int redBB, int redLB, int blueBB, int blueLB);
int redCylinderBBs();
int redCylinderLBs();
int redCylinderPoints();
int blueCylinderBBs();
int blueCylinderLBs();
int blueCylinderPoints();

int calculateArmTorque(int motorTorque, int motorNumber, int gearRatio, int armLength)
{
	return (motorTorque * motorNumber * gearRatio) / armLength;
}

int calculateArmRPMs(int motorSpeed, int gearRatio)
{
	return motorSpeed / gearRatio;
}

void setWatchdogTimerInterval(float interval)
{
	nWatchdogTimeoutInterval = interval;
}

void printWatchdogTimerInterval()
{
	printFloat(nWatchdogTimeoutInterval);
}

float getWatchdogTimerInterval()
{
	return nWatchdogTimeoutInterval;
}

bool watchdogResetOccured()
{
	return bResetFromWatchdogTimeout;
}



void DriveMotors(int leftPower, int rightPower)
{
	motor[port2] = leftPower;
	motor[port10] = leftPower;
	motor[port1] = rightPower;
	motor[port9] = rightPower;
}

void lineFollow(int startingPower, int encoder, int sensitivity)
{
	int rightPower = startingPower;
	int leftPower = startingPower;
	while(SensorValue(dgtl1)  < encoder)
	{
		int powerAdd = (SensorValue(in4) / sensitivity) - (SensorValue(in6) / sensitivity);
		rightPower = rightPower + powerAdd;
		leftPower = leftPower - powerAdd;
		DriveMotors(leftPower, rightPower);
		wait1Msec(5);
	}
	DriveMotors(0,0);
}

void lineFollower(int startingPower, int time, int sensitivity)
{
	int startTime = nSysTime;
	int rightPower = startingPower;
	int leftPower = startingPower;

	while(nSysTime < (startTime + time))
	{
		int powerAdd = (SensorValue(in4) / sensitivity) - (SensorValue(in6) / sensitivity);
		rightPower = rightPower + powerAdd;
		leftPower = leftPower - powerAdd;
		DriveMotors(leftPower, rightPower);
		wait1Msec(5);
	}
	DriveMotors(0,0);
}

void timeMove(int time,int power, bool brake)
{
	while(time1[T4] < time)
		DriveMotors(power,power);
	if(brake)
	{
		int y = sgn(power);
		if(y == 1)
		{
			DriveMotors(-37,-37);
			wait1Msec(300);
		}
		else if(y == -1)
		{
			DriveMotors(37,37);
			wait1Msec(300);
		}
	}
}

void doNothing(int time)
{
	int startTime = nSysTime;
	while(nSysTime < (startTime + time))
		noOp();
}


/*
*
*
PID FUNCTIONS
*
*
*/

typedef struct
{
	float kp;
	float ki;
	float kd;
	float integral;
	float derivative;
	float previousError;
	float error;
	float target;
	int integralLimit;
	int sensorVal;
	bool setup;
}PIDValues;

PIDValues driveSystem;
PIDValues armSystem;
PIDValues IntakeSystem;
PIDValues customSystem;

void setupPID(float pe, float ie, float de, int integralLim, string system)
{
	if(strcmp(system, "Drive") == 0)
	{
		driveSystem.kp = pe;
		driveSystem.ki = ie;
		driveSystem.kd = de;
		driveSystem.integralLimit = integralLim;
		driveSystem.setup = true;
	}

	else if(strcmp(system, "Intake") == 0)
	{
		IntakeSystem.kp = pe;
		IntakeSystem.ki = ie;
		IntakeSystem.kd = de;
		IntakeSystem.integralLimit = integralLim;
		IntakeSystem.setup = true;
	}

	else if(strcmp(system, "Arm") == 0)
	{
		armSystem.kp = pe;
		armSystem.ki = ie;
		armSystem.kd = de;
		armSystem.integralLimit = integralLim;
		armSystem.setup = true;
	}
	else
	{
		customSystem.kp = pe;
		customSystem.ki = ie;
		customSystem.kd = de;
		customSystem.integralLimit = integralLim;
		customSystem.setup = true;
	}
}

void changeIntakeTarget(int newTarget)
{
	IntakeSystem.target = newTarget;
}

void changeIntakeSensorValue(int q)
{
	IntakeSystem.sensorVal = q;
}

void intakePID(bool constant)
{
	if(constant)
	{
		while(true)
		{
			IntakeSystem.error = IntakeSystem.target - IntakeSystem.sensorVal;
			IntakeSystem.integral = IntakeSystem.integral + IntakeSystem.error;
			IntakeSystem.derivative = IntakeSystem.error - IntakeSystem.previousError;

			if(IntakeSystem.integral > IntakeSystem.integralLimit)
				IntakeSystem.integral = 0;

			int power = floor(IntakeSystem.error * IntakeSystem.kp + IntakeSystem.integral * IntakeSystem.ki + IntakeSystem.derivative * IntakeSystem.kd);
			DriveMotors(power, power);
			IntakeSystem.previousError = IntakeSystem.error;
		}
	}

	else if(!constant)
	{
		string error = "ERROR This function only works on a constant basis";
		printString(error);
	}
}

void DrivePID(int sensorVal, int target)
{
	driveSystem.previousError = 0;

	while(sensorVal < target)
	{
		driveSystem.error = target - sensorVal;
		driveSystem.integral = driveSystem.integral + driveSystem.error;
		driveSystem.derivative = driveSystem.error - driveSystem.previousError;

		if(driveSystem.integral > driveSystem.integralLimit)
			driveSystem.integral = 0;

		int power = floor(driveSystem.error * driveSystem.kp + driveSystem.integral * driveSystem.ki + driveSystem.derivative * driveSystem.kd);
		DriveMotors(power, power);

		driveSystem.previousError = driveSystem.error;
	}

}


void changeKp(string system, float n)
{
	if(strcmp(system, "Drive") == 0)
		driveSystem.kp = n;
	else if(strcmp(system, "Intake") == 0)
		IntakeSystem.kp = n;
	else if(strcmp(system, "Arm") == 0)
		armSystem.kp = n;
	else
		customSystem.kp = n;

}

void changeKi(string system, float n)
{
	if(strcmp(system, "Drive") == 0)
		driveSystem.ki = n;
	else if(strcmp(system, "Intake") == 0)
		IntakeSystem.ki = n;
	else if(strcmp(system, "Arm") == 0)
		armSystem.ki = n;
	else
		customSystem.ki = n;
}

void changeKd(string system, float n)
{
	if(strcmp(system, "Drive") == 0)
		driveSystem.kd = n;
	else if(strcmp(system, "Intake") == 0)
		IntakeSystem.kd = n;
	else if(strcmp(system, "Arm") == 0)
		armSystem.kd = n;
	else
		customSystem.kd = n;
}

void changeIntegralLimit(string system, float n)
{
	if(strcmp(system, "Drive") == 0)
		driveSystem.integralLimit = n;
	else if(strcmp(system, "Intake") == 0)
		IntakeSystem.integralLimit = n;
	else if(strcmp(system, "Arm") == 0)
		armSystem.integralLimit = n;
	else
		customSystem.integralLimit = n;
}

float getKp(string system)
{
	if(strcmp(system, "Drive") == 0)
		return driveSystem.kp;
	else if(strcmp(system, "Intake") == 0)
		return IntakeSystem.kp;
	else if(strcmp(system, "Arm") == 0)
		return armSystem.kp;
	else
		return customSystem.kp;
}

float getKi(string system)
{
	if(strcmp(system, "Drive") == 0)
		return driveSystem.ki;
	else if(strcmp(system, "Intake") == 0)
		return IntakeSystem.ki;
	else if(strcmp(system, "Arm") == 0)
		return armSystem.ki;
	else
		return customSystem.ki;
}

float getKd(string system)
{
	if(strcmp(system, "Drive") == 0)
		return driveSystem.kd;
	else if(strcmp(system, "Intake") == 0)
		return IntakeSystem.kd;
	else if(strcmp(system, "Arm") == 0)
		return armSystem.kd;
	else
		return customSystem.kd;
}

int getIntegralLimit(string system)
{
	if(strcmp(system, "Drive") == 0)
		return driveSystem.integralLimit;
	else if(strcmp(system, "Intake") == 0)
		return IntakeSystem.integralLimit;
	else if(strcmp(system, "Arm") == 0)
		return armSystem.integralLimit;
	else
		return customSystem.integralLimit;
}

bool systemSetup(string system)
{
	if(strcmp(system, "Drive") == 0)
		return driveSystem.setup;
	else if(strcmp(system, "Intake") == 0)
		return IntakeSystem.setup
	else if(strcmp(system, "Arm") == 0)
		return armSystem.setup;
	else
		return customSystem.setup;
}

void eraseSystem(string system)
{
	if(strcmp(system, "Drive") == 0)
		eraseDriveSystem();
	else if(strcmp(system, "Intake") == 0)
		eraseIntakeSystem();
	else if(strcmp(system, "Arm") == 0)
		eraseArmSystem();
	else
		eraseCustomSystem();
}

void eraseIntakeSystem()
{
	IntakeSystem.kp = 0;
	IntakeSystem.ki = 0;
	IntakeSystem.kd = 0;
	IntakeSystem.integral = 0;
	IntakeSystem.derivative = 0;
	IntakeSystem.previousError = 0;
	IntakeSystem.error = 0;
	IntakeSystem.target = 0;
	IntakeSystem.integralLimit = 0;
	IntakeSystem.sensorVal = 0;
	IntakeSystem.setup = false;
}

void eraseArmSystem()
{
	armSystem.kp = 0;
	armSystem.ki = 0;
	armSystem.kd = 0;
	armSystem.integral = 0;
	armSystem.derivative = 0;
	armSystem.previousError = 0;
	armSystem.error = 0;
	armSystem.target = 0;
	armSystem.integralLimit = 0;
	armSystem.sensorVal = 0;
	armSystem.setup = false;
}

void eraseDriveSystem()
{
	driveSystem.kp = 0;
	driveSystem.ki = 0;
	driveSystem.kd = 0;
	driveSystem.integral = 0;
	driveSystem.derivative = 0;
	driveSystem.previousError = 0;
	driveSystem.error = 0;
	driveSystem.target = 0;
	driveSystem.integralLimit = 0;
	driveSystem.sensorVal = 0;
	driveSystem.setup = false;
}

void eraseCustomSystem()
{
	customSystem.kp = 0;
	customSystem.ki = 0;
	customSystem.kd = 0;
	customSystem.integral = 0;
	customSystem.derivative = 0;
	customSystem.previousError = 0;
	customSystem.error = 0;
	customSystem.target = 0;
	customSystem.integralLimit = 0;
	customSystem.sensorVal = 0;
	customSystem.setup = false;
}



/*
*
*
* TIME FUNCTIONS
*
*
*/

void delayMsec(int time)
{
	wait1Msec(time);
}

void delayTenMsec(int time)
{
	wait10Msec(time);
}

int systemTime()
{
	return nSysTime;
}

int getTime(int timer, int msecInterval)
{
	if(timer == 1)
		return ceil(time1(T1) / msecInterval);
	else if(timer == 2)
		return ceil(time1(T2) / msecInterval);
	else if(timer == 3)
		return ceil(time1(T3) / msecInterval);
	else if(timer == 4)
		return ceil(time1(T4) / msecInterval);
	else
	{
		string error = "ERROR getTime: The timer you have sent does not exist";
		printString(error);
		printNewLine();
		string u = "You are attempting to call Timer #";
		printString(u);
		printInt(timer);
		printNewLine();
		string a = "You can only call timers numbered 1,2,3, and 4";
		printString(a);
		return -999999999;
	}
}

int getTimeMsec(int timer)
{
	return getTime(timer, 1);
}

int getTimeTenMsec(int timer)
{
	return getTime(timer, 10);
}

void clearTime(int num)
{
	num--;
	switch(num)
	{
	case 0:
		ClearTimer(T1);
		break;

	case 1:
		ClearTimer(T2);
		break;

	case 2:
		ClearTimer(T3);
		break;

	case 3:
		ClearTimer(T4);
		break;
	}
}
void clearAllTimers()
{
	ClearTimer(T1);
	ClearTimer(T2);
	ClearTimer(T3);
	ClearTimer(T4);
}

/*
*
*
*
*DEBUGGING FUNCTIONS
*
*
*/

void printString(string s)
{
	writeDebugStream(s);
}

void printInt(int y)
{
	writeDebugStream("%d", y);
}

void printFloat(float y)
{
	writeDebugStream("%f", y);
}

void printNewLine()
{
	writeDebugStream("\n");
}

void printBatteryVoltageContinuous(int frequency)
{
	if(time1[T1] % frequency > 5)
	{
		writeDebugStreamLine("%d", nAvgBatteryLevel);
	}
}

void clearScreen()
{
	clearDebugStream();
}

void printAverageBatteryVoltage()
{
	printInt(nAvgBatteryLevel);
}

void printCurrentBatteryVoltage()
{
	printInt(nImmediateBatteryLevel);
}

void printBackupBatteryVoltage()
{
	printInt(BackupBatteryLevel);
}

int getBatteryVoltage()
{
	return nAvgBatteryLevel;
}
int getBackupBatteryVoltage()
{
	return BackupBatteryLevel;
}

// MOTORVALUES

void setMotorArrayNum(int time, int motorNum, int power)
{
	int startTime = nSysTime;
	while(nSysTime < (startTime + time))
		motor[motorNum] = power;
}

void setAllMotorsHiSpeed()
{
	for(int i = 0; i < 10; i++)
		motorType[i] = 8;
}

void setMotorHighSpeed(int port)
{
	motorType[port - 1] = 8;
}

void setAllMotors(int power)
{
	for(int i = 0; i < 10; i++)
		motor[i] = power;
}

void setMotor(int time, int motorNum, int power)
{
	int startTime = nSysTime;
	while(nSysTime < (time + startTime))
		motor[motorNum - 1] = power;
}

int motorValueArrayNum(int portNum)
{
	return motor[portNum];
}

int motorValue(int num)
{
	return motor[num - 1];
}

void maximizeAllMotors()
{
	for(int i = 0; i < 10; i++)
		motor[i] = maxPower;
}

void zeroAllMotors()
{
	for(int i = 0; i < 10; i++)
		motor[i] = 0;
}

void printAllMotorValues()
{
	for(int i = 0; i < 10; i++)
		printInt(motor[i]);
}

void printMotorValueArrayNum(int motorNum)
{
	printInt(motor[motorNum]);
}

void printMotorValue(int motorNum)
{
	printInt(motor[motorNum - 1]);
}

bool isMotorReversedArrayNum(int port)
{
	return bMotorReflected[port];
}

bool isMotorReversed(int port)
{
	return bMotorReflected[port - 1];
}

void setMotorReversedArrayNum(bool reversed, int port)
{
	bMotorReflected[port] = reversed;
}

void setMotorReversed(bool reversed, int port)
{
	bMotorReflected[port - 1] = reversed;
}

void flipMotorArrayNum(int port)
{
	if(isMotorReversedArrayNum(port))
		setMotorReversedArrayNum(false, port);
	else
		setMotorReversedArrayNum(true,port);
}

int motorPower(tMotor t)
{
	return motor[t];
}

void flipMotor(int port)
{
	if(isMotorReversed(port))
		setMotorReversed(false, port);
	else
		setMotorReversed(true, port);
}

void flipMotor(tMotor t)
{
	if(bMotorReflected[t])
		bMotorReflected[t] = false;
	else
		bMotorReflected[t] = true;
}

void setMotor(tMotor t, int power)
{
	motor[t] = power;
}

void setMotorType(tMotor t, int type)
{
	motorType[t] = type;
}

void setMotorTypeHighSpeed(tMotor t)
{
	motorType[t] = 8;
}

bool isMotorReversed(tMotor t)
{
	return bMotorReflected[t];
}



















/*
*
*
*Sensor Math Functions
*
*
*
*/

int potentiometerGround;
bool reversedPotentiometer;
#define degreeRate 4095 / 270

void setupPotentiometerCalculations(int ground, bool reverse)
{
	potentiometerGround = ground;
	reversedPotentiometer = reverse;
}

void changePotentiometerReversed()
{
	if(reversedPotentiometer)
		reversedPotentiometer = false;
	else if(!reversedPotentiometer)
		reversedPotentiometer = true;
}

void changePotentiometerGround(int new)
{
	potentiometerGround = new;
}

int getPotentiometerGround()
{
	return potentiometerGround;
}

int calculatePotentiometerValues(int angle)
{
	return ((degreeRate * angle) + potentiometerGround);
}

int potSelector(int regions, int potValue)
{
	int workingInt = 0;
	int regionRate = 4095 / regions;

	for(int i = 0; i < regions; i++)
	{
		int prevMax = i * regionRate;
		if(potValue > prevMax && potValue <= (prevMax + regionRate))
			workingInt = i + 1;
	}

	if(workingInt > regions)
	{
		string error = "ERROR REGION: Somehow went positive";
		printString(error);
		return -99999999;
	}

	else if(sgn(workingInt) == -1)
	{
		string errorTwo = "ERROR REGION: Somehow went (-)";
		printString(errorTwo);
		return -99999999;
	}

	else if(sgn(workingInt) == 0)
	{
		string errorThree = "ERROR REGION: Equal to Zero";
		printString(errorThree);
		return -99999999;
	}

	else
		return workingInt;
}
int low;
int high;
int limitSensitivity;
bool limitSensitivitySetup;
bool exceedsLimits(tSensors t)
{
	if(SensorValue(t) > high || SensorValue(t) < low)
		return true;
	else
		return false;
}

bool closeToExceedingLimits(tSensors t, int sensitivity)
{
	if((SensorValue(t) + sensitivity) > high || (SensorValue(t) - sensitivity) < low)
		return true;
	else
		return false;
}

void setExceedingLimitSensitivity(int sensitivity)
{
	limitSensitivity = sensitivity;
	limitSensitivitySetup = true;
}
bool limitSensitivitySet()
{
	return limitSensitivitySetup;
}

int getExceedingLimitSensitivity()
{
	return limitSensitivity;
}

bool closeToLimits(tSensors t)
{
	if(closeHighLimit(t) || closeLowLimit(t))
		return true;
	else
		return false;
}

bool closeToLowLimit(tSensors t, int sensitivity)
{
	if(SensorValue(t) - sensitivity < low)
		return true;
	else
		return false;
}

bool closeLowLimit(tSensors t)
{
	if(SensorValue(t) - limitSensitivity < high)
		return true;
	else
		return false;
}

bool closeToHighLimit(tSensors t, int sensitivity)
{
	if(SensorValue(t) + sensitivity > high)
		return true;
	else
		return false;
}

bool closeHighLimit(tSensors t)
{
	if(SensorValue(t) + limitSensitivity > high)
		return true;
	else
		return false;
}

void setPotentiometerLowLimit(int lim)
{
	low = lim;
}

void setPotentiometerHighLimit(int lim)
{
	high = lim;
}










// ENCODER VALUES

int encoderValues(int wheelDiameter, int desiredDistance, int revPerRot)
{
	float radius = (wheelDiameter / 2);
	float circumference = PI * (radius * radius);
	float wheelRotations = desiredDistance / circumference;
	return wheelRotations * revPerRot;
}

int definedRevPerRot;
int definedDiameter;
bool encoderSetup;
string encoderName;

void setupEncoder(int revPerRot, int wheelDiameter, string name)
{
	definedDiameter = wheelDiameter;
	definedRevPerRot = revPerRot;
	encoderName = name;
	encoderSetup = true;
}

void printCurrentEncoderValues()
{
	printString(encoderName);
	printInt(definedDiameter);
	printInt(definedRevPerRot);

	if(encoderSetup)
	{
		string trueG = "encoderSetup is True";
		printString(trueG);
	}

	else if(!encoderSetup)
	{
		string falseG = "encoderSetup is False";
		printString(falseG);
	}
}

bool isRightEncoder(string name)
{
	if(strcmp(encoderName,name) == 0)
		return true;
	else
		return false;
}

int getRevPerPot()
{
	return definedRevPerRot;
}

int getWheelDiameter()
{
	return definedDiameter;
}

bool isEncoderSetup()
{
	return encoderSetup;
}

int encoderValues(int desiredDistance)
{
	if(encoderSetup)
		return encoderValues(definedDiameter, desiredDistance, definedRevPerRot);
	else
	{
		string y = "Failed to setup encoder";
		printString(y);
		return 0;
	}
}

/*
*
*
Gyro Sensor
*
*
*/
int gyroBias = -999;

bool oneSetup;
bool twoSetup;
bool threeSetup;
bool fourSetup;
bool fiveSetup;
bool sixSetup;
bool sevenSetup;
bool eightSetup;

void setupGyro(int port, int bias)
{
	setupGyroPort(port);
	gyroBias = bias;
}

void setupGyroPort(int port)
{
	if(port == 1){SensorType[in1] = sensorNone;SensorType[in1] = sensorGyro;wait10Msec(5);oneSetup = true;}
	if(port == 2){SensorType[in2] = sensorNone;SensorType[in2] = sensorGyro;wait10Msec(5);twoSetup = true;}
	if(port == 3){SensorType[in3] = sensorNone;SensorType[in3] = sensorGyro;wait10Msec(5);threeSetup = true;}
	if(port == 4){SensorType[in4] = sensorNone;SensorType[in4] = sensorGyro;wait10Msec(5);fourSetup = true;}
	if(port == 5){SensorType[in5] = sensorNone;SensorType[in5] = sensorGyro;wait10Msec(5);fiveSetup = true;}
	if(port == 6){SensorType[in6] = sensorNone;SensorType[in6] = sensorGyro;wait10Msec(5);sixSetup = true;}
	if(port == 7){SensorType[in7] = sensorNone;SensorType[in7] = sensorGyro;wait10Msec(5);sevenSetup = true;}
	if(port == 8){SensorType[in8] = sensorNone;SensorType[in8] = sensorGyro;wait10Msec(5);eightSetup = true;}
}

int getGyroValue(int port)
{
	switch(port)
	{
	case 1:
		return SensorValue(in1);
		break;

	case 2:
		return SensorValue(in2);
		break;

	case 3:
		return SensorValue(in3);
		break;

	case 4:
		return SensorValue(in4);
		break;

	case 5:
		return SensorValue(in5);
		break;

	case 6:
		return SensorValue(in6);
		break;

	case 7:
		return SensorValue(in7);
		break;

	case 8:
		return SensorValue(in8);
		break;

	default:
		return -999999999;
		break;
	}
}

bool checkSetup(int port)
{
	if(port == 1)
		return oneSetup;
	else if(port == 2)
		return twoSetup;
	else if(port == 3)
		return threeSetup;
	else if(port == 4)
		return fourSetup;
	else if(port == 5)
		return fiveSetup;
	else if(port == 6)
		return sixSetup;
	else if(port == 7)
		return sevenSetup;
	else if(port == 8)
		return eightSetup;
	else
	{
		if(port < 1 || port > 8)
		{
			string error = "ERROR GYRO SETUP CHECK : YOUR PORT CANNOT EXIST";
			printString(error);
		}
		else
		{
			string er = "You have broken the Boolean!";
			printString(er);
		}
		return false;
	}
}

void rotate(int degrees, int bias, int port, int leftPower, int rightPower)
{
	if(!checkSetup(port))
	{
		string error = "Port not setup";
		printString(error);
	}
	else if(gyroBias != -999)
		bias = gyroBias;

	int gyroValue = getGyroValue(port);

	while(abs(gyroValue - bias) > (degrees))
	{
		DriveMotors(leftPower, rightPower);
		wait1Msec(5);
	}
}


/*
*
*
* Touch Sensor
*
*
*/

bool touchSetup;
int portSetup;

int returnValue(int port)
{
	switch(port)
	{
	case 1:
		return SensorValue(dgtl1);
		break;
	case 2:
		return SensorValue(dgtl2);
		break;
	case 3:
		return SensorValue(dgtl3);
		break;
	case 4:
		return SensorValue(dgtl4);
		break;
	case 5:
		return SensorValue(dgtl5);
		break;
	case 6:
		return SensorValue(dgtl6);
		break;
	case 7:
		return SensorValue(dgtl7);
		break;
	case 8:
		return SensorValue(dgtl8);
		break;
	case 9:
		return SensorValue(dgtl9);
		break;
	case 10:
		return SensorValue(dgtl10);
		break;
	case 11:
		return SensorValue(dgtl11);
		break;
	case 12:
		return SensorValue(dgtl12);
		break;
	}

	return -99999;
}

void setupTouch(int port)
{
	if(port == 1)
	{
		SensorType(dgtl1) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 2)
	{
		SensorType(dgtl2) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 3)
	{
		SensorType(dgtl3) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 4)
	{
		SensorType(dgtl4) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 5)
	{
		SensorType(dgtl5) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 6)
	{
		SensorType(dgtl6) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 7)
	{
		SensorType(dgtl7) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 8)
	{
		SensorType(dgtl8) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 9)
	{
		SensorType(dgtl9)  = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 10)
	{
		SensorType(dgtl10)  = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 11)
	{
		SensorType(dgtl11) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}

	if(port == 12)
	{
		SensorType(dgtl12) = sensorTouch;
		portSetup = port;
		touchSetup = true;
	}
}

#define touchDetectSamplingRate 3
void waitForPress()
{
	if(!touchSetup)
	{
		string error = "ERROR: NO TOUCH SETUP";
		printString(error);
	}

	while (returnValue(portSetup) != 1)
	{
		doNothing(touchDetectSamplingRate);
	}
}

void waitForPressPort(int port)
{
	setupTouch(port);
	waitForPress();
}

bool isTouchPressedPort(int port)
{
	setupTouch(port);
	if(returnValue(portSetup) == 1)
		return true;
	else
		return false;
}

bool isTouchPressed()
{
	if(returnValue(portSetup) == 1)
		return true;
	else
		return false;
}

/*
*
*
* LCD SCREEN
*
*
*/

void clearLCD()
{
	clearLCDLine(0);
	clearLCDLine(1);
}

void LCDClearLine(int num)
{
	if(num < 0)
	{
		string err = "LCD CLEAR ERROR: Number too Low";
		clearLCDLine(0);
	}
	else if(num > 1)
	{
		string error = "LCD CLEAR ERROR: Number too Large";
		clearLCDLine(1);
	}

	if(num == 0)
		clearLCDLine(0);
	if(num == 1)
		clearLCDLine(1);
}

void LCDClearLineOne()
{
	clearLCDLine(0);
}

void LCDClearLineTwo()
{
	clearLCDLine(1);
}

void LCDLineOne(string a)
{
	displayLCDCenteredString(0, a);
}

void LCDLineTwo(string a)
{
	displayLCDCenteredString(1, a);
}

void setPosition(int line, int pos)
{
	setLCDPosition(line, pos);
}

void LCDdisplayStringHere(string x)
{
	displayNextLCDString(x);
}

void LCDdisplayNumberHere(int x)
{
	displayNextLCDNumber(x);
}
void displayFileLineOne()
{}

void displayFileLineTwo()
{}

bool stringFitOnLine(string x)
{
	int xe = strlen(x);

	if(xe > 16)
		return false;
	else
		return true;
}

bool lcdButtonPressed()
{
	if(nLCDButtons != 0)
		return true;
	else
		return false;
}

int  LCDButtonPressed()
{
	if(lcdButtonPressed())
		return nLCDButtons;
	else
		return -999999999;
}

bool LCDLeftPressed()
{
	if(nLCDButtons == 1)
		return true;
	else
		return false;
}

bool LCDCenterPressed()
{
	if(nLCDButtons == 2)
		return true;
	else
		return false;
}

bool LCDRightPressed()
{
	if(nLCDButtons == 4)
		return true;
	else
		return false;
}

bool defaultLCD = true;
bool LCDBacklightStatus()
{
	return defaultLCD;
}

void turnLCDBacklightOff()
{
	bLCDBacklight = false;
	defaultLCD = false;
}

void turnLCDBacklightOn()
{
	bLCDBacklight = true;
	defaultLCD = true;
}
/*
*
*
Special Functions
*
*
*/
///| angle two
/// | this side is side two
///  |
//----
// angle one, side one

typedef struct
{
	float angleOne;
	bool angleOneDefined;
	float sideOne;
	bool sideOneDefined;
	float angleTwo;
	bool angleTwoDefined;
	float sideTwo;
	bool sideTwoDefined;
	float hypo;
	bool HypotunuseDefined;
}TriangleVariables;

TriangleVariables triangle;

void setAngleOne(float a)
{
	triangle.angleOne = a;
	triangle.angleOneDefined = true;
}

bool angleOneDefined()
{
	return triangle.angleOneDefined;
}

void setAngleTwo(float a)
{
	triangle.angleTwo = a;
	triangle.angleTwoDefined = true;
}

bool angleTwoDefined()
{
	return triangle.angleTwoDefined;
}

void setSideOne(float a)
{
	triangle.sideOne = a;
	triangle.sideOneDefined = true;
}

bool sideOneDefined()
{
	return triangle.sideOneDefined;
}

void setSideTwo(float a)
{
	triangle.sideTwo = a;
	triangle.sideTwoDefined = true;
}

bool sideTwoDefined()
{
	return triangle.sideTwoDefined;
}

void setSideHypotunuse(float a)
{
	triangle.hypo = a;
	triangle.HypotunuseDefined = true;
}

bool hypotunuseDefined()
{
	if(triangle.HypotunuseDefined)
		return true;
	else
		return false;
}

bool triangleSolveable()
{
	bool isSideDefined = (triangle.sideOneDefined || triangle.sideTwoDefined || triangle.HypotunuseDefined);
	bool isAngleDefined = (triangle.angleOneDefined || triangle.angleTwoDefined);
	bool twoSidesDefined = 	twoSidesDefinedCheck();
	if((isSideDefined && isAngleDefined) || twoSidesDefined)
		return true;
	else
		return false;
}

float tan(float angle)
{
	return sinDegrees(angle) / cosDegrees(angle);
}

void solveSidesWithTwo()
{
	if(triangle.sideOneDefined && triangle.sideTwoDefined)
	{
		triangle.hypo = sqrt(triangle.sideOne * triangle.sideOne + triangle.sideTwo * triangle.sideTwo);
		triangle.HypotunuseDefined = true;
	}
	else if(triangle.sideTwoDefined && triangle.HypotunuseDefined)
	{
		triangle.sideOne = sqrt(pow(triangle.hypo,2) - pow(triangle.sideTwo,2));
		triangle.sideOneDefined = true;
	}
	else if(triangle.sideOneDefined && triangle.HypotunuseDefined)
	{
		triangle.sideTwo = sqrt(pow(triangle.hypo,2) - pow(triangle.sideOne,2));
		triangle.sideTwoDefined = true;
	}
	else
	{
		string x = "ERROR TWO SIDES NOT DEFINED";
		printString(x);
	}
}
void solveAnglesWithTwo()
{
	if(triangle.angleOneDefined)
	{
		triangle.angleTwo = 90 - triangle.angleOne;
		triangle.angleTwoDefined = true;
	}

	else if(triangle.angleTwoDefined)
	{
		triangle.angleOne = 90 - triangle.angleTwo;
		triangle.angleOneDefined = true;
	}
}

bool twoSidesDefinedCheck()
{
	return ((triangle.sideOneDefined && triangle.sideTwoDefined) || (triangle.sideOneDefined && triangle.HypotunuseDefined) || (triangle.sideTwoDefined && triangle.HypotunuseDefined));
}

#define t triangle

bool triangleComplete()
{
	if(t.sideOneDefined && t.sideTwoDefined && t.angleOneDefined && t.angleTwoDefined && t.hypotunuseDefined)
		return true;
	else
		return false;
}

void triangleSolver()
{
	/* Too lazy to type triangle a gazillion times*/

	/* Given angle one, side one */
	if(t.angleOneDefined && t.sideOneDefined)
	{
		float num = tan(t.angleOne);
		t.sideTwo = num * t.sideOne; // tan = (opposite / adj) opposite = tan * adj
		solveSidesWithTwo();
		solveAnglesWithTwo();
	}

	else if(t.angleOneDefined && t.sideTwoDefined)
	{
		t.hypo = t.sideTwo / sinDegrees(t.angleOne);
		t.HypotunuseDefined = true;
		solveAnglesWithTwo();
		solveSidesWithTwo();
	}

	else if(t.angleOneDefined && t.HypotunuseDefined)
	{
		t.sideOne = cosDegrees(t.angleOne) / t.hypo;
		solveAnglesWithTwo();
		solveSidesWithTwo();
	}

	else if(t.angleTwoDefined && t.HypotunuseDefined)
	{
		t.sideTwo = cosDegrees(t.angleTwo) / t.hypo;
		solveAnglesWithTwo();
		solveSidesWithTwo();
	}

	else if(t.angleTwoDefined && t.sideTwoDefined)
	{
		t.sideOne = radiansToDegrees(tan(t.angleTwo)) * t.sideTwoDefined;
		solveAnglesWithTwo();
		solveSidesWithTwo();
	}

	else if(t.angleTwoDefined && t.sideOneDefined)
	{
		t.sideTwo = t.sideOne / radiansToDegrees(tan(t.angleTwo));
		solveAnglesWithTwo();
		solveSidesWithTwo();
	}

	else if(twoSidesDefinedCheck())
	{
		solveSidesWithTwo();
		t.angleOne = radiansToDegrees(asin(t.sideOne / t.hypo));
		solveAnglesWithTwo();
	}

	if(!triangleComplete())
		triangleSolver();
}


void printTriangleResults()
{
	string one = "Side One:";

	printString(one);
	printFloat(t.sideOne);
	printNewLine();

	one = "Side Two:";
	printString(one);
	printFloat(t.sideTwo);
	printNewLine();

	one = "Angle One:";
	printString(one);
	printFloat(t.angleOne);
	printNewLine();

	one = "Angle Two:";
	printString(one);
	printFloat(t.angleTwo);
	printNewLine();

	one = "Hypotunuse:";
	printString(one);
	printFloat(t.hypo);
}

float getSideOne()
{
	return t.sideOne;
}

float getSideTwo()
{
	return t.sideTwo;
}

float getHypotunuse()
{
	return t.hypo;
}

float getAngleOne()
{
	return t.angleOne;
}

float getAngleTwo()
{
	return t.angleTwo;
}




void driveCircle(){} // MAY REQUIRE YOU TO USE THE PID LOOPS IF SO DO THE ONE BELOW WILL REQUIRE
void setRadius(float radius){}
void setDiameter(float diameter){}
void setCircumference(float circumference){}
void solveCircle(){}
float getArcLength(float degrees){}
float getRadius(){}
float getDiameter(){}
float getArea(){}
bool diameterDefined(){}
bool radiusDefined(){}
bool circumferenceDefined(){}
bool areaDefined(){}
bool circleSolveable(){}
void printCircle(){}

/* USER CONTROL */


int channelOneValue()
{
	return vexRT[Ch1];
}

int channelTwoValue()
{
	return vexRT[Ch2];
}

int channelThreeValue()
{
	return vexRT[Ch3];
}

int channelFourValue()
{
	return vexRT[Ch4];
}

int button8U()
{
	return vexRT[Btn8U];
}

int button8D()
{
	return vexRT[Btn8D];
}

int button8L()
{
	return vexRT[Btn8L];
}

int button8R()
{
	return vexRT[Btn8R];
}

int button7U()
{
	return vexRT[Btn7U];
}

int button7D()
{
	return vexRT[Btn7D];
}

int button7L()
{
	return vexRT[Btn7L];
}

int button7R()
{
	return vexRT[Btn7R];
}

int button6U()
{
	return vexRT[Btn6U];
}

int button6D()
{
	return vexRT[Btn6D];
}

int button5U()
{
	return vexRT[Btn5U];
}

int button5D()
{
	return vexRT[Btn5D];
}

bool button8UPressed()
{
	if(button8U() == 1)
		return true;
	else
		return false;
}

bool button8DPressed()
{
	if(button8D() == 1)
		return true;
	else
		return false;
}

bool button8LPressed()
{
	if(button8L() == 1)
		return true;
	else
		return false;
}

bool button8RPressed()
{
	if(button8R() == 1)
		return true;
	else
		return false;
}

bool button7UPressed()
{
	if(button7U() == 1)
		return true;
	else
		return false;
}

bool button7DPressed()
{
	if(button7D() == 1)
		return true;
	else
		return false;
}

bool button7LPressed()
{
	if(button8L() == 1)
		return true;
	else
		return false;
}

bool button7RPressed()
{
	if(button7R() == 1)
		return true;
	else
		return false;
}

bool button6UPressed()
{
	if(button6U() == 1)
		return true;
	else
		return false;
}

bool button6DPressed()
{
	if(button6D() == 1)
		return true;
	else
		return false;
}

bool button5UPressed()
{
	if(button5U() == 1)
		return true;
	else
		return false;
}

bool button5DPressed()
{
	if(button5D() == 1)
		return true;
	else
		return false;
}

int xAcceler()
{
	return vexRT[AccelX];
}

int yAcceler()
{
	return vexRT[AccelY];
}

int zAcceler()
{
	return vexRT[AccelZ];
}







typedef struct
{
	int dead;
	int scale;
	bool active;
}Deadzone;

Deadzone universal;
Deadzone partnerOne;
Deadzone partnerTwo;
Deadzone partnerThree;
Deadzone partnerFour;
Deadzone mainOne;
Deadzone mainTwo;
Deadzone mainThree;
Deadzone mainFour;

void setupChannelDeadzone(int channelNumber, int deadZoneLimit)
{
	switch(channelNumber)
	{
	case 1:
		mainOne.dead = deadZoneLimit;
		mainOne.active = true;
		break;

	case 2:
		mainTwo.dead = deadZoneLimit;
		mainTwo.active = true;
		break;

	case 3:
		mainThree.dead = deadZoneLimit;
		mainThree.active = true;
		break;

	case 4:
		mainFour.dead = deadZoneLimit;
		mainFour.active = true;
		break;

	case 5:
		partnerOne.dead = deadZoneLimit;
		partnerOne.active = true;
		break;

	case 6:
		partnerTwo.dead = deadZoneLimit;
		partnerTwo.active = true;
		break;

	case 7:
		partnerThree.dead = deadZoneLimit;
		partnerThree.active = true;
		break;

	case 8:
		partnerFour.dead = deadZoneLimit;
		partnerFour.active = true;
		break;

	default:
		universal.dead = deadZoneLimit;
		universal.active = true;
	}
}

void setupUniversalDeadzone(int deadzoneLimit)
{
	universal.dead = deadzoneLimit;
	universal.active = true;
}

void setupDriveScaling(int scaleFactor)
{
	universal.scale = scaleFactor;
}

int getDriveScaleFactor()
{
	return universal.scale;
}

int getDeadzoneLimit()
{
	return universal.dead;
}

int getDeadzoneLimitChannel(int channelNumber)
{
	if(channelNumber == 1)
		return mainOne.dead;
	else if(channelNumber == 2)
		return mainTwo.dead;
	else if(channelNumber == 3)
		return mainThree.dead;
	else if(channelNumber == 4)
		return mainFour.dead;
	else if(channelNumber == 5)
		return partnerOne.dead;
	else if(channelNumber == 6)
		return partnerTwo.dead;
	else if(channelNumber == 7)
		return partnerThree.dead;
	else if(channelNumber == 8)
		return partnerFour.dead;
	else
		return universal.dead;
}

int getChannelOneDedzone()
{
	return mainOne.dead;
}

int getChannelOnePartnerDeadzone()
{
	return partnerOne.dead;
}

int getChannelTwoDeadzone()
{
	return mainTwo.dead;
}

int getChannelTwoPartnerDeadzone()
{
	return partnerTwo.dead;
}

int getChannelThreeDeadzone()
{
	return mainThree.dead;
}

int getChannelThreePartnerDeadzone()
{
	return partnerThree.dead;
}

int getChannelFourPartnerDeadzone()
{
	return partnerFour.dead;
}

int getChannelFourDeadzone()
{
	return mainFour.dead;
}

bool channelOneDeadzoneActive()
{
	return mainOne.active;
}

bool channelTwoDeadzoneActive()
{
	return mainTwo.active;
}

bool channelThreeDeadzoneActive()
{
	return mainThree.active;
}

bool channelFourDeadzoneActive()
{
	return mainFour.active;
}

bool channelOnePartnerDeadzoneActive()
{
	return partnerOne.active;
}

bool channelTwoPartnerDeadzoneActive()
{
	return partnerTwo.active;
}

bool channelThreePartnerDeadzoneActive()
{
	return partnerThree.active;
}

bool channelFourPartnerDeadzoneActive()
{
	return partnerFour.active;
}

bool partnerJoystickActive();

int partnerChannelOneValue()
{
	return  vexRT[Ch1Xmtr2];
}

int partnerChannelTwoValue()
{
	return vexRT[Ch2Xmtr2];
}

int partnerChannelThreeValue()
{
	return vexRT[Ch3Xmtr2];
}

int partnerChannelFourValue()
{
	return vexRT[Ch4Xmtr2];
}

int partnerButton8U()
{
	return vexRT[Btn8UXmtr2];
}

int partnerButton8D()
{
	return vexRT[Btn8DXmtr2];
}

int partnerButton8L()
{
	return vexRT[Btn8LXmtr2];
}

int partnerButton8R()
{
	return vexRT[Btn8RXmtr2];
}

int partnerButton7U()
{
	return vexRT[Btn7UXmtr2];
}

int partnerButton7D()
{
	return vexRT[Btn7DXmtr2];
}

int partnerButton7L()
{
	return vexRT[Btn7LXmtr2];
}

int partnerButton7R()
{
	return vexRT[Btn7RXmtr2];
}

int partnerButton6U()
{
	return vexRT[Btn6UXmtr2];
}

int partnerButton6D()
{
	return vexRT[Btn6DXmtr2];
}

int partnerButton5U()
{
	return vexRT[Btn5UXmtr2];
}

int partnerButton5D()
{
	return vexRT[Btn5DXmtr2];
}

int partnerXAccel()
{
	return vexRT[AccelXXmtr2];
}

int partnerYAccel()
{
	return vexRT[AccelYXmtr2];
}

int partnerZAccel()
{
	return vexRT[AccelZXmtr2];
}

bool partnerButton8UPressed()
{
	if(partnerButton8U() == 1)
		return true;
	else
		return false;
}

bool partnerButton8DPressed()
{
	if(partnerButton8D() == 1)
		return true;
	else
		return false;
}

bool partnerButton8LPressed()
{
	if(partnerButton8L() == 1)
		return true;
	else
		return false;
}

bool partnerButton8RPressed()
{
	if(partnerButton8R() == 1)
		return true;
	else
		return false;
}

bool partnerButton7UPressed()
{
	if(partnerButton7U() == 1)
		return true;
	else
		return false;
}

bool partnerButton7DPressed()
{
	if(partnerButton7D() == 1)
		return true;
	else
		return false;
}

bool partnerButton7LPressed()
{
	if(partnerButton7L() == 1)
		return true;
	else
		return false;
}

bool partnerButton7RPressed()
{
	if(partnerButton7R() == 1)
		return true;
	else
		return false;
}

bool partnerButton6UPressed()
{
	if(partnerButton6U() == 1)
		return true;
	else
		return false;
}

bool partnerButton6DPressed()
{
	if(partnerButton6D() == 1)
		return true;
	else
		return false;
}

bool partnerButton5UPressed()
{
	if(partnerButton5U() == 1)
		return true;
	else
		return false;
}

bool partnerButton5DPressed()
{
	if(partnerButton5D() == 1)
		return true;
	else
		return false;
}

bool anyControlsPressed()
{
	if(anyMainPressed() || anyPartnerPressed())
		return true;
	else
		return false;
}

bool anyMainPressed()
{
	bool fiveButtons = (button5DPressed() || button5UPressed());
	bool sixButtons = (button6UPressed() || button6DPressed());
	bool sevenButtons = (button7UPressed() || button7DPressed() || button7LPressed() || button7RPressed());
	bool eightButtons = (button8UPressed() || button8DPressed() || button8LPressed() || button8RPressed());

	if(fiveButtons || sixButtons || sevenButtons || eightButtons)
		return true;
	else
		return false;
}

bool anyPartnerPressed()
{
	bool fivePartnerButtons = (partnerButton5DPressed() || partnerButton5UPressed());
	bool sixPartnerButtons = (partnerButton6UPressed() || partnerButton6DPressed());
	bool sevenPartnerButtons = (partnerButton7UPressed() || partnerButton7DPressed() || partnerButton7LPressed() || partnerButton7RPressed());
	bool eightPartnerButtons = (partnerButton8UPressed() || partnerButton8DPressed() || partnerButton8LPressed() || partnerButton8RPressed());

	if(fivePartnerButtons || sixPartnerButtons || sevenPartnerButtons || eightPartnerButtons)
		return true;
	else
		return false;
}


task LCDMotorFlipScreen()
{
	int CurrentMotor = 1;
	clearLCD();
	while(true)
	{
		string a = "Motor:";
		LCDdisplayStringHere(a);
		LCDdisplayNumberHere(CurrentMotor);
		setLCDPosition(1,0);

		if(LCDCenterPressed())
			flipMotor(CurrentMotor);

		if(LCDLeftPressed())
		{
			if(CurrentMotor != 1)
				CurrentMotor--;
			else
				CurrentMotor = 10;
		}

		if(LCDRightPressed())
		{
			if(CurrentMotor != 10)
				CurrentMotor++;
			else
				CurrentMotor = 1;
		}
		wait10Msec(10);
	}
}



bool halfPower;
int halfPowerCount;

void runMechanum(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack)
{
	while(true)
	{

		motor[rightFront] = vexRT[Ch3] - vexRT[Ch1] - vexRT[Ch4];
		motor[rightBack] =  vexRT[Ch3] - vexRT[Ch1] + vexRT[Ch4];
		motor[leftFront] = vexRT[Ch3] + vexRT[Ch1] + vexRT[Ch4];
		motor[leftBack] =  vexRT[Ch3] + vexRT[Ch1] - vexRT[Ch4];
	}
}

void runStandardTank(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack)
{
	while(true)
	{
		motor[leftFront] = vexRT[Ch2];
		motor[leftBack] =  vexRT[Ch2];
		motor[rightBack] =  vexRT[Ch3];
		motor[rightFront] = vexRT[Ch3];
	}
}

void runStandardArcade(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack)
{
	while(true)
	{
		motor[leftBack] = channelThreeValue() + channelOneValue();
		motor[leftFront] = channelThreeValue() + channelOneValue();
		motor[rightFront] = channelThreeValue() - channelOneValue();
		motor[rightBack] = channelThreeValue() - channelOneValue();
	}
}

void runSixMotorTank(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack)
{
	while(true)
	{
		motor[leftFront] = vexRT[Ch2];
		motor[leftMid] = vexRT[Ch2];
		motor[leftBack] =  vexRT[Ch2];
		motor[rightFront] = vexRT[Ch3];
		motor[rightMid] = vexRT[Ch3];
		motor[rightBack] = vexRT[Ch3];
	}
}

void runSixMotorArcade(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack)
{
	while(true)
	{

		motor[leftFront] = channelThreeValue() + channelOneValue();
		motor[leftMid] = channelThreeValue() + channelOneValue();
		motor[leftBack] = channelThreeValue() + channelOneValue();
		motor[rightFront] = channelThreeValue() - channelOneValue();
		motor[rightMid] = channelThreeValue() - channelOneValue();
		motor[rightBack] = channelThreeValue() - channelOneValue();
	}
}




void runMechanumHalf(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack)
{
	while(true)
	{
		if(halfPowerEnabled())
		{
			motor[rightFront] = vexRT[Ch3] - vexRT[Ch1] - vexRT[Ch4] /2;
			motor[rightBack] =  vexRT[Ch3] - vexRT[Ch1] + vexRT[Ch4]/2;
			motor[leftFront] = vexRT[Ch3] + vexRT[Ch1] + vexRT[Ch4]/2;
			motor[leftBack] =  vexRT[Ch3] + vexRT[Ch1] - vexRT[Ch4]/2;
		}
		else
		{
			motor[rightFront] = vexRT[Ch3] - vexRT[Ch1] - vexRT[Ch4];
			motor[rightBack] =  vexRT[Ch3] - vexRT[Ch1] + vexRT[Ch4];
			motor[leftFront] = vexRT[Ch3] + vexRT[Ch1] + vexRT[Ch4];
			motor[leftBack] =  vexRT[Ch3] + vexRT[Ch1] - vexRT[Ch4];
		}
	}
}

void runStandardTankHalf(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack)
{
	while(true)
	{
	motor[leftFront] = halfPowerEnabled() ? vexRT[Ch2] / 2 : vexRT[Ch2];
	motor[leftBack] = halfPowerEnabled() ? vexRT[Ch2] / 2 : vexRT[Ch2];
	motor[rightBack] = halfPowerEnabled() ? vexRT[Ch3] / 2 : vexRT[Ch3];
	motor[rightFront] = halfPowerEnabled() ? vexRT[Ch3] / 2 : vexRT[Ch3];
	}
}

void runStandardArcadeHalf(tMotor leftFront, tMotor rightFront, tMotor leftBack, tMotor rightBack)
{
	while(true)
	{
		if(halfPowerEnabled())
		{
			motor[leftBack] = channelThreeValue() + channelOneValue()/2;
			motor[leftFront] = channelThreeValue() + channelOneValue()/2;
			motor[rightFront] = channelThreeValue() - channelOneValue()/2;
			motor[rightBack] = channelThreeValue() - channelOneValue()/2;
		}
		else
		{
			motor[leftBack] = channelThreeValue() + channelOneValue();
			motor[leftFront] = channelThreeValue() + channelOneValue();
			motor[rightFront] = channelThreeValue() - channelOneValue();
			motor[rightBack] = channelThreeValue() - channelOneValue();
		}
	}
}

void runSixMotorTankHalf(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack)
{
	while(true)
	{
	motor[leftFront] = halfPowerEnabled() ? vexRT[Ch2] / 2 : vexRT[Ch2];
	motor[leftMid] = halfPowerEnabled() ? vexRT[Ch2] / 2 : vexRT[Ch2];
	motor[leftBack] = halfPowerEnabled() ? vexRT[Ch2] / 2 : vexRT[Ch2];
	motor[rightFront] = halfPowerEnabled() ? vexRT[Ch3] / 2 : vexRT[Ch3];
	motor[rightMid] = halfPowerEnabled() ? vexRT[Ch3] / 2 : vexRT[Ch3];
	motor[rightBack] = halfPowerEnabled() ? vexRT[Ch3] / 2 : vexRT[Ch3];
	}
}

void runSixMotorArcadeHalf(tMotor leftFront, tMotor leftMid, tMotor leftBack, tMotor rightFront, tMotor rightMid, tMotor rightBack)
{
	while(true)
	{
		if(halfPowerEnabled())
		{
			motor[leftFront] = channelThreeValue() + channelOneValue() / 2;
			motor[leftMid] = channelThreeValue() + channelOneValue() / 2;
			motor[leftBack] = channelThreeValue() + channelOneValue() / 2;
			motor[rightFront] = channelThreeValue() - channelOneValue() / 2;
			motor[rightMid] = channelThreeValue() - channelOneValue() / 2;
			motor[rightBack] = channelThreeValue() - channelOneValue() / 2;
		}
		else
		{
			motor[leftFront] = channelThreeValue() + channelOneValue();
			motor[leftMid] = channelThreeValue() + channelOneValue();
			motor[leftBack] = channelThreeValue() + channelOneValue();
			motor[rightFront] = channelThreeValue() - channelOneValue();
			motor[rightMid] = channelThreeValue() - channelOneValue();
			motor[rightBack] = channelThreeValue() - channelOneValue();
		}
	}
}


void runMechanumMotor(int LF, int RF, int LB, int RB)
{
	runMechanum(motor[LF], motor[LB], motor[RF], motor[RB]);
}

void runStandardTankMotor(int LF, int RF, int LB, int RB)
{
	runStandardTank(motor[LF], motor[LB], motor[RF], motor[RB]);
}

void runStandardArcadeMotor(int LF, int RF, int LB, int RB)
{
	runStandardArcade(motor[LF], motor[LB], motor[RF], motor[RB]);
}

void runSixMotorTankMotor(int LF, int LM, int LB, int RF, int RM, int RB )
{
	runSixMotorTank(motor[LF], motor[LM], motor[LB], motor[RF], motor[RM], motor[RB]);
}

void runSixMotorArcadeMotor(int LF, int LM, int LB, int RF, int RM, int RB)
{
	runSixMotorArcade(motor[LF], motor[LM], motor[LB], motor[RF], motor[RM], motor[RB]);
}


void runMechanumMotorHalf(int LF, int RF, int LB, int RB)
{
	runMechanumHalf(motor[LF], motor[LB], motor[RF], motor[RB]);
}

void runStandardTankMotorHalf(int LF, int RF, int LB, int RB)
{
	runStandardTankHalf(motor[LF], motor[LB], motor[RF], motor[RB]);
}

void runStandardArcadeMotorHalf(int LF, int RF, int LB, int RB)
{
	runStandardArcadeHalf(motor[LF], motor[LB], motor[RF], motor[RB]);
}

void runSixMotorTankMotorHalf(int LF, int LM, int LB, int RF, int RM, int RB )
{
	runSixMotorTankHalf(motor[LF], motor[LM], motor[LB], motor[RF], motor[RM], motor[RB]);
}

void runSixMotorArcadeMotorHalf(int LF, int LM, int LB, int RF, int RM, int RB)
{
	runSixMotorArcadeHalf(motor[LF], motor[LM], motor[LB], motor[RF], motor[RM], motor[RB]);
}

bool halfPowerEnabled()
{
	if(halfPowerCount % 2 == 1)
	{
		return true;
		halfPower = true;
	}
	else
	{
		return false;
		halfPower = false;
	}
}

int returnHalfPowerCount()
{
	return halfPowerCount;
}

void checkHalfPowerButton(int button)
{
	while(true)
	{
		if(button == 1)
			halfPowerCount++;
	}
}

bool redAuton;
bool blueAuton;
int redMidBB;
int redGoalBB;
int redCylBB;
int redMidLB;
int redGoalLB;
int redCylLB;
int blueMidBB;
int blueGoalBB;
int blueCylBB;
int blueMidLB;
int blueGoalLB;
int blueCylLB;
int blueLowHang;
int blueHighHang;
int blueBallHang;
int redLowHang;
int redHighHang;
int redBallHang;

void redAutonomousBonus()
{
	redAuton = true;
}

void blueAutonomousBonus()
{
	blueAuton = true;
}

bool autonomousWinner()
{
	if(blueAuton || redAuton)
		return true;
	else
		return false;

}

void addBBMiddleRed()
{
	redMidBB++;
}

void addBBMiddleBlue()
{
	blueMidBB++;
}

void addBBGoalRed()
{
	redGoalBB++;
}

void addBBGoalBlue()
{
	blueGoalBB++;
}

void addBBCylinderRed()
{
	redCylBB++;
}

void addBBCylinderBlue()
{
	blueCylBB++;
}

void addLBMiddleRed()
{
	redMidLB++;
}

void addLBMiddleBlue()
{
	blueMidLB++;
}

void addLBGoalRed()
{
	redGoalLB++;
}

void addLBGoalBlue()
{
	blueGoalLB++;
}

void addLBCylinderRed()
{
	redCylLB++;
}

void addLBCylinderBlue()
{
	blueCylLB++;
}

void addRedLowHang()
{
	redLowHang++;
}

void addBlueLowHang()
{
	blueLowHang++;
}

void addRedHighHang()
{
	redHighHang++;
}

void addBlueHighHang()
{
	blueHighHang++;
}

void addRedHangingBall()
{
	redBallHang++;
}

void addBlueHangingBall()
{
	blueBallHang++;
}

void setRedHanging(int low, int high)
{
	redLowHang = low;
	redHighHang = high;
}

void setBlueHanging(int low, int high)
{
	blueLowHang = low;
	blueHighHang = high;
}

void setHanging(int redLow, int redHigh, int blueLow, int blueHigh)
{
	redLowHang = redLow;
	redHighHang = redHigh;
	blueLowHang = blueLow;
	blueHighHang = blueHigh;
}

void setHangingBalls(int red, int blue)
{
	redBallHang = red;
	blueBallHang = blue;
}

void setRedMiddle(int BB, int LB)
{
	redMidBB = BB;
	redMidLB = LB;
}

void setBlueMiddle(int BB, int LB)
{
	blueMidBB = BB;
	blueMidLB = LB;
}

void setMiddle(int redBB, int blueBB, int redLB, int blueLB)
{
	redMidBB = redBB;
	redMidLB = redLB;
	blueMidBB = blueBB;
	blueMidLB = blueLB;
}

int getRedMiddleScore()
{
	return redMidBB + redMidLB;
}

int getBlueMiddleScore()
{
	return blueMidBB + blueMidLB;
}

int redMiddleBBs()
{
	return redMidBB;
}

int redMiddleLBs()
{
	return redMidLB;
}

int blueMiddleBBs()
{
	return blueMidBB;
}

int blueMiddleLBs()
{
	return blueMidLB;
}

void setRedGoal(int BB, int LB)
{
	redGoalBB = BB;
	redGoalLB = LB;
}

void setBlueGoal(int BB, int LB)
{
	blueGoalBB = BB;
	blueGoalLB = LB;
}

void setGoal(int redBB, int redLB, int blueBB, int blueLB)
{
	redGoalBB = redBB;
	redGoalLB = redLB;
	blueGoalBB = blueBB;
	blueGoalLB = blueLB;
}

int redGoalBBs()
{
	return redGoalBB;
}

int redGoalLBs()
{
	return redGoalLB;
}

int redGoalScore()
{
	return (2 * redGoalBB) + (5 * redGoalLB);
}

int blueGoalBBs()
{
	return blueGoalBB;
}

int blueGoalLBs()
{
	return blueGoalLB;
}

int blueGoalScore()
{
	return (2 * blueGoalBB) + (5 * blueGoalLB);
}

void setRedCylinder(int BB, int LB)
{
	redCylBB = BB;
	redCylLB = LB;
}

void setBlueCylinder(int BB, int LB)
{
	 blueCylBB = BB;
	 blueCylLB = LB;
}

void setCylinders(int redBB, int redLB, int blueBB, int blueLB)
{
	redCylBB = redBB;
	redCylLB = redLB;
	 blueCylBB = blueBB;
	 blueCylLB = blueLB;
}

int redCylinderBBs()
{
	return redCylBB;
}

int redCylinderLBs()
{
	return redCylLB;
}

int redCylinderPoints()
{
	return (redCylBB * 5) + (redCylLB * 10);
}

int blueCylinderBBs()
{
	return blueCylBB;
}

int blueCylinderLBs()
{
	return blueCylLB;
}

int blueCylinderPoints()
{
	return (blueCylBB * 5) + (blueCylLB * 10);
}
