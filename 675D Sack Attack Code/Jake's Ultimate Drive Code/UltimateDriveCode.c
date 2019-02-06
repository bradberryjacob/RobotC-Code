/*
* This file was written by Jacob Bradberry from the Gwinnett School of Mathematics, Science, and Technology for his 2012 - 2013 Senior Project
* Please give credit where credit is due
* If you have any problems or comments you can reach me at the email address jake1.gsmst@gmail.com
*/
#pragma config(UART_Usage, UART1, VEX_2x16_LCD, baudRate19200, IOPins, None, None)
#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, in2,    pot,            sensorPotentiometer)
#pragma config(Sensor, in3,    intakePot,      sensorPotentiometer)
#pragma config(Sensor, dgtl1,  RedLed,         sensorLEDtoVCC)
#pragma config(Sensor, dgtl2,  YellowLed,      sensorLEDtoVCC)
#pragma config(Sensor, dgtl3,  GreenLed,       sensorLEDtoVCC)
#pragma config(Sensor, I2C_1,  leftDrive,      sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  rightDrive,     sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           LBDrive,       tmotorVex393, openLoop, encoder, encoderPort, I2C_1, 1000)
#pragma config(Motor,  port2,           LFDrive,       tmotorVex393, openLoop)
#pragma config(Motor,  port3,           LTLift,        tmotorVex269, openLoop)
#pragma config(Motor,  port4,           LBLift,        tmotorVex269, openLoop)
#pragma config(Motor,  port5,           ScoopLeft,     tmotorVex269, openLoop)
#pragma config(Motor,  port6,           ScoopRight,    tmotorVex269, openLoop)
#pragma config(Motor,  port7,           RTLift,        tmotorVex269, openLoop)
#pragma config(Motor,  port8,           RBLift,        tmotorVex393, openLoop)
#pragma config(Motor,  port9,           RFDrive,       tmotorVex393, openLoop)
#pragma config(Motor,  port10,          RBDrive,       tmotorVex393, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma systemFile

#define DRIVECODE

#ifndef DEFINEFILE
#include "SackAttackCodeDefines.c"
#endif

bool xDrive;
bool uDrive;
bool halfHolonomic;
bool mechanumDrive;
bool halfPower;
bool motorRamping;
bool motorsShutDown;
bool arcadeControl;
bool partnerJoystick;
bool turnTaskRunning;
bool isRightTurn;
bool arcadeLeftStick;

int gyroTurnValue;
int gyroChoice;
int turnGoal;
int mechanumDeadZone;
int halfPowerCount = 0;

int motorPowerLeftBack();
int motorPowerLeftFront();
int motorPowerLeft();
int motorPowerRight();
int motorPowerRightFront();
int motorPowerRightBack();

void checkTurnTaskRunning();
void checkHalfPower();
void setGyroTurnValue();
void checkIsRightTurn();
void initializeEverything(bool xDrives,bool halfHolonomics,bool mechanumDrives,bool halfPowers,bool motorRampings,bool motorsShutDowns,bool arcadeControls,bool partnerJoysticks,bool turnTaskRunnings,bool isRightTurns,bool arcadeLeftSticks, int mechanumDeadZones);

task driveTask();
task autoTurnTask();

task driveTask()
{
  if(!motorsShutDown)
  {
    if(turnTaskRunning && abs(vexRT[Ch2]) > 30 && abs(vexRT[Ch3]) > 30)
    {
      StopTask(autoTurnTask);
    }
    else if(!turnTaskRunning)
    {
      if(vexRT[Btn6U] == 1)
      {
        StartTask(autoTurnTask);
        turnGoal = SensorValue(gyro) + gyroTurnValue;
      }
      setGyroTurnValue();
    motor[LBDrive] = (xDrive || mechanumDrive || halfHolonomic) ? motorPowerLeftBack() : motorPowerLeft();
    motor[LFDrive] = (xDrive || mechanumDrive || halfHolonomic) ? motorPowerLeftFront() : motorPowerLeft();
    motor[RBDrive] = (xDrive || mechanumDrive || halfHolonomic) ? motorPowerRightBack() : motorPowerRight();
    motor[RFDrive] = (xDrive || mechanumDrive || halfHolonomic) ? motorPowerRightFront() : motorPowerRight();
      checkTurnTaskRunning();
      checkHalfPower();
    }
  }
  else
  {
    motor[LBDrive] = 0;
    motor[LFDrive] = 0;
    motor[RFDrive] = 0;
    motor[RBDrive] = 0;
  }
}

void checkHalfPower()
{
  if(vexRT[Btn6D] == 1)
    halfPowerCount++;
  if(halfPowerCount % 2 == 0)
    halfPower = false;
  else if(halfPowerCount % 2 == 1)
    halfPower = true;
}
int motorPowerLeft()
{
  int tempPower = 0;
  if(!arcadeControl)
  {
    if(partnerJoystick)
    {
      if(halfPower)
      {
        if(motorRamping)
        {
          int leftStart;
          int target = PJCh3 / 2;
          for(int i = leftStart; i < target; i++)
            tempPower= i;
        }
        else if(!motorRamping)
          tempPower = PJCh3 / 2;
      }
      else if(!halfPower)
      {
        int leftStart;
        int target = PJCh3;
        for(int i = leftStart; i < target; i++)
          tempPower= i;
      }
      else if(!motorRamping)
        tempPower = PJCh3;
    }
    else if(!partnerJoystick)
    {
      if(halfPower)
      {
        if(motorRamping)
        {
          int leftStart;
          int target = MJCh3 / 2;
          for(int i = leftStart; i < target; i++)
            tempPower= i;
        }
        else if(!motorRamping)
          tempPower = MJCh3 / 2;
      }
      else if(!halfPower)
      {
        int leftStart;
        int target = MJCh3;
        for(int i = leftStart; i < target; i++)
          tempPower= i;
      }
      else if(!motorRamping)
        tempPower = MJCh3;
    }
  }
  else if(arcadeControl)
  {
    if(arcadeLeftStick)
      tempPower = (vexRT[Ch3] + vexRT[Ch4]) / 2;
    else if(!arcadeLeftStick)
      tempPower = (vexRT[Ch2] + vexRT[Ch1])/2;
  }
  return tempPower;
}

int motorPowerRight()
{
  int tempPower = 0;
  if(!arcadeControl)
  {
    if(partnerJoystick)
    {
      if(halfPower)
      {
        if(motorRamping)
        {
          int rightStart;
          int target = PJCh2 / 2;
          for(int i = rightStart; i < target; i++)
            tempPower= i;
        }
        else if(!motorRamping)
          tempPower = PJCh2 / 2;
      }
      else if(!halfPower)
      {
        int rightStart;
        int target = PJCh2;
        for(int i = rightStart; i < target; i++)
          tempPower= i;
      }
      else if(!motorRamping)
        tempPower = PJCh2;
    }
    else if(!partnerJoystick)
    {
      if(halfPower)
      {
        if(motorRamping)
        {
          int rightStart;
          int target = MJCh2 / 2;
          for(int i = rightStart; i < target; i++)
            tempPower= i;
        }
        else if(!motorRamping)
          tempPower = MJCh2 / 2;
      }
      else if(!halfPower)
      {
        int rightStart;
        int target = MJCh2;
        for(int i = rightStart; i < target; i++)
          tempPower= i;
      }
      else if(!motorRamping)
        tempPower = MJCh2;
    }
  }
  else if(arcadeControl)
  {
    if(arcadeLeftStick)
      tempPower = (vexRT[Ch3] - vexRT[Ch4]) / 2;
    else if(!arcadeLeftStick)
      tempPower = (vexRT[Ch2] - vexRT[Ch1])/2;
  }
  return tempPower;
}

int motorPowerRightFront()
{
  int tempPower = 0;
  int Y1;
  int X1;
  int X2;
  if(mechanumDrive)
  {
    if(partnerJoystick)
    {
      if(abs(vexRT[Ch3Xmtr2]) > mechanumDeadZone)
        Y1 = vexRT[Ch3Xmtr2];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4Xmtr2]) > mechanumDeadZone)
        X1 = vexRT[Ch4Xmtr2];
      else
        X1 = 0;
      if(abs(vexRT[Ch1Xmtr2]) > mechanumDeadZone)
        X2 = vexRT[Ch1Xmtr2];
      else
        X2 = 0;
    }
    else if(!partnerJoystick)
    {
      if(abs(vexRT[Ch3]) > mechanumDeadZone)
        Y1 = vexRT[Ch3];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4]) > mechanumDeadZone)
        X1 = vexRT[Ch4];
      else
        X1 = 0;
      if(abs(vexRT[Ch1]) > mechanumDeadZone)
        X2 = vexRT[Ch1];
      else
        X2 = 0;
    }
    if(motorRamping)
    {
      if(halfPower)
      {
        int rightStart;
        int target = Y1 - X2 - X1;
        for(int i = rightStart; i < target; i++)
          tempPower= i;
      }
      else
      {
        int rightFrontStart;
        int target = Y1 - X2 - X1;
        for(int i = rightFrontStart; i < target; i++)
          tempPower = i;
      }
    }
    else if(!motorRamping)
    tempPower = (halfPower) ? (Y1 - X2 - X1) / 2 : (Y1 - X2 - X1);

  }

  else if(xDrive)
  {
    if(partnerJoystick)
    tempPower = (halfPower) ? (vexRT[Ch3Xmtr2] - vexRT[Ch4Xmtr2] - vexRT[Ch1Xmtr2]) / 2 : (vexRT[Ch3Xmtr2] - vexRT[Ch4Xmtr2] - vexRT[Ch1Xmtr2]);
    else
    tempPower = (halfPower) ? (vexRT[Ch3] - vexRT[Ch4] - vexRT[Ch1]) / 2 : (vexRT[Ch3] - vexRT[Ch4] - vexRT[Ch1]);
  }

  else if(halfHolonomic)
  {
    if(partnerJoystick)
  {tempPower = (halfPower) ? vexRT[Ch2Xmtr2] / 2: vexRT[Ch2Xmtr2];}
    else if(!partnerJoystick)
  {tempPower = (halfPower) ? vexRT[Ch2] / 2: vexRT[Ch2];}
  }

  return tempPower;
}

int motorPowerRightBack()
{
  int tempPower = 0;
  int Y1;
  int X1;
  int X2;
  if(mechanumDrive)
  {
    if(partnerJoystick)
    {
      if(abs(vexRT[Ch3Xmtr2]) > mechanumDeadZone)
        Y1 = vexRT[Ch3Xmtr2];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4Xmtr2]) > mechanumDeadZone)
        X1 = vexRT[Ch4Xmtr2];
      else
        X1 = 0;
      if(abs(vexRT[Ch1Xmtr2]) > mechanumDeadZone)
        X2 = vexRT[Ch1Xmtr2];
      else
        X2 = 0;
    }
    else if(!partnerJoystick)
    {
      if(abs(vexRT[Ch3]) > mechanumDeadZone)
        Y1 = vexRT[Ch3];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4]) > mechanumDeadZone)
        X1 = vexRT[Ch4];
      else
        X1 = 0;
      if(abs(vexRT[Ch1]) > mechanumDeadZone)
        X2 = vexRT[Ch1];
      else
        X2 = 0;
    }
    if(motorRamping)
    {
      if(halfPower)
      {
        int rightStart;
        int target = Y1 - X2 - X1;
        for(int i = rightStart; i < target; i++)
          tempPower= i;
      }
      else
      {
        int rightFrontStart;
        int target = Y1 - X2 + X1;
        for(int i = rightFrontStart; i < target; i++)
          tempPower = i;
      }
    }
    else if(!motorRamping)
    tempPower = (halfPower) ? (Y1 - X2 + X1) / 2 : (Y1 - X2 + X1);

  }

  else if(xDrive)
  {
    if(partnerJoystick)
    tempPower = (halfPower) ? (vexRT[Ch3Xmtr2] - vexRT[Ch4Xmtr2] + vexRT[Ch1Xmtr2]) / 2 : (vexRT[Ch3Xmtr2] - vexRT[Ch4Xmtr2] + vexRT[Ch1Xmtr2]);
    else
    tempPower = (halfPower) ? (vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1]) / 2 : (vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1]);
  }

  else if(halfHolonomic)
  {
    if(partnerJoystick)
  {tempPower = (halfPower) ? vexRT[Ch2Xmtr2] / 2: vexRT[Ch2Xmtr2];}
    else if(!partnerJoystick)
  {tempPower = (halfPower) ? vexRT[Ch2] / 2: vexRT[Ch2];}
  }

  return tempPower;
}

int motorPowerLeftBack()
{
  int tempPower = 0;
  int Y1;
  int X1;
  int X2;
  if(mechanumDrive)
  {
    if(partnerJoystick)
    {
      if(abs(vexRT[Ch3Xmtr2]) > mechanumDeadZone)
        Y1 = vexRT[Ch3Xmtr2];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4Xmtr2]) > mechanumDeadZone)
        X1 = vexRT[Ch4Xmtr2];
      else
        X1 = 0;
      if(abs(vexRT[Ch1Xmtr2]) > mechanumDeadZone)
        X2 = vexRT[Ch1Xmtr2];
      else
        X2 = 0;
    }
    else if(!partnerJoystick)
    {
      if(abs(vexRT[Ch3]) > mechanumDeadZone)
        Y1 = vexRT[Ch3];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4]) > mechanumDeadZone)
        X1 = vexRT[Ch4];
      else
        X1 = 0;
      if(abs(vexRT[Ch1]) > mechanumDeadZone)
        X2 = vexRT[Ch1];
      else
        X2 = 0;
    }
    if(motorRamping)
    {
      if(halfPower)
      {
        int rightStart;
        int target = Y1 + X2 - X1;
        for(int i = rightStart; i < target; i++)
          tempPower= i;
      }
      else
      {
        int rightFrontStart;
        int target = Y1 + X2 - X1;
        for(int i = rightFrontStart; i < target; i++)
          tempPower = i;
      }
    }
    else if(!motorRamping)
    tempPower = (halfPower) ? (Y1 + X2 - X1) / 2 : (Y1 + X2 - X1);

  }

  else if(xDrive)
  {
    if(partnerJoystick)
    tempPower = (halfPower) ? (vexRT[Ch3Xmtr2] + vexRT[Ch4Xmtr2] - vexRT[Ch1Xmtr2]) / 2 : (vexRT[Ch3Xmtr2] + vexRT[Ch4Xmtr2] - vexRT[Ch1Xmtr2]);
    else
    tempPower = (halfPower) ? (vexRT[Ch3] + vexRT[Ch4] - vexRT[Ch1]) / 2 : (vexRT[Ch3] + vexRT[Ch4] - vexRT[Ch1]);
  }

  else if(halfHolonomic)
  {
    if(partnerJoystick)
  {tempPower = (halfPower) ? vexRT[Ch2Xmtr2] / 2: vexRT[Ch2Xmtr2];}
    else if(!partnerJoystick)
  {tempPower = (halfPower) ? vexRT[Ch2] / 2: vexRT[Ch2];}
  }

  return tempPower;
}

int motorPowerLeftFront()
{
  int tempPower = 0;
  int Y1;
  int X2;
  int X1;
  if(mechanumDrive)
  {
    if(partnerJoystick)
    {
      if(abs(vexRT[Ch3Xmtr2]) > mechanumDeadZone)
        Y1 = vexRT[Ch3Xmtr2];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4Xmtr2]) > mechanumDeadZone)
        X1 = vexRT[Ch4Xmtr2];
      else
        X1 = 0;
      if(abs(vexRT[Ch1Xmtr2]) > mechanumDeadZone)
        X2 = vexRT[Ch1Xmtr2];
      else
        X2 = 0;
    }
    else if(!partnerJoystick)
    {
      if(abs(vexRT[Ch3]) > mechanumDeadZone)
        Y1 = vexRT[Ch3];
      else
        Y1 = 0;
      if(abs(vexRT[Ch4]) > mechanumDeadZone)
        X1 = vexRT[Ch4];
      else
        X1 = 0;
      if(abs(vexRT[Ch1]) > mechanumDeadZone)
        X2 = vexRT[Ch1];
      else
        X2 = 0;
    }
    if(motorRamping)
    {
      if(halfPower)
      {
        int rightStart;
        int target = Y1 + X2 + X1;
        for(int i = rightStart; i < target; i++)
          tempPower= i;
      }
      else
      {
        int rightFrontStart;
        int target = Y1 + X2 + X1;
        for(int i = rightFrontStart; i < target; i++)
          tempPower = i;
      }
    }
    else if(!motorRamping)
    tempPower = (halfPower) ? (Y1 + X2 + X1) / 2 : (Y1 + X2 + X1);
  }
  else if(xDrive)
  {
    if(partnerJoystick)
    tempPower = (halfPower) ? (vexRT[Ch3Xmtr2] + vexRT[Ch4Xmtr2] + vexRT[Ch1Xmtr2]) / 2 : (vexRT[Ch3Xmtr2] + vexRT[Ch4Xmtr2] + vexRT[Ch1Xmtr2]);
    else
    tempPower = (halfPower) ? (vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1]) / 2 : (vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1]);
  }

  else if(halfHolonomic)
  {
    if(partnerJoystick)
    tempPower = (halfPower) ? vexRT[Ch3Xmtr2] / 2: vexRT[Ch3Xmtr2];
    else if(!partnerJoystick)
    tempPower = (halfPower) ? vexRT[Ch3] / 2: vexRT[Ch3];
  }
  return tempPower;
}

void checkIsRightTurn()
{
  if(vexRT[Btn7D] == 1 && !isRightTurn)
    isRightTurn = true;
  else if(vexRT[Btn7D] == 1)
    isRightTurn = false;
}

void checkTurnTaskRunning()
{
  if(getTaskState(autoTurnTask) == taskStateRunning)
    turnTaskRunning = true;
  else
    turnTaskRunning = false;
}

void setGyroTurnValue()
{
  checkIsRightTurn();
  if(vexRT[Btn7L] == 1)
    gyroChoice = 1;
  if(vexRT[Btn7U] == 1)
    gyroChoice = 2;
  if(vexRT[Btn7R] == 1)
    gyroChoice = 3;
  switch(gyroChoice)
  {
  case 1:
    gyroTurnValue = 900;
    break;
  case 2:
    gyroTurnValue = 1800;
    break;
  case 3:
    gyroTurnValue = 2700;
    break;
  }
}

task autoTurnTask()
{
  if(isRightTurn)
  {
    while(SensorValue(gyro) < turnGoal)
    {
      if(halfPower)
      {
        motor[LBDrive] = 127 / 2;
        motor[LFDrive] = 127 / 2;
        motor[RBDrive] = -127 / 2;
        motor[RFDrive] = -127 / 2;}
      else if(!halfPower)
      {
        motor[LBDrive] = 127;
        motor[LFDrive] = 127;
        motor[RBDrive] = -127;
        motor[RFDrive] = -127;
      }
    }
  }
  else if(!isRightTurn)
  {
    while(SensorValue(gyro) > turnGoal)
    {
      if(halfPower)
      {
        motor[LBDrive] = -127 / 2;
        motor[LFDrive] = -127 / 2;
        motor[RBDrive] = 127 / 2;
        motor[RFDrive] = 127 / 2;
      }
      else if(!halfPower)
      {
        motor[LBDrive] = -127;
        motor[LFDrive] = -127;
        motor[RBDrive] = 127;
        motor[RFDrive] = 127;
      }
    }
  }
}

void initializeEverything(bool xDrives,bool halfHolonomics,bool mechanumDrives,bool halfPowers,bool motorRampings,bool motorsShutDowns,bool arcadeControls,bool partnerJoysticks,bool turnTaskRunnings,bool isRightTurns,bool arcadeLeftSticks, int mechanumDeadZones)
{
  xDrive = xDrives;
  halfHolonomic = halfHolonomics;
  mechanumDrive = mechanumDrives;
  halfPower = halfPowers;
  motorRamping = motorRampings;
  motorsShutDown = motorsShutDowns;
  arcadeControl = arcadeControls;
  partnerJoystick = partnerJoysticks;
  turnTaskRunning = turnTaskRunnings;
  isRightTurn = isRightTurns;
  arcadeLeftStick = arcadeLeftSticks;
  mechanumDeadZone = mechanumDeadZones;
}