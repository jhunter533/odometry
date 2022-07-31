#include "PID.h"

#include "GlobalVar.h"
#include "vex.h"
#include"Conversion.h"


int turnCount = 0;
int angleTracker = 0;
double turnToTolerance = .5;
int maxIter = 700;

PID_ turnPID;
PID_ drivePID;
PID_ strafePID;
/*

void driveTo() {
  drivePID.iBound = 9;
  drivePID.threshold = 0.5;

  drivePID.kI = 0.002;

  drivePID.kD = 0.001;

  drivePID.kP = 0.035;

  drivePID.error = yTargetLocation - yPosGlobal;

  drivePID.derivative = drivePID.error - drivePID.prevError;

  drivePID.prevError = drivePID.error;

  if(fabs(drivePID.error) < drivePID.iBound && drivePID.error != 0) {
    drivePID.integral += drivePID.error;
  } else {
    drivePID.integral = 0;
  }

 
  drivePID.powerDrive =((drivePID.error * drivePID.kP) + (drivePID.derivative * drivePID.kD) + (drivePID.integral *drivePID.kI));
  if (drivePID.powerDrive >drivePID.maxSpeed) {
    drivePID.powerDrive = drivePID.maxSpeed;
  }
}
*/



void driveToNS() {
  drivePID.iBound = 9;
  drivePID.threshold = 0.5;

  drivePID.kI = 0.002;

  drivePID.kD = 0.001;

  drivePID.kP = 0.035;

  drivePID.error = sqrt((yTargetLocation - yPosGlobal)*(yTargetLocation-yPosGlobal)+(xTargetLocation-xPosGlobal)*(xTargetLocation-xPosGlobal));

  drivePID.derivative = drivePID.error - drivePID.prevError;

  drivePID.prevError = drivePID.error;

  if(fabs(drivePID.error) < drivePID.iBound && drivePID.error != 0) {
    drivePID.integral += drivePID.error;
  } else {
    drivePID.integral = 0;
  }


  drivePID.powerDrive =((drivePID.error * drivePID.kP) + (drivePID.derivative * drivePID.kD) + (drivePID.integral *drivePID.kI));
  
  if (drivePID.powerDrive > drivePID.maxSpeed) {
    drivePID.powerDrive = drivePID.maxSpeed;
  } else if (drivePID.powerDrive < -drivePID.maxSpeed) {
    drivePID.powerDrive = -drivePID.maxSpeed;
  }
}


void strafeTo() {
  strafePID.iBound = 9;
  strafePID.threshold = 0.5;

  strafePID.kI = 0.002;

  strafePID.kD = 0.001;

  strafePID.kP = 0.035;

  strafePID.error = xTargetLocation-xPosGlobal;
  strafePID.derivative = strafePID.error - strafePID.prevError;
  strafePID.prevError = strafePID.error;

  if(fabs(strafePID.error) < strafePID.iBound && strafePID.error != 0) {
    strafePID.integral += strafePID.error;
  } else {
    strafePID.integral = 0;
  }
  strafePID.powerDrive = (strafePID.error * strafePID.kP) + (strafePID.integral * strafePID.kI) + (strafePID.derivative *strafePID.kD);
}

void turnTo() {
  /*
  turnPID.iBound = .2;
  turnPID.threshold = 0.5;
  
  turnPID.kP = 0.19;

  turnPID.kI = 0.0025;

  turnPID.kD = 0.9;
  */
    turnPID.iBound = .2;
  turnPID.threshold = 0.5;
  
  turnPID.kP = 0.19;

  turnPID.kI = 0.0025;

  turnPID.kD = 0.9;

  turnPID.error = (targetAC-absoluteAngle);
  //ShortAngle();
  
  /*
  if (turnPID.error+pi/2 ==0) {
    turnPID.error = 0;
  }
  */
  turnPID.derivative = turnPID.error -turnPID.prevError;
  turnPID.prevError = turnPID.error;

  if (fabs(turnPID.error) < turnPID.iBound && turnPID.error != 0) {
    turnPID.integral += turnPID.error;
  } else {
    turnPID.integral = 0;
  }
  turnPID.powerDrive = (turnPID.error * turnPID.kP) + (turnPID.integral * turnPID.kI) + (turnPID.derivative * turnPID.kD);
  
  if (turnPID.powerDrive > turnPID.maxSpeed) {
    turnPID.powerDrive = turnPID.maxSpeed;
  } else if (turnPID.powerDrive < -turnPID.maxSpeed) {
    turnPID.powerDrive = -turnPID.maxSpeed;
  }
}