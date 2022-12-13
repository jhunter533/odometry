/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\CibolaA                                          */
/*    Created:      Thu Apr 21 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "PID.h"
#include "Odometry.h"
#include "GlobalVar.h"
#include "Conversion.h"
#include "oldPID.h"
#include <iostream>
#include "Graphics.h"

using namespace vex;

//declare odometry task
task TrackPositionTask;

//declare motor groups
motor_group LeftDriveSmart = motor_group(FrontLeft, BackLeft);
motor_group FRBLStrafe = motor_group(FrontRight,BackLeft);
motor_group FLBRStrafe = motor_group(FrontLeft,BackRight);
motor_group RightDriveSmart = motor_group(FrontRight, BackRight);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1);

void test() {
  }

int turnCountp = 0;
// Relative degree tracking
int angleTrackerp = 0;
// Weighted factor of porportion error
double kP = 0.19;
// Weighted factor of integral error
double kI = 0.0025;
// Weighted factor of the derivated error
double kD = .9;
// Max speed in Volts for motors
double maxSpeedp = 12;
// The angle difference from error when integral adjustments turns on
int turnThresholdp = 16;
// Tolerance for approximating the target angle
double turnTolerancep = .5;
// Total number of iterations to exit while loop
int maxIterp = 300;

// Turning Function
// Turns to absolute heading inputed
void turnPTo(double angleTurn) {
  //  Distance to target in degrees
  double error = 0;
  //  Error degree from the last iteration
  double prevError = 0;
  // Derivative of the error. The slope between iterations
  double derivative = 0;
  // Accumulated error after threashold. Summing error
  double integral = 0;
  // Iterations of the loop. Counter used to exit loop if not converging
  double iter = 0;

  // Used for Relative Coordinates. For absolute coordinates, comment out
  // following lines for angleTracker
  /*
  //When previously enabled the robot would turn 90 to the right instead of to orientation 90
  //This meant that it would turn correctly until whatever angles that are adding equaled 0
  //Once that happened the robot would turn in a circle the opposite direction then turn to the angle going the other direction
  //While relative is easier for humans for a robot it makes it unreliable since every angle is built on the error of the previous

  angleTracker += angleTurn;
  angleTurn = angleTracker % 360;
*/
  // Automated error correction loop
  //Loop runs every 15 miliseconds
  //This function is in degrees internally and externally
  //while the error is less than thresholds and the amount of times in the loop is less than threshold
  // This is to account for when you haven't reached your target but the robot is stuck it may exit the loop
  while (fabs(TurnGyroSmart.rotation(degrees) - angleTurn) > turnTolerancep && iter < maxIterp) {
    iter += 1;
    //The error is the target angle - current angle 
    error = angleTurn - TurnGyroSmart.rotation(degrees);
    //The derivative is the error-previous error
    //These are defined as 0 before the loop runs
    derivative = error - prevError;
    //Then change the previous error value to the error before the next loop
    prevError = error;

    // Checking if error passes threshold to build the integral
    //The integral allows you to correct for overshooting so you don't want this to always run
    if (fabs(error) < turnThresholdp && error != 0) {
      integral += error;
    } else {
      integral = 0;
    }

    // Voltage to use. PID calculation
    double powerDrive = error * kP + derivative * kD + integral * kI;

    // Capping voltage to max speed
    if (powerDrive > maxSpeedp) {
      powerDrive = maxSpeedp;
    } else if (powerDrive < -maxSpeedp) {
      powerDrive = -maxSpeedp;
    }
    // Send voltage to motors
    LeftDriveSmart.spin(forward, powerDrive, voltageUnits::volt);
    RightDriveSmart.spin(forward, -powerDrive, voltageUnits::volt);

    this_thread::sleep_for(15);
  }

  // Angle achieved, brake robot
  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);

  // Tuning data, output to screen
  turnCountp += 1;
  error = angleTurn - TurnGyroSmart.rotation(degrees);
  derivative = error - prevError;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Turn #: %d", turnCountp);
  Controller1.Screen.setCursor(1, 13);
  Controller1.Screen.print("iter: %.0f", iter);
  Controller1.Screen.newLine();
  Controller1.Screen.print("error: %.5f", error);
  Controller1.Screen.newLine();
  Controller1.Screen.print("derivative: %.5f", derivative);
  Controller1.Screen.newLine();
}



int main() {
   vexcodeInit();
    wait(5,sec);
    //Sets the starting rotation at desired angle
    //TurnGyroSmart.setRotation(90, deg);
    //initiates odometry task to run in background
    task TrackPositionTask(trackPosition);

    //reset encoders just in case
    Right.resetRotation();
    Back.resetRotation();
    //print starting coordinates to console
    printf("y %.5f \n", yPosGlobal);
    printf("x %.5f \n", xPosGlobal);


  
  
}
