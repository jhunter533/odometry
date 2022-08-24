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
task TrackPositionTask;
task chassisTask;
motor_group LeftDriveSmart = motor_group(FrontLeft, BackLeft);
motor_group FRBLStrafe = motor_group(FrontRight,BackLeft);
motor_group FLBRStrafe = motor_group(FrontLeft,BackRight);
motor_group RightDriveSmart = motor_group(FrontRight, BackRight);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1);
//task ChassisTrackTask;
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
double maxSpeedp = 6;
// The angle difference from error when integral adjustments turns on
int turnThresholdp = 16;
// Tolerance for approximating the target angle
double turnTolerancep = .5;
// Total number of iterations to exit while loop
int maxIterp = 300;

// Turning Function
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
  angleTracker += angleTurn;
  angleTurn = angleTracker % 360;
*/
  // Automated error correction loop
  while (fabs(TurnGyroSmart.rotation(degrees) - angleTurn) > turnTolerancep && iter < maxIterp) {
    iter += 1;
    error = angleTurn - TurnGyroSmart.rotation(degrees);
    derivative = error - prevError;
    prevError = error;

    // Checking if error passes threshold to build the integral
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

    // Send to motors
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
    task TrackPositionTask(trackPosition);
    printf("xpos %.5f\n",xPosGlobal);
    printf("yPos %.5f\n",yPosGlobal);
    Right.resetRotation();
    Back.resetRotation();
    turnPTo(90);
        printf("xpos %.5f\n",xPosGlobal);
    printf("yPos %.5f\n",yPosGlobal);
    printf("turnoffsety %.5f\n", turnOffsety);
printf("turnoffsetx %.5f\n", turnOffsetx);
   
   //Drivetrain.turnFor(90,deg);
   

  
  //  task ChassisTrackTask(chassisTrack);
  
  /*
 FrontLeft.spinFor(forward, 2, rev, false);
 BackLeft.spinFor(forward,2,rev, false);
 FrontRight.spinFor(forward,2,rev, false);
 BackRight.spinFor(forward,2,rev);
 */

 /*
  FrontLeft.spinFor(forward, 2, rev, false);
 BackLeft.spinFor(forward,2,rev, false);
 FrontRight.spinFor(reverse,2,rev, false);
 BackRight.spinFor(reverse,2,rev);
 */

 //driveToP(2,8,0);
 //driveToP(8,2,0);
 /*
 while(true) {

     Brain.Screen.print("angle %.5f", absoluteAngle);
 Brain.Screen.newLine();
 Brain.Screen.print("x %.5f", xPosGlobal);
 Brain.Screen.newLine();
 Brain.Screen.print("y %.5f", yPosGlobal);
 Brain.Screen.newLine();
  Brain.Screen.print("dr %.5f", deltaR);
 Brain.Screen.newLine();
   Brain.Screen.print("ds %.5f", deltaS);
 Brain.Screen.newLine();
   Brain.Screen.print("yvec %.5f",yVector);
 Brain.Screen.newLine();
// wait(5,sec);
//SensorReadB();
//Brain.Screen.print("%.d", SensorReadB);
Brain.Screen.print("dyl %.5f", deltaYLocal);
 Brain.Screen.newLine();
 Brain.Screen.print("prevr %.5f",prevR);
 Brain.Screen.newLine();
 Brain.Screen.print("dpidPWRDR %.5f", drivePID.powerDrive);
 Brain.Screen.newLine();
   Brain.Screen.print("dyg %.5f", deltaYGlobal);
 Brain.Screen.newLine();
   Brain.Screen.print("dxl %.5f", deltaXLocal);
 Brain.Screen.newLine();
  Brain.Screen.print("Tary %.5f", yTargetLocation);
 Brain.Screen.newLine();
 printf( "xPos %.5f\n", xPosGlobal);
 printf( "yPos %.5f\n", yPosGlobal);
 printf( "dYG %.5f\n", deltaYGlobal);
 printf( "dxg %.5f\n", deltaXGlobal);
 printf( "dxL %.5f\n", deltaXLocal);
 printf( "dyl %.5f\n", deltaYLocal);
 printf( "dR %.5f\n", deltaR);
 printf( "ds %.5f\n", deltaS);
 printf( "main code \n");
 printf( "prevr %.5f\n", prevR);
 printf( "ytarget %.5f\n", yTargetLocation);
  printf( "xtarget %.5f\n", xTargetLocation);
 printf( "New Iteration00 \n");
 printf( "      \n");


wait(50,msec);
 Brain.Screen.clearScreen();
 Brain.Screen.setCursor(1,1);
 
 }



*/
   // Back.resetRotation();
  //Right.resetRotation();
 // while(true) {
    //wait(3,sec);
    //trackPosition();
    //chassisTrack();
// Controller1.Screen.print(printf("%d", chassisControl));
  //Controller1.Screen.print("enocder r %.5f", chassisControl);
 // Controller1.Screen.newLine();

  //Controller1.Screen.print("yGlobal %.5f",xTargetLocation);
  /*
  Controller1.Screen.newLine();
  Brain.Screen.print("%.5f", turnPID.powerDrive);
  Brain.Screen.newLine();
  Brain.Screen.print("%.5f", strafePID.powerDrive);
  Brain.Screen.newLine();
  Brain.Screen.print("%.5f", drivePID.powerDrive);
  wait(5,sec);
  }
  */


 // }

  // Initializing Robot Configuration. DO NOT REMOVE!
 
  
}
