#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FrontLeft = motor(PORT1, ratio18_1, false);
motor BackLeft = motor(PORT11, ratio18_1, false);
motor FrontRight = motor(PORT10, ratio18_1, true);
motor BackRight = motor(PORT20, ratio18_1, true);
controller Controller2 = controller(partner);
inertial TurnGyroSmart = inertial(PORT17);
rotation Right = rotation(PORT5,true);
rotation Back = rotation(PORT13,true);

motor_group LeftDriveSmart = motor_group(BackLeft,FrontLeft);
motor_group RightDriveSmart = motor_group(BackRight,FrontRight);

smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1);
//gps GPS = gps(18, 0, 0,inches, 0);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  TurnGyroSmart.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (TurnGyroSmart.isCalibrating()) {
    wait(250, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}