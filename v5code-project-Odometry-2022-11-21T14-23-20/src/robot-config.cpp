#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller Controller1 = controller(primary);
triport ThreeWirePort = triport(PORT22);
encoder Right = encoder(ThreeWirePort.E);
encoder Back = encoder(ThreeWirePort.A);
motor FrontLeft = motor(PORT18, ratio18_1, true);
motor BackLeft = motor(PORT10, ratio18_1, true);
motor FrontRight = motor(PORT11, ratio18_1, false);
motor BackRight = motor(PORT2, ratio18_1, false);
inertial TurnGyroSmart = inertial(PORT5);
sonar Sonic = sonar(Brain.ThreeWirePort.G);


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