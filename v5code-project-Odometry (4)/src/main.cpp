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
//task ChassisTrackTask;
void test() {

}

int main() {
   vexcodeInit();
    wait(5,sec);
    task TrackPositionTask(trackPosition);
  
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

 //driveToP(6,8,0);
 driveToP(2,4,0);
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
