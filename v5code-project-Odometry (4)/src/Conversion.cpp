#include "Conversion.h"
#include "GlobalVar.h"
#include "vex.h"
#include "PID.h"

double ConvertToDeg(double radians) {
  return (radians *180/M_PI);
}
double ConvertToRadians(double degree) {
  return (degree*M_PI/180);
}



void ShortAngle() {
  //this broke because the method to convert between compass and radian degrees
  // have it do same thing of target - 2pi 
  // target -2pi +- 360
  //which is smaller then set target to that
  //do it in radians for simplicity 
  //use targetA not faca
  double sigma = (targetFacA-absoluteAngle);
  double beta = (targetFacA-absoluteAngle)+360;
  //double alpha = (360 - targetFacA) - absoluteAngle - 360;

  if (fabs(sigma)<fabs(beta) ){
    turnPID.error = sigma;
  }
  if (fabs(beta)<fabs(sigma)) {
    turnPID.error = beta;
  }

}
