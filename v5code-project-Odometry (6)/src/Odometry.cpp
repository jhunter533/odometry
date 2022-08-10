#include "GlobalVar.h"
#include "Odometry.h"
#include "vex.h"
#include "Conversion.h"
#include "PID.h"
#include "oldPID.h"
#include "Graphics.h"


    double FrontLeftPower;
    double BackLeftPower;
    double FrontRightPower;
    double BackRightPower;
//double head = TurnGyroSmart.heading(deg);
//double testEnc = Right.rotation(deg);
double startAngle = M_PI/2;
//pi/2
//double startPosX=(1*360*M_PI)/(M_PI*3.25*180);
//double startPosY = (1*360*M_PI)/(M_PI*3.25*180);
double startPosX=(2*360)/(M_PI*3.25);
double startPosY = (2*360)/(M_PI*3.25);
//double startPosX=(2);
//double startPosY = (2);

double sDist = (5*360)/(M_PI*3.25);
double rDist = (6*360)/(M_PI*3.25);

//double sDist = (5);
//double rDist = (6);

double eTicks = 360;
double wheelDiameter = 3.25;

float prevR = 0;
float prevS = 0;
//double absoluteAngle=startAngle;
double absoluteAngle = ConvertToRadians(TurnGyroSmart.heading());
double prevTheta = absoluteAngle;
double totalDeltaDistR = 0;
double yPosGlobal = startPosY;
double xPosGlobal = startPosX;
double deltaTheta = 0;
double halfAngle = deltaTheta/2;
double avgTheta = absoluteAngle +halfAngle;
double deltaXGlobal = 0;
double deltaYGlobal = 0;

double deltaXLocal = 0;
double deltaYLocal = 0;
double r = 0;
double r2 = 0;

double tOX=0;
double tOY=0;
bool chassisControl= false;
double xTargetLocation = xPosGlobal;
double yTargetLocation = yPosGlobal;
double targetFacA = 0;
double targetAC =0;

double xVector = 0;
double yVector = 0;
double hAngle = 0;

double deltaR = 0;
double deltaS = 0;
//int head = TurnGyroSmart.heading(degrees);

double distToX = 0;
double distToY = 0;
double avgThetaR = ConvertToRadians(absoluteAngle);
double deltaYGlobalD = ConvertToDeg(deltaYGlobal);
double deltaXGlobalD = ConvertToDeg(deltaXGlobal);
float rEnc = Right.rotation(deg);
float sEnc = Back.rotation(deg);

int trackPosition() {
  
  
  while(1) {
//double turnOffsetx = 360*(sDist*deltaTheta*2)/wheelDiameter;
//double turnOffsety = 360*(rDist*deltaTheta*2)/wheelDiameter;
double arc=ConvertToDeg(deltaTheta)/360*(M_PI*18);
double turnOffsetx = (arc*360/(M_PI*3.25));
double turnOffsety = (arc*360/(M_PI*3.25));
 printf( "      \n");
 printf( "xPos %.5f\n", xPosGlobal);
 printf( "yPos %.5f\n", yPosGlobal);
 printf( "dYG %.5f\n", deltaYGlobal);
 printf( "dxg %.5f\n", deltaXGlobal);
 printf( "dxL %.5f\n", deltaXLocal);
 printf( "dyl %.5f\n", deltaYLocal);
    printf( "turnxoff %.5f\n", turnOffsetx);
        printf( "turnyoff %.5f\n", turnOffsety);
            printf( "rightEnc %.5f\n", Right.rotation(deg));
    printf( "backenc %.5f\n", Back.rotation(deg));
 tOX=turnOffsetx;
 tOY=turnOffsety;
     printf( "tox %.5f\n", tOX);
    printf( "toy %.5f\n", tOY);
 //prev used ones in deg
 deltaR = ((Right.rotation(deg)-prevR))+tOY;
  deltaS = ((Back.rotation(deg)- prevS))-tOX;

//deltaR = ((Right.rotation(deg)-prevR)*wheelDiameter*M_PI)/360;
//deltaS = ((Back.rotation(deg)- prevS)*wheelDiameter*M_PI)/360;



   //deltaR = ((Right.rotation(deg)+tOY-prevR));//*wheelDiameter/2);
  //deltaS = ((Back.rotation(deg)-tOX- prevS));//*wheelDiameter/2);
  //double deltaR = (((Right.rotation(deg)-turnOffsety)-prevR) *pi/180);
  //double deltaS = (((Back.rotation(deg)-turnOffsetx)- prevS) *pi/180);
//is this the issue
//change to int? idk however I can to store variable.
  prevR = Right.rotation(deg);
  prevS = Back.rotation(deg);
 // double prevTOffsetX = turnOffsetx;
  //double prevTOffsetY = turnOffsety;

  totalDeltaDistR += deltaR;
  if (TurnGyroSmart.heading() == 0) {
    //absoluteAngle = ConvertToRadians(fmod(-360+90,360));
    //   absoluteAngle=ConvertToRadians(360-TurnGyroSmart.heading());
    absoluteAngle=ConvertToRadians(TurnGyroSmart.heading());
     //absoluteAngle = (fmod((-360+90),360));
  } else {
    absoluteAngle=ConvertToRadians(TurnGyroSmart.heading());
    //   absoluteAngle=ConvertToRadians(360-TurnGyroSmart.heading());

  //absoluteAngle = ConvertToRadians(fmod((-TurnGyroSmart.heading()+90),360));
   //absoluteAngle = fmod((-(TurnGyroSmart.heading()+90),360);
//absoluteAngle = trunc(fmod(TurnGyroSmart.heading() * -1 + 90,360));
  }
  if (absoluteAngle <0) {
    absoluteAngle+=2*M_PI;
  }
  if (absoluteAngle >360) {
    absoluteAngle-=2*M_PI;
  }

  deltaTheta = absoluteAngle - prevTheta;

  prevTheta = absoluteAngle;

  if ((deltaTheta) <= .5 && deltaTheta >= -.5) {
    deltaXLocal = (deltaS);
    deltaYLocal = (deltaR);
    
    halfAngle = 0;
    
  } else {
    halfAngle = deltaTheta/2;
    r = deltaR/deltaTheta;
    r2 = deltaS/deltaTheta;
deltaXLocal = 2*sin(halfAngle) *(r2+sDist);
deltaYLocal = 2*sin(halfAngle) * (r-rDist);
//y = 2sin(half angle * r+rDist)

//what is it
   // deltaXLocal = 2*sin(ConvertToRadians(halfAngle)) * ConvertToRadians(r2 + sDist);
    //deltaYLocal = 2*sin(ConvertToRadians(halfAngle)) * ConvertToRadians(r +rDist);
  }
  avgTheta = absoluteAngle - halfAngle;
  //abs - half


  //amount of change from global coord
//something needs to change with calculating angle
//check the sin and all that so when it hasnt moved it = 0 so it isn't adding.
//all units are degrees (radial) which means they come out with currect sin.
//changing back  to radians because sin is always in radians.
//double avgThetaR = (avgTheta);

double thetaT = atan2(deltaYLocal,deltaXLocal);
double radius = sqrt(deltaXLocal*deltaXLocal + deltaYLocal*deltaYLocal);
//thetaT -= avgTheta;
//deltaXGlobal = radius*cos(thetaT);
//deltaYGlobal = radius*sin(thetaT);

deltaYGlobal=deltaXLocal*sin(absoluteAngle)+deltaYLocal*cos(absoluteAngle);
deltaXGlobal=deltaXLocal*cos(absoluteAngle)-deltaYLocal*sin(absoluteAngle);
  //deltaXGlobal = (deltaXLocal) *cos(avgTheta) - (deltaYLocal) * sin(avgTheta);
  //xcos - ysin
  //deltaYGlobal = (deltaYLocal) * cos(avgTheta) + (deltaXLocal) * -sin(avgTheta);
  //dyg = dyl*cos(avgtheta) +deltaxl(sin(avgtheta))

//double deltaYGlobalD = ConvertToDeg(deltaYGlobal);
//double deltaXGlobalD = ConvertToDeg(deltaXGlobal);


//always adding never stops
  xPosGlobal += (deltaXGlobal);
  yPosGlobal += (deltaYGlobal);
  //xPosGlobal += deltaXGlobal;
  //yPosGlobal += deltaYGlobal;
  //Brain.Screen.setCursor(12,12);
  //Brain.Screen.print("%.5", prevR);
  draw();

  task::sleep(10);

}
return 1;
}

/*

int trackPosition() {
  
  
  while(1) {
  rEnc = Right.rotation(deg);
  sEnc = Back.rotation(deg);
//double turnOffsetx = 360*(sDist*deltaTheta*2)/wheelDiameter;
//double turnOffsety = 360*(rDist*deltaTheta*2)/wheelDiameter;

  deltaR = ((rEnc)-prevR);
  deltaS = ((sEnc -prevS));
  if (fabs(deltaR) >=0 && fabs(deltaR) <=4) {
    deltaR =0;
  }
  if(fabs(deltaS) >= 0 && fabs(deltaS)<=4) {
    deltaS =0;
  }
  //double deltaR = (((Right.rotation(deg)-turnOffsety)-prevR) *pi/180);
  //double deltaS = (((Back.rotation(deg)-turnOffsetx)- prevS) *pi/180);
//is this the issue
//change to int? idk however I can to store variable.
  //prevR = Right.rotation(deg);
  //prevS = Back.rotation(deg);
    prevR = rEnc;
  prevS = sEnc;

  totalDeltaDistR += deltaR;
  if (TurnGyroSmart.heading() >= 0 && TurnGyroSmart.heading() <=1) {
    //absoluteAngle = ConvertToRadians(fmod(-360+90,360));
     absoluteAngle = trunc((fmod((-360+90),360)));
  } else {
  //absoluteAngle = ConvertToRadians(fmod((-TurnGyroSmart.heading()+90),360));
   //absoluteAngle = fmod((-(TurnGyroSmart.heading()+90),360);
absoluteAngle = trunc(fmod(TurnGyroSmart.heading() * -1 + 90,360));
  }
  
  if ((absoluteAngle <= 93 && absoluteAngle>=90) || (absoluteAngle <= 90 && absoluteAngle>= 88) ) {
    absoluteAngle = 90;
  }
  if((absoluteAngle <= 182 && absoluteAngle >= 180)|| (absoluteAngle <=180 &&absoluteAngle >=179)) {
    absoluteAngle = 180;
  }
  if((absoluteAngle <=272 && absoluteAngle >=270)|| (absoluteAngle <=270 &&absoluteAngle >= 269)) {
    absoluteAngle = 270;
  }
  if ((absoluteAngle <=360 && absoluteAngle >= 359) || (absoluteAngle >= 0 && absoluteAngle <=1)) {
    absoluteAngle = 360;
  }
  deltaTheta = trunc(absoluteAngle - prevTheta);

  prevTheta = trunc(absoluteAngle);

  if (fabs(deltaTheta) >=0 && fabs(deltaTheta) <=3) {
    deltaXLocal = ConvertToRadians(deltaS);
    deltaYLocal = ConvertToRadians(deltaR);
    halfAngle = 0;
    r = 0;
    r2 = 0;
  } else {
    halfAngle = deltaTheta/2;
    r = deltaR/deltaTheta;
    r2 = deltaS/deltaTheta;

    deltaXLocal = 2*sin(ConvertToRadians(halfAngle)) * ConvertToRadians(r2 + sDist);
    deltaYLocal = 2*sin(ConvertToRadians(halfAngle)) * ConvertToRadians(r +rDist);
  }
  avgTheta = absoluteAngle - halfAngle;
  //amount of change from global coord
//something needs to change with calculating angle
//check the sin and all that so when it hasnt moved it = 0 so it isn't adding.
//all units are degrees (radial) which means they come out with currect sin.
//changing back  to radians because sin is always in radians.
//double avgThetaR = ConvertToRadians(avgTheta);
avgThetaR = ConvertToRadians(absoluteAngle);
 // deltaXGlobal += (deltaS) *cos(avgThetaR) - (deltaR) * sin(avgThetaR);
  //deltaYGlobal += (deltaR) * cos(avgThetaR) + (deltaS) * sin(avgThetaR);
  deltaXGlobal = (deltaXLocal) *cos(avgThetaR) - (deltaYLocal) * sin(avgThetaR);
  deltaYGlobal = (deltaYLocal) * cos(avgThetaR) + (deltaXLocal) * sin(avgThetaR);
deltaYGlobalD = ConvertToDeg(deltaYGlobal);
deltaXGlobalD = ConvertToDeg(deltaXGlobal);

    while(absoluteAngle > 360) {
    absoluteAngle -= 360;
  }
  while(absoluteAngle < 0) {
    absoluteAngle += 360;
  }
//always adding never stops

  xPosGlobal += (deltaXGlobalD);
  yPosGlobal += (deltaYGlobalD);
  //xPosGlobal += deltaXGlobal;
  //yPosGlobal += deltaYGlobal;
  //Brain.Screen.setCursor(12,12);
  //Brain.Screen.print("%.5", prevR);

  task::sleep(25);

}
return 1;
}
*/



void driveToP(double xTarget, double yTarget, double targetA) {
 xTargetLocation = (xTarget*360)/(M_PI*wheelDiameter);
 yTargetLocation = (yTarget*360)/(M_PI*wheelDiameter);
 //xTargetLocation=xTarget;
 //yTargetLocation=yTarget;
 targetAC=ConvertToRadians(targetA);
   xVector = xTargetLocation - xPosGlobal;
  yVector = yTargetLocation - yPosGlobal;
  double head_Error = absoluteAngle-targetAC;
  //while(fabs(yVector)>10 ||fabs(xVector)>10 || fabs(head_Error)>10) {
    while(sqrt(xVector*xVector+yVector*yVector)>10 || (atan2(yVector,xVector))-absoluteAngle >10 ) {
   // driveTo();
   driveToNS();
    turnTo();
    //strafeTo();

    FrontLeftPower = ((drivePID.powerDrive)/3 -turnPID.powerDrive*5);//+strafePID.powerDrive;
    BackLeftPower = (drivePID.powerDrive/3-turnPID.powerDrive*5);//-strafePID.powerDrive;
    FrontRightPower = (drivePID.powerDrive/3 + turnPID.powerDrive*5);//-strafePID.powerDrive;
  BackRightPower = (drivePID.powerDrive/3 +turnPID.powerDrive*5);//+strafePID.powerDrive;
  /*
   
if ((xTargetLocation<=xPosGlobal-1&&xTargetLocation<=xPosGlobal+1) || (yTargetLocation<=yPosGlobal-1 && yTargetLocation<=yPosGlobal+1)){
    FrontLeft.spin(forward,-FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,-FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,-BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,-BackRightPower,voltageUnits::volt);
} else if ((xTarget>=xPosGlobal+1 && xTarget>=xPosGlobal-1) || (yTargetLocation>=yPosGlobal+1 && yTargetLocation>=yPosGlobal-1)) {
    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);
} else {
  break;
}
*/

    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);
//robot turns to fix x in wrong direction since technically that side gets bigger turning left but we want to turn right

 printf( "      \n");
 printf( "xPos %.5f\n", xPosGlobal);
 printf( "yPos %.5f\n", yPosGlobal);
 printf( "dYG %.5f\n", deltaYGlobal);
 printf( "dxg %.5f\n", deltaXGlobal);
 printf( "dxL %.5f\n", deltaXLocal);
 printf( "dyl %.5f\n", deltaYLocal);
 printf( "dR %.5f\n", deltaR);
 printf( "ds %.5f\n", deltaS);
 printf( "prevr %.5f\n", prevR);
 printf( "ytarget %.5f\n", yTargetLocation);
  printf( "xtarget %.5f\n", xTargetLocation);
printf( "FLP %.5f\n", FrontLeftPower);
printf( "FRP %.5f\n",FrontRightPower);
printf( "BLP %.5f\n", BackLeftPower);
printf( "BRP %.5f\n", BackRightPower);
 printf( "New Iteration00 \n");
 printf( "      \n");

    task::sleep(15);
  }

FrontLeft.stop();
FrontRight.stop();
BackRight.stop();
BackLeft.stop();
}
/*

  void driveToP(double xTarget, double yTarget, int targetA) {
 xTarget = (xTarget*260*pi)/(pi*wheelDiameter*180);
 yTarget = (yTarget*360*pi)/(pi*wheelDiameter*180);
  xTargetLocation = xTarget;
  yTargetLocation = yTarget;
  targetAC = ((-targetA+90)%360);
  if (targetAC == 0) {
    targetAC = 360;
  }

  
 
  
  //ShortAngle();

  targetFacA = ConvertToRadians(targetAC);
    if (targetFacA <0) {
    targetFacA +=2*pi;
  }
  if (targetFacA > 0) {
    targetFacA -= 2*pi;
  }
  chassisControl = true;
}

  */
/*
  void driveToP(double xTarget, double yTarget, int targetA) {
 xTargetLocation = (xTarget*360)/(M_PI*wheelDiameter);
 yTargetLocation = (yTarget*360)/(M_PI*wheelDiameter);
  //xTarget = (xTarget*360*M_PI)/(M_PI*wheelDiameter*180);
 //yTarget = (yTarget*360*M_PI)/(M_PI*wheelDiameter*180);

  //xTargetLocation = xTarget;
  //yTargetLocation = yTarget;

  targetAC = ConvertToRadians(targetA);

  
 
  
  //ShortAngle();

  targetFacA = ConvertToRadians(targetAC);
  //targetFacA = atan2(yTargetLocation-yPosGlobal,xTargetLocation-xPosGlobal)-M_PI/2;
  //targetFacA = targetAC;
    if (targetFacA <0) {
    targetFacA +=2*pi;
  }
  if (targetFacA > 2*pi) {
    targetFacA -= 2*pi;
  }
  xVector = xTargetLocation - xPosGlobal;
  yVector = yTargetLocation - yPosGlobal;
  double headError = absoluteAngle - targetAC;//targetfaca
  //while(yVector >3) { //|| turnPID.error > .008 || strafePID.error < .3) {
  //while(sqrt(xVector*xVector + yVector*yVector) > .3 && headError > .008) {
    //while (fabs(xVector) > 3 && fabs(yVector) >3) {
if (xVector !=0 && yVector != 0) { 
  while (fabs(xVector) >7 && fabs(yVector) >7)  {
driveToNS();
//driveTo();
turnTo();
//strafeTo();

   FrontLeftPower = ((drivePID.powerDrive) -turnPID.powerDrive)*10;//+strafePID.powerDrive;
    BackLeftPower = (drivePID.powerDrive-turnPID.powerDrive)*10;//-strafePID.powerDrive;
    FrontRightPower = (drivePID.powerDrive + turnPID.powerDrive)*10;//-strafePID.powerDrive;
    BackRightPower = (drivePID.powerDrive +turnPID.powerDrive)*10;//+strafePID.powerDrive;

    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);


 printf( "xPos %.5f\n", xPosGlobal);
 printf( "yPos %.5f\n", yPosGlobal);
 printf( "dYG %.5f\n", deltaYGlobal);
 printf( "dxg %.5f\n", deltaXGlobal);
 printf( "dxL %.5f\n", deltaXLocal);
 printf( "dyl %.5f\n", deltaYLocal);
 printf( "dR %.5f\n", deltaR);
 printf( "ds %.5f\n", deltaS);
 
 printf( "prevr %.5f\n", prevR);
 printf( "targetFacA %.5f\n", targetFacA);
 printf( "absoluteAng %.5f\n", absoluteAngle);
  printf( "xTarget %.5f\n", xTargetLocation);
    printf( "yTarget %.5f\n", yTargetLocation);
 printf( "FRPower %.5f\n", FrontRightPower);


 printf( "New Iteration \n");
 
 printf( "      \n");


this_thread::sleep_for(15);
  }
  Drivetrain.stop();
}

if(xVector ==0 && yVector !=0) {
  while (yVector >7){
    driveToNS();
    turnTo();
    
   FrontLeftPower = ((drivePID.powerDrive) -turnPID.powerDrive);//+strafePID.powerDrive;
    BackLeftPower = (drivePID.powerDrive-turnPID.powerDrive);//-strafePID.powerDrive;
    FrontRightPower = (drivePID.powerDrive + turnPID.powerDrive);//-strafePID.powerDrive;
    BackRightPower = (drivePID.powerDrive +turnPID.powerDrive);//+strafePID.powerDrive;

    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);


 printf( "xPos %.5f\n", xPosGlobal);
 printf( "yPos %.5f\n", yPosGlobal);
 printf( "dYG %.5f\n", deltaYGlobal);
 printf( "dxg %.5f\n", deltaXGlobal);
 printf( "dxL %.5f\n", deltaXLocal);
 printf( "dyl %.5f\n", deltaYLocal);
 printf( "dR %.5f\n", deltaR);
 printf( "ds %.5f\n", deltaS);
 
 printf( "prevr %.5f\n", prevR);
 printf( "targetFacA %.5f\n", targetFacA);
 printf( "absoluteAng %.5f\n", absoluteAngle);
  printf( "xTarget %.5f\n", xTargetLocation);
    printf( "yTarget %.5f\n", yTargetLocation);
 printf( "FRPower %.5f\n", FrontRightPower);


 printf( "New Iteration 111\n");
 
 printf( "      \n");
    this_thread::sleep_for(15);
  }
LeftDriveSmart.stop();
RightDriveSmart.stop();
xPosGlobal=xTargetLocation;
yPosGlobal=yTargetLocation;

}
if(yVector ==0&& xVector !=0) {
  while ( xVector >7) {
    driveToNS();
    turnTo();
    
   FrontLeftPower = ((drivePID.powerDrive) -turnPID.powerDrive);//+strafePID.powerDrive;
    BackLeftPower = (drivePID.powerDrive-turnPID.powerDrive);//-strafePID.powerDrive;
    FrontRightPower = (drivePID.powerDrive + turnPID.powerDrive);//-strafePID.powerDrive;
    BackRightPower = (drivePID.powerDrive +turnPID.powerDrive);//+strafePID.powerDrive;

    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);


 printf( "xPos %.5f\n", xPosGlobal);
 printf( "yPos %.5f\n", yPosGlobal);
 printf( "dYG %.5f\n", deltaYGlobal);
 printf( "dxg %.5f\n", deltaXGlobal);
 printf( "dxL %.5f\n", deltaXLocal);
 printf( "dyl %.5f\n", deltaYLocal);
 printf( "dR %.5f\n", deltaR);
 printf( "ds %.5f\n", deltaS);
 
 printf( "prevr %.5f\n", prevR);
 printf( "targetFacA %.5f\n", targetFacA);
 printf( "absoluteAng %.5f\n", absoluteAngle);
  printf( "xTarget %.5f\n", xTargetLocation);
    printf( "yTarget %.5f\n", yTargetLocation);
 printf( "FRPower %.5f\n", FrontRightPower);


 printf( "New Iteration 222\n");
 
 printf( "      \n");
    this_thread::sleep_for(15);
  }
 Drivetrain.stop();
 xPosGlobal=xTargetLocation;
 yPosGlobal=yTargetLocation;
}
  LeftDriveSmart.stop();
  RightDriveSmart.stop();

}
*/
/*

void driveToP(double xTarget, double yTarget, double targetA) {
 xTargetLocation = (xTarget*360)/(M_PI*wheelDiameter);
 yTargetLocation = (yTarget*360)/(M_PI*wheelDiameter);

  targetAC= ConvertToRadians(targetA);
  chassisControl=true;

}
int chassisTrack() {
  while(1) {
    if(chassisControl == true) {
      xVector=xTargetLocation-xPosGlobal;
      yVector=yTargetLocation-yPosGlobal;
      double hyAngle = atan2(yVector,xVector);
      double vector = sqrt(xVector*xVector + yVector*yVector);
      if(hyAngle <0) {
        hyAngle+=2*M_PI;

      }
      double relAngle = hyAngle-absoluteAngle+M_PI_2;
      if (relAngle>2*M_PI) {
        relAngle-=2*M_PI;
      }
      else if(relAngle<0) {
        relAngle+=2*M_PI;
      }
      driveToNS();
      turnTo();
    FrontLeftPower = ((drivePID.powerDrive/2) -turnPID.powerDrive);//+strafePID.powerDrive;
    BackLeftPower = (drivePID.powerDrive/2-turnPID.powerDrive);//-strafePID.powerDrive;
    FrontRightPower = (drivePID.powerDrive/2 + turnPID.powerDrive);//-strafePID.powerDrive;
    BackRightPower = (drivePID.powerDrive/2 +turnPID.powerDrive);//+strafePID.powerDrive;

    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);
    if(fabs(drivePID.error)<3 && fabs(turnPID.error)<3) {
      chassisControl=false;
      xPosGlobal=xTargetLocation;
      yPosGlobal=yTargetLocation;
    }

    } else {
      LeftDriveSmart.stop();
      RightDriveSmart.stop();
      
    }
    task::sleep(15);
  }
  return 1;
}

*/




/*
int chassisTrack() {
  //while(1) {
  //if(chassisControl == true) {
    while(chassisControl ==true) {
    xVector = xTargetLocation - xPosGlobal;
    yVector = yTargetLocation - yPosGlobal;

    //hAngle = atan2(xVector,yVector);
    hAngle = atan2(ConvertToRadians(xVector/(pi*wheelDiameter)*360),ConvertToRadians(yVector/(pi*wheelDiameter)*360));

driveToNS();
   // driveTo();
    turnTo();
    //strafeTo();



    FrontLeftPower = ((drivePID.powerDrive) +turnPID.powerDrive);//+strafePID.powerDrive;
    BackLeftPower = drivePID.powerDrive+turnPID.powerDrive;//-strafePID.powerDrive;
    FrontRightPower = drivePID.powerDrive - turnPID.powerDrive;//-strafePID.powerDrive;
    BackRightPower = drivePID.powerDrive -turnPID.powerDrive;//+strafePID.powerDrive;

    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);

    //if(fabs(drivePID.error) < drivePID.threshold &&fabs(turnPID.error) < turnPID.threshold && fabs(strafePID.error) < strafePID.threshold) {
     //if(fabs(xTargetLocation-xPosGlobal) <.5 && fabs(yTargetLocation-yPosGlobal) <.5 && (targetFacA-absoluteAngle) <.008) {
     if(fabs(drivePID.error) < .01 &&fabs(turnPID.error) < .008) {
      chassisControl=false;


    } 
    if (chassisControl==false) {
      break;
    }


    
  Controller1.Screen.newLine();
  Brain.Screen.print("%.5f", xTargetLocation);
  Brain.Screen.newLine();
  Brain.Screen.print("%.5f", xPosGlobal);
  Brain.Screen.newLine();
  Brain.Screen.print("%.5f", yTargetLocation);
  

    task::sleep(20);
  
  } 
  FrontLeft.stop();
  FrontRight.stop();
  BackRight.stop();
  BackLeft.stop();
  return 1;
  }
  */
  



  
