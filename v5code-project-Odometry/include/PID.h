#ifndef PID
#define PID

typedef struct _pid{
  double integral;
  double derivative;
  double error;
  double prevError;
  double powerDrive;
  double kP;
  double kD;
  double kI;
  double threshold;
  double iBound;
  double maxSpeed = 8;
} PID_;

extern PID_ turnPID;
extern PID_ strafePID;
extern PID_ drivePID;
void driveToNS();
void driveTo();
void turnTo();
void strafeTo();



#endif