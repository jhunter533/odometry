using namespace vex;

extern brain Brain;
extern controller Controller1;
extern triport ThreeWire;
extern motor FrontLeft;
extern motor BackLeft;
extern motor FrontRight;
extern motor BackRight;
extern encoder Right;
extern encoder Back;
extern sonar Sonic;
extern inertial TurnGyroSmart;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
