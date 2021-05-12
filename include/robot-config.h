#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;

extern brain Brain;

//vex::controller Controller1;
//vex::motor RightRoller (vex::PORT10, vex::gearSetting::ratio18_1,true);
//vex::motor LeftRoller (vex::PORT9, vex::gearSetting::ratio18_1,false);
//vex::motor LowerIndexer (vex::PORT3, vex::gearSetting::ratio6_1,false);
//vex::motor UpperIndexer (vex::PORT4, vex::gearSetting::ratio6_1,false);

extern vex::motor RF;
//extern vex::motor RF(vex::PORT13, vex::gearSetting::ratio18_1,true);
extern vex::motor RB;
extern vex::motor LF;
extern vex::motor LB;
extern double ptermval;
extern double itermval;
extern double dtermval;
extern double left_vel_val;
extern double right_vel_val;


//inertial IMU = inertial(PORT7);


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
