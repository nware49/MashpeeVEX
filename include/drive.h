// This is a header guard, which you don't necessarily need but it can be helpful
// because it makes sure that the compiler includes the file only once, so if you end
// up including it multiple times it doesn't give an error. At the end of the file is "#endif", and 
// everything in between is guarded. In the lines below, where I wrote "MY_FILE" it doesn't matter
// what you write there, as long as it's different in each header file.
#ifndef DRIVE
#define DRIVE

//#include "vex.h"
void leftside(int vel);
void rightside(int vel);
void reset(void);
void leftSlew(int leftTarget);
void rightSlew(int rightTarget);
int driveTask();
int turnTask();
bool isDriving();
void driveAsync(int sp);
void turnAsync(int sp);
void drive(int sp);
void turn(int sp);

#endif
