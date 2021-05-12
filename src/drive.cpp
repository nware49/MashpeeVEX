/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "robot-config.h"
//#include "VisionSensor.h"
#include <stdio.h>
#include <string.h>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>

using namespace vex;


//double inch2deg= 360.0/12.56;
//double deg2inch=12.56/360;
//int sample_cnt;


const double fc = 1; //friction coefficient
#define MAX 100
#define MAX_TURN 20
static int driveMode = -1;
static int driveTarget = 0;
static int turnTarget = 0;
static int maxSpeed = 100;
static int maxTurnSpeed = 20;
static int slant = 0;
bool mirror=true;

extern int frontSwitchCount;
int bumperCount = 0;

//vex::task auton_drive_task;




// A global instance of competition




void leftside(int vel)
{
  LF.spin(directionType::fwd,vel,velocityUnits::pct);
  LB.spin(directionType::fwd,vel,velocityUnits::pct); 
}

void rightside(int vel){
  RF.spin(directionType::fwd,vel,velocityUnits::pct);
  RB.spin(directionType::fwd,vel,velocityUnits::pct);
}




/**************************************************/
//slew control
const int accel_step = 2;
const int deccel_step = 256; // no decel slew
static int leftSpeed = 0;
static int rightSpeed = 0;

void leftSlew(int leftTarget){
  int step;

  if(abs(leftSpeed) < abs(leftTarget))
    step = accel_step;
  else
    step = deccel_step;

  if(leftTarget > (leftSpeed + step))
    leftSpeed += step;
  else if(leftTarget < (leftSpeed - step))
    leftSpeed -= step;
  else
    leftSpeed = leftTarget;
left_vel_val=leftSpeed;
  leftside(leftSpeed);
}

//slew control
void rightSlew(int rightTarget){
  int step;

  if(abs(rightSpeed) < abs(rightTarget))
    step = accel_step;
  else
    step = deccel_step;

  if(rightTarget > (rightSpeed + step))
    rightSpeed += step;
  else if(rightTarget < (rightSpeed - step))
    rightSpeed -= step;
  else
    rightSpeed = rightTarget;
  //FIXME  
  right_vel_val=rightSpeed;
  rightside(rightSpeed);
}

void reset(void){
  maxSpeed = MAX;
  maxTurnSpeed = MAX_TURN;
  driveMode = -1;
  slant = 0;
  driveTarget = 0;
  turnTarget = 0;
  leftSlew(0);
  rightSlew(0);
  wait(30,msec);
  LF.resetRotation();
  LB.resetRotation();
  RF.resetRotation();
  RB.resetRotation();
  leftside(0);
  rightside(0);
}

/**************************************************/
//task control
int driveTask(){
  int prevError = 0;
  int integral=0;
  int drivetaskcnt=0;

  while(1){
    if(bumperCount == frontSwitchCount || driveTarget<=0){
    task::sleep(20);
    drivetaskcnt++;

    if(driveMode != 1)
      continue;

    int sp = driveTarget;
    int startIntegralPosition=500;
    double kp = .22;
    //double kd = .5;  
    double kd = 0.5;
     //kd = 0;
    double ki = 0.005;
    //ki =0;
    int minSpeed = 8;
    double rpos=RF.position(rotationUnits::deg);
    double lpos=LF.position(rotationUnits::deg);
  

    //read sensors
    double sv = (lpos+rpos)/2;
    double wheel_diff= lpos-rpos; 

    //speed+;

    //speed
    double error = sp-sv;

    double derivative = (error - prevError);
    if(abs(driveTarget) <250)
      {
        startIntegralPosition = abs(driveTarget);
      }
    else if (abs(driveTarget) <= 300.0)  
      {
        startIntegralPosition = 0.5*abs(driveTarget);
      }
    else if (abs(driveTarget) <= 400.0)  
      {
        startIntegralPosition = 0.6*abs(driveTarget);
      }
    else if (abs(driveTarget) <= 500.0)  
      {
        startIntegralPosition = 0.7*abs(driveTarget);
      }            
    if (fabs(error) > startIntegralPosition)
       integral =0;
    if (fabs(error) < startIntegralPosition)
      {
      integral += error;
      }


    prevError = error;

    if (((driveTarget > 0) && (error < 0)) || ((driveTarget < 0) && (error > 0)))
      {
        error = 0;
        integral=0;
        derivative=0;
        LF.stop(brakeType::brake);
        RF.stop(brakeType::brake);
        LB.stop(brakeType::brake);
        RB.stop(brakeType::brake);
      //  driveMode=0;
        
      }
    
    double speed = error*kp + integral*ki + derivative*kd;
    // set global values to be recorded
    ptermval = (double) error;
    itermval = (double) integral;
    dtermval = (double) derivative;

    if(speed > maxSpeed)
      speed = maxSpeed;
    else if ((speed > 0)  && (speed <minSpeed))
      speed = minSpeed;
    else if ((speed > -minSpeed)  && (speed < 0))
      speed = -minSpeed; 
    if(speed < -maxSpeed)
      speed = -maxSpeed;


    // left_vel_val=(speed-slant);
    // right_vel_val=(speed+slant);
    left_vel_val=(speed);
    right_vel_val=(speed + .3*wheel_diff);

    //set motors
    leftSlew(left_vel_val);

    rightSlew(right_vel_val);
    }
    else{
      LF.stop(brakeType::brake);
      RF.stop(brakeType::brake);
      LB.stop(brakeType::brake);
      RB.stop(brakeType::brake);
      driveMode = -1;
    }
  }
}

void turnTask(){
  double prevError;
  int minTurnSpeed = 8;
  double integral=0;
  

  while(1){
    task::sleep(20);

    if(driveMode != 0)
      continue;

    double sp = turnTarget;

    if(sp > 0)
      sp *= 2.00;
  //    sp *= 2.35;
    else
   //   sp *= 2.3;
      sp *= 2.00;

    double kp = .2;
    double kd = .5;
    double ki = .001;

    double sv = (RF.position(rotationUnits::deg) - LF.position(rotationUnits::deg))/2;
    double error = sp-sv;
    double derivative = error - prevError;
    prevError = error;
    

    if (((turnTarget > 0) && (error < 0)) || ((turnTarget < 0) && (error > 0)))
      {
        error = 0;
        integral=0;
        derivative=0;
        LF.stop(brakeType::brake);
        RF.stop(brakeType::brake);
        LB.stop(brakeType::brake);
        RB.stop(brakeType::brake);
        
      }

    if (fabs(error) > 1000)
       integral =0;
    if (fabs(error) < 1000)
      {
      integral += error;
      }

    ptermval = (double) error;
    itermval = (double) integral;
    dtermval = (double) derivative;

    double  speed = error*kp + derivative*kd + integral*ki;
/*    if(speed > maxSpeed)
      speed = maxSpeed;
    if(speed < -maxSpeed)
      speed = -maxSpeed;
*/
    if(speed > maxTurnSpeed)
      speed = maxTurnSpeed;
    else if ((speed > 0)  && (speed <minTurnSpeed))
      speed = minTurnSpeed;
    else if ((speed > -minTurnSpeed)  && (speed < 0))
      speed = -minTurnSpeed; 
    if(speed < -maxTurnSpeed)
      speed = -maxTurnSpeed;
    left_vel_val=(-speed);
    right_vel_val=(speed);

    leftSlew(left_vel_val);
    rightSlew(right_vel_val);
    }
}

/**************************************************/
//feedback
bool isDriving(){
  static int count = 0;
  static int last = 0;
  static int lastTarget = 0;

  int curr = LF.position(rotationUnits::deg);

  int target = turnTarget;
  if(driveMode == 1)
    target = driveTarget;

  if(abs(last-curr) < 3)
    count++;
  else
    count = 0;

  if(target != lastTarget)
    count = 0;

  lastTarget = target;
  last = curr;

  //not driving if we haven't moved
  if(count > 4)
    return false;
  else
    return true;

}

/**************************************************/
//autonomous functions
void driveAsync(int sp){
  bumperCount = frontSwitchCount;
  reset();
  driveTarget = sp;
  driveMode = 1;
}

void turnAsync(int sp){
  if(mirror)
    sp = -sp; // inverted turn for blue auton
  reset();
  turnTarget = sp;
  driveMode = 0;
}

void drive(int sp){
  driveAsync(sp);
  task::sleep(450);
  while(isDriving()) task::sleep(20);
}

void turn(int sp){
  turnAsync(sp);
  task::sleep(450);
  while(isDriving()) task::sleep(20);
}



