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
#include "VisionSensor.h"
#include "drive.h"
#include <stdio.h>
#include <string.h>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include "drive.h"

using namespace vex;
int tilt_lock=0;
int lift_lock=0;
int move_lock=0;
int push=0;
int abort_flag=0;
int lift_location=0; //0=back, 1=low tower, 2=mid tower
const int cJoyDead = 4;
const int cMotorMin = 10;
const float cDriveExp = 1.4;
long int time_data[6000];
double heading_data[6000];
double gyro_data[6000];
double rpos[6000];
double lpos[6000];
double left_mtr_vel[6000];
double right_mtr_vel[6000];
double wheel_angle_diff[6000];
double accelx_data[6000];
double accely_data[6000];
int joyLeft[6000];
int joyRight[6000];
int rawJoyLeft;
int rawJoyRight;
double pterm[6000];
double iterm[6000];
double dterm[6000];
double left_vel_val=0.0;
double right_vel_val=0.0;
double wheel_error=1.0;

double inch2deg= 360.0/12.56;
double deg2inch=12.56/360;
int sample_cnt;
double ptermval,itermval,dtermval;
long int time_top_switch = 0;
long int time_bottom_switch = 0;
long int time_front_switch = 0;
int topSwitchCount=0;
int bottomSwitchCount=0;
int frontSwitchCount=0;
int topSwitchPressed=0;
int bottomSwitchPressed=0;
int frontSwitchPressed=0;

brain Brain;
vex::controller Controller1;
vex::motor RightRoller (vex::PORT10, vex::gearSetting::ratio18_1,true);
vex::motor LeftRoller (vex::PORT1, vex::gearSetting::ratio18_1,false);
vex::motor LowerIndexer (vex::PORT3, vex::gearSetting::ratio6_1,false);
vex::motor UpperIndexer (vex::PORT9, vex::gearSetting::ratio6_1,false);
vex::motor RF (vex::PORT13, vex::gearSetting::ratio18_1,true);
vex::motor RB (vex::PORT11, vex::gearSetting::ratio18_1,false);
vex::motor LF (vex::PORT14, vex::gearSetting::ratio18_1,true);
vex::motor LB (vex::PORT12, vex::gearSetting::ratio18_1,false);
inertial IMU = inertial(PORT7);
limit TopSwitch = limit(Brain.ThreeWirePort.G);
limit BottomSwitch = limit(Brain.ThreeWirePort.E);
bumper FrontSwitch = bumper(Brain.ThreeWirePort.D);

extern vex::vision::signature Vision1_SIG_1;
extern vex::vision::signature Vision1_SIG_2;
extern vex::vision::signature Vision1_SIG_3;
extern vex::vision::signature Vision1_SIG_4;
extern vex::vision::signature Vision1_SIG_5;
extern vex::vision::signature Vision1_SIG_6;
extern vex::vision::signature Vision1_SIG_7;
extern vex::vision Vision1;

void TopSwitchIsPressed() { 
  long int swtime = vex::timer::system();
    if (swtime > (time_top_switch+50))
      {
      topSwitchCount++;
      topSwitchPressed =1;
      //Brain.Screen.printAt(20,150, "Top Cnt: %d, mSec: %d    ", topSwitchCount,swtime - time_top_switch);
      time_top_switch = vex::timer::system();
      } 
    }
void BottomSwitchIsPressed() {  
  long int swtime = vex::timer::system(); 
   if (swtime > (time_bottom_switch+200))
      {
      bottomSwitchCount++;
      bottomSwitchPressed =1;
      //Brain.Screen.printAt(20,180, "Bot Cnt: %d, mSec: %d    ", bottomSwitchCount,swtime - time_bottom_switch);
            time_bottom_switch = vex::timer::system();
      } 
    }

void FrontSwitchIsPressed() { 
  long int swtime = vex::timer::system();
    if (swtime > (time_front_switch+200))
      {
      frontSwitchCount++;
      frontSwitchPressed =1;
      //Brain.Screen.printAt(20,200, "F Cnt: %d, mSec: %d    ", frontSwitchCount,swtime - time_front_switch);
      time_front_switch = vex::timer::system();
      } 
    }

void TopSwitchIsReleased() { topSwitchPressed=0; }
void BottomSwitchIsReleased() { bottomSwitchPressed=0;}
  
std::ofstream ofs;
vex::task imu_sample_task;
vex::task imu_write_task;
vex::task imu_sample_driver_task;
//FIXME
vex::task auton_drive_task;
vex::task auton_turn_task;



vex::line BottomBallLine = line(Brain.ThreeWirePort.B);
vex::line TopBallLine = line(Brain.ThreeWirePort.C);

// A global instance of competition

competition Competition;


int imu_write()
{

int write_cnt=0;

char rec_string[200];

     if( Brain.SDcard.isInserted() ) {
      // create a file with long filename
      ofs.open("programming_data.txt", std::ofstream::app);
      for (write_cnt=0;write_cnt<sample_cnt;write_cnt++)
        {
          sprintf(rec_string,"%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\r\n", time_data[write_cnt],heading_data[write_cnt],gyro_data[write_cnt],rpos[write_cnt],lpos[write_cnt],left_mtr_vel[write_cnt],right_mtr_vel[write_cnt],wheel_angle_diff[write_cnt],accelx_data[write_cnt],accely_data[write_cnt],pterm[write_cnt],iterm[write_cnt],dterm[write_cnt],joyLeft[write_cnt],joyRight[write_cnt]);
//          sprintf(rec_string,"%d,%.1f,%.1f\r\n", time_data[write_cnt],heading_data[write_cnt],gyro_data[write_cnt]);
          ofs << rec_string;
        }

      ofs.close();

      Brain.Screen.printAt(10, 70, "%d done",sample_cnt);
    }
    else {
      Brain.Screen.printAt(10, 40, "No SD Card");        
    }

return(0);
}

float tempRF = RF.temperature(temperatureUnits::fahrenheit);

void vehicle_turn(int turn_rot, int turn_vel)
    {
    int turn_timeout;
    float turn_max_sec;


    /* Drivetrain speed at 50% 3000 rotation took 5.05s */
    /* max speed = 3000 rotations/5 sec*100%/50% = 1188 rotations at 100 %  */
    /* to be safe assume use 900 rotation pers second to calculation timer */
    
    turn_max_sec= (float) turn_rot/900*(100/(float)turn_vel);
    turn_timeout = (int)(turn_max_sec*1000);  //convert to millisec
    if (turn_timeout <  1000)
    {
        turn_timeout =1000;
    }
    LF.startRotateFor(turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);
    RF.startRotateFor(-turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);
    LB.startRotateFor(turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);
    RB.startRotateFor(-turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);
    while((LF.isSpinning()||RF.isSpinning()||LB.isSpinning()||RB.isSpinning()) &&(turn_timeout> 0)){
        

        
        this_thread::sleep_for(10);
        turn_timeout -= 10;

    }

}

int imu_write_driver()
{

int write_cnt=0;

char rec_string[200];

     if( Brain.SDcard.isInserted() ) {
      // create a file with long filename
      ofs.open("driver_data.txt", std::ofstream::app);
      for (write_cnt=0;write_cnt<sample_cnt;write_cnt++)
        {
//          sprintf(rec_string,"%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", time_data[write_cnt],heading_data[write_cnt],gyro_data[write_cnt],rpos[write_cnt],lpos[write_cnt],left_mtr_vel[write_cnt],right_mtr_vel[write_cnt],wheel_angle_diff[write_cnt]);
          sprintf(rec_string,"%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\r\n", time_data[write_cnt],heading_data[write_cnt],gyro_data[write_cnt],rpos[write_cnt],lpos[write_cnt],left_mtr_vel[write_cnt],right_mtr_vel[write_cnt],wheel_angle_diff[write_cnt],accelx_data[write_cnt],accely_data[write_cnt],pterm[write_cnt],iterm[write_cnt],dterm[write_cnt],joyLeft[write_cnt],joyRight[write_cnt]);
//          sprintf(rec_string,"%d,%.1f,%.1f\r\n", time_data[write_cnt],heading_data[write_cnt],gyro_data[write_cnt]);
          ofs << rec_string;
        }
      ofs.close();

      Brain.Screen.printAt(10, 70, "%d done",sample_cnt);
    }
    else {
      Brain.Screen.printAt(10, 40, "No SD Card");        
    }

return(0);
}


int imu_sample()
{
sample_cnt = 0;
//int write_cnt=0;
int max_cnt=750;  //15*20 samples/sec

IMU.resetHeading();    
while (sample_cnt < max_cnt) 
  {
  //test[cnt]=test[cnt]+.1;
    time_data[sample_cnt]=vex::timer::system();
    heading_data[sample_cnt]=IMU.heading(rotationUnits::deg);
    gyro_data[sample_cnt]=IMU.gyroRate(axisType::zaxis, velocityUnits::dps); 
    accelx_data[sample_cnt]=IMU.acceleration(axisType::xaxis);
    accely_data[sample_cnt]=IMU.acceleration(axisType::yaxis); 
    rpos[sample_cnt]=RF.position(rotationUnits::deg);
    lpos[sample_cnt]=LF.position(rotationUnits::deg);
    joyLeft[sample_cnt]=0;
    joyRight[sample_cnt]=0;
    left_mtr_vel[sample_cnt]=left_vel_val;
    right_mtr_vel[sample_cnt]=right_vel_val;
    wheel_angle_diff[sample_cnt]=wheel_error;
    pterm[sample_cnt]=ptermval;
    iterm[sample_cnt]=itermval;
    dterm[sample_cnt]=dtermval;


    wait(20,msec);
    
    sample_cnt++;
//          Brain.Screen.printAt(10, 40, "Sample_cnt=%d",sample_cnt);  
   
          
   }


return(0);
}

int imu_sample_driver()
{


sample_cnt = 0;
int write_cnt=0;
int max_cnt=4400;  //15*20 samples/sec



//IMU.resetHeading();

    
while (sample_cnt < max_cnt) 
  {
  //test[cnt]=test[cnt]+.1;
    time_data[sample_cnt]=vex::timer::system();
    heading_data[sample_cnt]=IMU.heading(rotationUnits::deg);
    gyro_data[sample_cnt]=IMU.gyroRate(axisType::zaxis, velocityUnits::dps); 
    accelx_data[sample_cnt]=IMU.acceleration(axisType::xaxis); 
    accely_data[sample_cnt]=IMU.acceleration(axisType::yaxis); 
    rpos[sample_cnt]=RB.position(rotationUnits::deg);
    lpos[sample_cnt]=LB.position(rotationUnits::deg);
    joyLeft[sample_cnt]=rawJoyLeft;
    joyRight[sample_cnt]=rawJoyRight;
    left_mtr_vel[sample_cnt]=left_vel_val;
    right_mtr_vel[sample_cnt]=right_vel_val;
    wheel_angle_diff[sample_cnt]=wheel_error;
    wait(25,msec);
    
    sample_cnt++;
//          Brain.Screen.printAt(10, 40, "Sample_cnt=%d",sample_cnt);  
  
          
   }

  char rec_string[200];

     if( Brain.SDcard.isInserted() ) {
      // create a file with long filename
      ofs.open("driver_data.txt", std::ofstream::app);
      for (write_cnt=0;write_cnt<sample_cnt;write_cnt++)
        {
           sprintf(rec_string,"%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\r\n", time_data[write_cnt],heading_data[write_cnt],gyro_data[write_cnt],rpos[write_cnt],lpos[write_cnt],left_mtr_vel[write_cnt],right_mtr_vel[write_cnt],wheel_angle_diff[write_cnt],accelx_data[write_cnt],accely_data[write_cnt],pterm[write_cnt],iterm[write_cnt],dterm[write_cnt],joyLeft[write_cnt],joyRight[write_cnt]);
//          sprintf(rec_string,"%d,%.1f,%.1f\r\n", time_data[write_cnt],heading_data[write_cnt],gyro_data[write_cnt]);
          ofs << rec_string;
        }
      ofs.close();

      Brain.Screen.printAt(10, 70, "%d done",sample_cnt);
    }
    else {
      Brain.Screen.printAt(10, 40, "No SD Card");        
    }
 
return(0);
}


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  //vexcodeInit();

}
    


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//Move Function




int sgn(double value)
  {
    if (value>=0)
    return(1);
    else {
   return(-1);
    }
  }

 void set_driveVelocity(double dpsLeft, double dpsRight)
  {      
  left_vel_val=dpsLeft;
  right_vel_val=dpsRight;     
  LF.setVelocity(dpsLeft,velocityUnits::dps);
  RF.setVelocity(dpsRight,velocityUnits::dps);
  LB.setVelocity(dpsLeft,velocityUnits::dps);
  RB.setVelocity(dpsRight,velocityUnits::dps);     
  }
   
   
void start_driveVelocity(double dpsLeft, double dpsRight)
  {
  left_vel_val=dpsLeft;
  right_vel_val=dpsRight;
  RF.spin(directionType::fwd,dpsRight,velocityUnits::dps);
  LF.spin(directionType::fwd,dpsLeft,velocityUnits::dps);
  RB.spin(directionType::fwd,dpsRight,velocityUnits::dps);
  LB.spin(directionType::fwd,dpsLeft,velocityUnits::dps);       
  }     


void vehicle_move_inpersec(double move_inches, double move_vel, int from_zero)
{  

//double v;
double move_rot; 
int move_timeout;
    //int cnt;
double direction=1.0;
//double lfstart;
//double rfstart;
double leftstart,rightstart;
int spin_dir;
double move_abs;

int change_positions;
    //int ball_limit_reset=0;
    //int ball_limit_once=0;
float vel_change_pos[64];  
float left_vel[64];
float right_vel[64];
int cnt,revcnt,tcnt;
int last_position=0;
int next_change_pos=1;
double left_position=0.0; 

int move_mid=fabs(move_inches/2);
float vel_prof[32];
float pos_prof[32];
float timeout_val[32];
double right_wheel_ang,left_wheel_ang;
//double Kp=2;
//double rDiff;
double rMod;
//double sgn_error;
double wheel_factor;
//char outmes[320];
//char outstr[20];
int write_cnt;

double right_timeout,timeout;

memset(left_vel,0,sizeof(left_vel));
memset(right_vel,0,sizeof(right_vel));
memset(timeout_val,0,sizeof(timeout_val));

//original
/*
pos_prof[0]  = 0;  vel_prof[0]=2.0; 
pos_prof[1]  = 0.33;  vel_prof[1]=4.0;  // 10%
pos_prof[2] = 0.9; vel_prof[2]=6.0;
pos_prof[3] = 1.7; vel_prof[3]=8.0;  // 20%
pos_prof[4] = 2.7; vel_prof[4]=10.0;
pos_prof[5] = 3.9; vel_prof[5]=12.0; // 30%
pos_prof[6] = 5.3; vel_prof[6]=14.0;
pos_prof[7] = 6.9; vel_prof[7]=16.0; // 40

pos_prof[8] = 8.9; vel_prof[8]=20.0;
pos_prof[9] = 11.3; vel_prof[9]=24.0;
pos_prof[10] = 14.1; vel_prof[10]=28.0;
pos_prof[11] = 17.3; vel_prof[11]=32.0;
pos_prof[12] = 20.9; vel_prof[12]=36.0;
pos_prof[13] = 24.9; vel_prof[13]=40.0;
*/

//speed version
pos_prof[0]  = 0;  vel_prof[0]=3.0; 
pos_prof[1]  = 0.33;  vel_prof[1]=6.0;  // 10%
pos_prof[2] = 0.9; vel_prof[2]=9.0;
pos_prof[3] = 1.7; vel_prof[3]=12.0;  // 20%
pos_prof[4] = 2.7; vel_prof[4]=14.0;
pos_prof[5] = 3.9; vel_prof[5]=17.0; // 30%
pos_prof[6] = 5.3; vel_prof[6]=20.0;
pos_prof[7] = 6.9; vel_prof[7]=24.0; // 40
pos_prof[8] = 8.9; vel_prof[8]=28.0;
pos_prof[9] = 11.3; vel_prof[9]=32.0;
pos_prof[10] = 14.1; vel_prof[10]=36.0;
pos_prof[11] = 17.3; vel_prof[11]=40.0;
pos_prof[12] = 20.9; vel_prof[12]=44.0;
pos_prof[13] = 24.9; vel_prof[13]=48.0;

move_timeout=10000;
move_rot=move_inches/(4*3.14);  
if(move_inches>=0) {
  direction=1.0;
  spin_dir=1; 
  }
else {
  direction=-1.0;
  spin_dir=-1;
  }
move_abs=fabs(move_inches);


//Brain.Screen.printAt(10, 40, "vehicle move inpersec %.f %.1f",move_inches,move_vel);
for (cnt=0;cnt<=13;cnt++)
  {
  if ((pos_prof[cnt]<=move_mid) && ( vel_prof[cnt]<=move_vel))
    {
    vel_change_pos[cnt]=pos_prof[cnt];  
    left_vel[cnt]=vel_prof[cnt];
    right_vel[cnt]=vel_prof[cnt];

 //   Brain.Screen.printAt(10, 40+30*cnt, "%d %.1f %.1f",cnt, vel_change_pos[cnt],left_vel[cnt]);      
    }
  else 
    {
  //  Brain.Screen.printAt(10, 200, "Break at %d",cnt);      
   break;
    }  
  }


for (revcnt=cnt;revcnt>1;revcnt--)
  {
    vel_change_pos[cnt]=move_abs - pos_prof[revcnt-1];  
    left_vel[cnt]=vel_prof[revcnt-2];
    right_vel[cnt]=vel_prof[revcnt-2];
    cnt++; 
  //  Brain.Screen.printAt(10, 40+30*cnt, "%d %.1f %.1f",cnt, vel_change_pos[cnt],left_vel[cnt]);     
  }
  vel_change_pos[cnt]=0.0;
  left_vel[cnt]=0.0;
  right_vel[cnt]=0.0;
  change_positions= cnt;
timeout=0;
for (tcnt=1;tcnt<=(change_positions-1);tcnt++)
    {
    timeout=timeout + (vel_change_pos[tcnt]-vel_change_pos[tcnt-1])/((left_vel[tcnt]+left_vel[tcnt-1])/2);
    timeout_val[tcnt-1]=timeout;
 //   right_timeout=right_timeout + (vel_change_pos[tcnt]-vel_change_pos[tcnt-1])/((left_vel[tcnt]-left_vel[tcnt-1])/2);
    }

timeout+=0.2; //for last step
move_timeout =( int ) round((1.2*timeout)*1000.0);
move_timeout=5000;

char rec_string[200];

     if( Brain.SDcard.isInserted() ) {
      // create a file with long filename
      ofs.open("vel_data.txt", std::ofstream::app);
      for (write_cnt=0;write_cnt<change_positions;write_cnt++)
        {
          sprintf(rec_string,"POS %.2f,LEFT %.2f,RIGHT %.2f,TOUT%.2f\r\n",   vel_change_pos[write_cnt],left_vel[write_cnt],right_vel[write_cnt],timeout_val[write_cnt]);
          ofs << rec_string;
        }
      ofs.close();

      Brain.Screen.printAt(10, 70, "%d done",sample_cnt);
    }
    else {
      Brain.Screen.printAt(10, 40, "No SD Card");        
    }


Brain.Screen.printAt(10, 80, "Move_timeout %d from timeout %.2f ",move_timeout,timeout);

 /* outmes[0]=0;
  sprintf(outmes,"Cnt %d  ",cnt);
  for (pcnt = 5;pcnt< 8; pcnt++){
       sprintf(outstr,"%d:%.1f:%.1f  ",pcnt, vel_change_pos[pcnt],left_vel[pcnt]);
      strcat(outmes,outstr);
      }
*/
        
if (from_zero)
  {
   leftstart=0.0;  // in degrees
   rightstart=0.0;
  }
else
  {
   leftstart=LB.rotation(rotationUnits::deg);  // in degrees
   rightstart=RB.rotation(rotationUnits::deg);
  }
//lfstart=LF.rotation(rotationUnits::deg);
//rfstart=RF.rotation(rotationUnits::deg);
       

  RF.spin(directionType::fwd,direction*inch2deg*right_vel[0],velocityUnits::dps);
  LF.spin(directionType::fwd,direction*inch2deg*left_vel[0],velocityUnits::dps);
  RB.spin(directionType::fwd,direction*inch2deg*right_vel[0],velocityUnits::dps);
  LB.spin(directionType::fwd,direction*inch2deg*left_vel[0],velocityUnits::dps);
left_vel_val=direction*left_vel[0];
right_vel_val=direction*right_vel[0];         

        
while((move_timeout> 0))
  {
  left_wheel_ang=(LB.rotation(rotationUnits::deg) - leftstart);
  right_wheel_ang= (RB.rotation(rotationUnits::deg) - rightstart);  
  left_position=fabs(deg2inch*(left_wheel_ang));
 //	rDiff = fabs(left_wheel_ang) - fabs(right_wheel_ang); // check for right encoder difference

  wheel_error= left_wheel_ang-right_wheel_ang;
          if (fabs(wheel_error)>15)
              {
	          wheel_factor = .15;
              }
          else if (fabs(wheel_error)>10)
              {
	        wheel_factor = .1;

              }
          else if (fabs(wheel_error)>5)
              {
	        wheel_factor = .05;

              }
          else if (fabs(wheel_error)>2)
              {
	        wheel_factor = .035;

              }
          else if (fabs(wheel_error)>0.5)
              {
	        wheel_factor = .02;

              }
          else    
              {
	        wheel_factor = 0;
	        
              }
  
  
  if (left_position >= move_abs)
    {
          LF.stop(brakeType::hold);
          RF.stop(brakeType::hold);
          LB.stop(brakeType::hold);
          RB.stop(brakeType::hold); 
          left_vel_val=0;
          right_vel_val=0;  
          break;
           
    }
  else if (left_position >= vel_change_pos[next_change_pos])
    { 
          last_position=next_change_pos;
//          right_vel_val=   (right_vel[next_change_pos]+ Kp*wheel_error);

	        rMod = sgn(wheel_error)*left_vel[next_change_pos]*wheel_factor;

          set_driveVelocity(direction*inch2deg*left_vel[next_change_pos],direction*inch2deg*(right_vel[next_change_pos]+rMod));


 
          if (vel_change_pos[next_change_pos]==0.0)
              {
              break;  
              }
          next_change_pos++; 
    }         
    
    else
    {
  	        rMod = direction*sgn(wheel_error)*left_vel[next_change_pos]*wheel_factor;
                

          set_driveVelocity(direction*inch2deg*left_vel[last_position],direction*inch2deg*(right_vel[last_position]+rMod));


    }
  
      
      this_thread::sleep_for(20);
      move_timeout -= 20;
  }

        
        
     
            
        LF.stop(brakeType::hold);
         RF.stop(brakeType::hold);
         LB.stop(brakeType::hold);
         RB.stop(brakeType::hold);
         //wait(40,msec);

}

void vehicle_turn_color(int turn_deg, int turn_vel, int color_detect)
    {
    int turn_timeout;
    float turn_max_sec;
    float turn_rot;
    int number_objects;
    int good_target;
    int lowest_target_ycnt;
    int cnt;
    int center_target_dist_from_mid;
    int ball_ycnt;
    int ball_dist_from_mid;
    int cube_width;
    int cube_height;
    int cube_diag;
    // int ball_dist_from_mid;
    int lowest_target_cube_diag;
    int center_target_index;
    //int screen_middle_x = 316/2;
    // int screen_middle_y = 212/2;
    int screen_middle_y = 106; 
    float lfrot;
    //int taim = 15;
    int dist_from_mid;
    int search_width=200;

    turn_rot=turn_deg;

    /* Drivetrain speed at 50% 3000 rotation took 5.05s */
    /* max speed = 3000 rotations/5 sec*100%/50% = 1188 rotations at 100 %  */
    /* to be safe assume use 900 rotation pers second to calculation timer */
    turn_max_sec= (float) turn_rot/900*(100/(float)turn_vel);
    turn_timeout = (int)(turn_max_sec*1000);  //convert to millisec
    if (turn_timeout <  1000)
    {
        turn_timeout =1000;
    }
    
    LF.resetRotation();
    LB.resetRotation();
    RF.resetRotation();
    RB.resetRotation();

    LB.startRotateFor(turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);
    RB.startRotateFor(-turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);
    LF.startRotateFor(turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);
    RF.startRotateFor(-turn_rot,rotationUnits::deg,turn_vel,velocityUnits::pct);


    while((LB.isSpinning()||RB.isSpinning()||LF.isSpinning()||RF.isSpinning()) &&(turn_timeout> 0))
      {   
      lfrot=fabs(LF.rotation(rotationUnits::deg));

      if (lfrot > (fabs(turn_rot)- 80.0))
        {
        if ( color_detect > 0)
        {
          if ( color_detect == 1) // green;
          {
              Vision1.takeSnapshot(SIG_1);  //compare what it sees against the green flag sigature to see if they match
          } 
          else if (color_detect == 2)  // if the target is red...
          {
              Vision1.takeSnapshot(SIG_2);  //red; compare what it sees against the red ball sigature to see if they match
          }
          else  // otherwise...
          {
              Vision1.takeSnapshot(SIG_3); // blue; compare what it sees to the blue ball sigature to see if they match
          }
      
        number_objects= Vision1.objectCount; //the number of objects the sensor sees is the object count     
           
        for (cnt = 0; cnt < 3;cnt++)
        {
          Brain.Screen.printAt(240,30+30*cnt,"                      ");                           
        }

        good_target=0; //its not a good taget until is decides if it is a good target      
        lowest_target_ycnt=400;
        center_target_dist_from_mid=200;  // initialize with value above maximum of 212/2;
        for (cnt = 0; cnt < number_objects;cnt++) {
          if (Vision1.objects[cnt].exists && Vision1.objects[cnt].width>3)
            {    
              ball_ycnt=Vision1.objects[cnt].centerX;  // X and Y axis are switched because the sensor is rotated 90 degrees
              ball_dist_from_mid = screen_middle_y - Vision1.objects[cnt].centerY;// X and Y axis are switched because the sensor is rotated 90 degrees
              cube_height = Vision1.objects[cnt].height;                       
              cube_width = Vision1.objects[cnt].width;                       
              cube_diag = sqrt(cube_height*cube_height + cube_width*cube_width);
              //Commented to free up screen space
              //Brain.Screen.printAt(240,25+25*cnt,"T%d X%+04d Y%03d W%02d", cnt,ball_ycnt,ball_dist_from_mid,cube_width);
              if (abs(ball_dist_from_mid) < search_width)  // if the object is within the search width...
              {    
                good_target=1; // the it is a good target
              
                if (ball_ycnt < lowest_target_ycnt){  //keeping track oft the lowest object
                    lowest_target_ycnt=ball_ycnt;
                    lowest_target_cube_diag=cube_diag;
                }
                if (abs(ball_dist_from_mid) < abs(center_target_dist_from_mid))
                {
                center_target_index=cnt; // keeps the numbers of the closest object
                center_target_dist_from_mid = ball_dist_from_mid;   // updates that object's distance from the middle as the distance from the center
                }
              }    
            }   
          }  
          if (good_target) //aim side to side
            { 
              //Commented to free up screen space
              //Brain.Screen.printAt(240,100,"Center=%d ID=%02d",center_target_dist_from_mid, center_target_index);
 
              dist_from_mid = center_target_dist_from_mid;
 
              if(( dist_from_mid > -4) && (dist_from_mid < 4)) {
                break;
              }
   
            }  //if (good_target) end side to side

          else
            {
              //Commented to free up screen space
              //Brain.Screen.printAt(240,100,"No Target           ");      
            }
           
        }
    }
      //die
        this_thread::sleep_for(10);
        turn_timeout -= 10;

    }
    LF.stop(brakeType::hold);
    RF.stop(brakeType::hold);
    LB.stop(brakeType::hold);
    RB.stop(brakeType::hold);

    /*
    if(turn_deg<0){
    vehicle_turn(3,15);
    }
    if(turn_deg>0){
    vehicle_turn(-3,15);
    }
    */
 
    LF.stop(brakeType::hold);
    RF.stop(brakeType::hold);
    LB.stop(brakeType::hold);
    RB.stop(brakeType::hold);
    

}

void vehicle_turn_inpersec(double turn_inches, double move_vel)
{  

//double v;
double move_rot; 
int move_timeout;
double direction=1.0;
double lfstart;
double rfstart;
double leftstart,rightstart;
int spin_dir;
double move_abs;
//double max_accel=2.0;  // 2 inches/sec per inch
int change_positions;
float vel_change_pos[64];  
float left_vel[64];
float right_vel[64];
int cnt,revcnt,tcnt;
double last_position=0.0;
int next_change_pos=1;
double left_position=0.0; 
int move_mid=fabs(turn_inches/2);
float vel_prof[32];
float pos_prof[32];
double timeout,right_timeout;
memset(right_vel,0,sizeof(left_vel));
memset(left_vel,0,sizeof(right_vel));

//original
/*
pos_prof[0]  = 0;  vel_prof[0]=2.0; 
pos_prof[1]  = 0.33;  vel_prof[1]=4.0;  // 10%
pos_prof[2] = 0.9; vel_prof[2]=6.0;
pos_prof[3] = 1.7; vel_prof[3]=8.0;  // 20%
pos_prof[4] = 2.7; vel_prof[4]=10.0;
pos_prof[5] = 3.9; vel_prof[5]=12.0; // 30%
pos_prof[6] = 5.3; vel_prof[6]=14.0;
pos_prof[7] = 6.9; vel_prof[7]=16.0; // 40
pos_prof[8] = 8.9; vel_prof[8]=20.0;
pos_prof[9] = 11.3; vel_prof[9]=24.0;
pos_prof[10] = 14.1; vel_prof[10]=28.0;
pos_prof[11] = 17.3; vel_prof[11]=32.0;
pos_prof[12] = 20.9; vel_prof[12]=36.0;
pos_prof[13] = 24.9; vel_prof[13]=40.0;
*/

//speed version
pos_prof[0]  = 0;  vel_prof[0]=3.0; 
pos_prof[1]  = 0.33;  vel_prof[1]=6.0;  // 10%
pos_prof[2] = 0.9; vel_prof[2]=9.0;
pos_prof[3] = 1.7; vel_prof[3]=12.0;  // 20%
pos_prof[4] = 2.7; vel_prof[4]=14.0;
pos_prof[5] = 3.9; vel_prof[5]=17.0; // 30%
pos_prof[6] = 5.3; vel_prof[6]=20.0;
pos_prof[7] = 6.9; vel_prof[7]=24.0; // 40
pos_prof[8] = 8.9; vel_prof[8]=28.0;
pos_prof[9] = 11.3; vel_prof[9]=32.0;
pos_prof[10] = 14.1; vel_prof[10]=36.0;
pos_prof[11] = 17.3; vel_prof[11]=40.0;
pos_prof[12] = 20.9; vel_prof[12]=44.0;
pos_prof[13] = 24.9; vel_prof[13]=48.0;

move_timeout=10000;
move_rot=turn_inches/(4*3.14);  
if(turn_inches>=0) {
  direction=1.0;
  spin_dir=1; 
  }
else {
  direction=-1.0;
  spin_dir=-1;
  }
move_abs=fabs(turn_inches);

//Brain.Screen.printAt(10, 40, "vehicle turn inpersec %.f %.1f",turn_inches,move_vel);
for (cnt=0;cnt<=13;cnt++)
  {
  if ((pos_prof[cnt]<=move_mid) && ( vel_prof[cnt]<=move_vel))
    {
    vel_change_pos[cnt]=pos_prof[cnt];  
    left_vel[cnt]=vel_prof[cnt];
    right_vel[cnt]=-1*vel_prof[cnt];

 //   Brain.Screen.printAt(10, 40+30*cnt, "%d %.1f %.1f",cnt, vel_change_pos[cnt],left_vel[cnt]);      
    }
  else 
    {
  //  Brain.Screen.printAt(10, 200, "Break at %d",cnt);      
   break;
    }  
  }


for (revcnt=cnt;revcnt>1;revcnt--)
  {
    vel_change_pos[cnt]=move_abs - pos_prof[revcnt-1];  
    left_vel[cnt]=vel_prof[revcnt-2];
    right_vel[cnt]=-1*vel_prof[revcnt-2];
    cnt++; 
  //  Brain.Screen.printAt(10, 40+30*cnt, "%d %.1f %.1f",cnt, vel_change_pos[cnt],left_vel[cnt]);     
  }

  vel_change_pos[cnt]=0.0;
  left_vel[cnt]=0.0;
  right_vel[cnt]=0.0;
  change_positions= cnt;

for (tcnt=1;tcnt<=change_positions;tcnt++)
    {
    timeout=timeout + (vel_change_pos[tcnt]-vel_change_pos[tcnt-1])/((left_vel[tcnt]-left_vel[tcnt-1])/2);
    right_timeout=right_timeout + (vel_change_pos[tcnt]-vel_change_pos[tcnt-1])/((left_vel[tcnt]-left_vel[tcnt-1])/2);
    }

move_timeout =( int ) round((1.2*timeout)*1000.0);
move_timeout=5000;

//Brain.Screen.printAt(40, 10, "Move_timeout %d from timeout %.2f ",timeout,right_timeout); 
  //     Brain.Screen.printAt(20,100,"Move_rot %.2f, acs %.2f,dcs%.2f, ma %.2f",move_rot,accel_stop,deccel_start,move_abs);       
        
leftstart=LB.rotation(rotationUnits::deg);  // in degrees
rightstart=RB.rotation(rotationUnits::deg);
lfstart=LF.rotation(rotationUnits::deg);
rfstart=RF.rotation(rotationUnits::deg);
       

  RF.spin(directionType::fwd,direction*inch2deg*right_vel[0],velocityUnits::dps);
  LF.spin(directionType::fwd,direction*inch2deg*left_vel[0],velocityUnits::dps);
  RB.spin(directionType::fwd,direction*inch2deg*right_vel[0],velocityUnits::dps);
  LB.spin(directionType::fwd,direction*inch2deg*left_vel[0],velocityUnits::dps);
left_vel_val=direction*left_vel[0];
right_vel_val=direction*right_vel[0];         
 
        
while((move_timeout> 0))
  {
  left_position=fabs(deg2inch*(LF.rotation(rotationUnits::deg) - lfstart));

  if (left_position >= move_abs)
    {
          LF.stop(brakeType::brake);
          RF.stop(brakeType::brake);
          LB.stop(brakeType::brake);
          RB.stop(brakeType::brake); 
          left_vel_val=0;
          right_vel_val=0;  
          break;
           
    }
  else if (left_position >= vel_change_pos[next_change_pos])
    { 
          last_position=next_change_pos;

  
     
          LF.setVelocity(direction*inch2deg*left_vel[next_change_pos],velocityUnits::dps);
          RF.setVelocity(direction*inch2deg*right_vel[next_change_pos],velocityUnits::dps);
          LB.setVelocity(direction*inch2deg*left_vel[next_change_pos],velocityUnits::dps);
          RB.setVelocity(direction*inch2deg*right_vel[next_change_pos],velocityUnits::dps);
          
          left_vel_val=direction*left_vel[next_change_pos];
          right_vel_val=direction*right_vel[next_change_pos];
          if (vel_change_pos[next_change_pos]==0.0)
              {
              break;  
              } 
          next_change_pos++;

    }
      
      this_thread::sleep_for(20);
      move_timeout -= 20;
  }

  vehicle_turn(3,15);
 
         this_thread::sleep_for(50);       
            
        LF.stop(brakeType::hold);
         RF.stop(brakeType::hold);
         LB.stop(brakeType::hold);
         RB.stop(brakeType::hold);
         
         //wait(40,msec);

}

void vehicle_move_fixedrate(double move_inches, double move_vel)
{  

//double v;
double move_rot; 
int move_timeout;
    //int cnt;
double direction=1.0;
//double lfstart;
//double rfstart;
double leftstart,rightstart;
int spin_dir;
double move_abs;

int change_positions;
    //int ball_limit_reset=0;
    //int ball_limit_once=0;
float vel_change_pos[64];  
float left_vel[64];
float right_vel[64];
int cnt,revcnt,tcnt;
int last_position=0;
int next_change_pos=1;
double left_position=0.0; 

int move_mid=fabs(move_inches/2);
float vel_prof[32];
float pos_prof[32];
double right_wheel_ang,left_wheel_ang;
//double Kp=2;
//double rDiff;
double rMod;
//double sgn_error;
double wheel_factor;
//char outmes[320];
//char outstr[20];

double right_timeout,timeout;

memset(left_vel,0,sizeof(left_vel));
memset(right_vel,0,sizeof(right_vel));



move_timeout=10000;
move_rot=move_inches/(4*3.14);  
if(move_inches>=0) {
  direction=1.0;
  spin_dir=1; 
  }
else {
  direction=-1.0;
  spin_dir=-1;
  }
move_abs=fabs(move_inches);



move_timeout =( int ) round((1.2*move_inches/move_vel)*1000.0)+300;
//move_timeout=5000;


 
        

   leftstart=LB.rotation(rotationUnits::deg);  // in degrees
   rightstart=RB.rotation(rotationUnits::deg);
  
//lfstart=LF.rotation(rotationUnits::deg);
//rfstart=RF.rotation(rotationUnits::deg);
 left_vel[0]=right_vel[0]=fabs(move_vel);     

  RF.spin(directionType::fwd,direction*inch2deg*right_vel[0],velocityUnits::dps);
  LF.spin(directionType::fwd,direction*inch2deg*left_vel[0],velocityUnits::dps);
  RB.spin(directionType::fwd,direction*inch2deg*right_vel[0],velocityUnits::dps);
  LB.spin(directionType::fwd,direction*inch2deg*left_vel[0],velocityUnits::dps);
left_vel_val=direction*left_vel[0];
right_vel_val=direction*right_vel[0];         

        
while((move_timeout> 0))
  {
  left_wheel_ang=(LB.rotation(rotationUnits::deg) - leftstart);
  right_wheel_ang= (RB.rotation(rotationUnits::deg) - rightstart);  
  left_position=fabs(deg2inch*(left_wheel_ang));
 //	rDiff = fabs(left_wheel_ang) - fabs(right_wheel_ang); // check for right encoder difference

  wheel_error= left_wheel_ang-right_wheel_ang;
          if (fabs(wheel_error)>15)
              {
	          wheel_factor = .15;
              }
          else if (fabs(wheel_error)>10)
              {
	        wheel_factor = .1;

              }
          else if (fabs(wheel_error)>5)
              {
	        wheel_factor = .05;

              }
          else if (fabs(wheel_error)>2)
              {
	        wheel_factor = .035;

              }
          else if (fabs(wheel_error)>0.5)
              {
	        wheel_factor = .02;

              }
          else    
              {
	        wheel_factor = 0;
	        
              }
  
  
  if (left_position >= move_abs)
    {
          LF.stop(brakeType::hold);
          RF.stop(brakeType::hold);
          LB.stop(brakeType::hold);
          RB.stop(brakeType::hold); 
          left_vel_val=0;
          right_vel_val=0;  
          break;
           
    }
    else
    {
  	        rMod = direction*sgn(wheel_error)*left_vel[next_change_pos]*wheel_factor;
                

          set_driveVelocity(direction*inch2deg*left_vel[last_position],direction*inch2deg*(right_vel[last_position]+rMod));

    } 
      
      this_thread::sleep_for(20);
      move_timeout -= 20;
  }
            
        LF.stop(brakeType::hold);
         RF.stop(brakeType::hold);
         LB.stop(brakeType::hold);
         RB.stop(brakeType::hold);
         wait(100,msec);

}

void goalCycle (int cycleTimeout, int topShoot, int btmIntake)
  {
    int cycle_timer = 0;
    int flytime_top = 0;
    int flytime_bottom = 0;
    int prevTopCount = 0;
    int prevBtmCount = 0;

    while((topSwitchCount<=topShoot || bottomSwitchCount<=topShoot) && (cycle_timer < cycleTimeout)){
      if(topSwitchCount>prevTopCount){
        Brain.Screen.printAt(0,0+(topSwitchCount*20), "TC: %d, mS: %d", topSwitchCount, cycle_timer);
        Brain.Screen.printAt(0,0+((topSwitchCount+1)*20), "                        ");
        prevTopCount = topSwitchCount;
        }
      if(bottomSwitchCount>prevBtmCount)
        {
        Brain.Screen.printAt(200,0+(bottomSwitchCount*20), "BC: %d, mS: %d", bottomSwitchCount, cycle_timer);
        Brain.Screen.printAt(200,0+((bottomSwitchCount+1)*20), "                        ");
        prevBtmCount = bottomSwitchCount;
        }
      if (topSwitchCount<topShoot){
        UpperIndexer.spin(fwd, 100, percentUnits::pct);
      }
      else if ((topSwitchCount>=topShoot) && (flytime_top<=200)){
        flytime_top+=20;
        if(flytime_top>=200){
          UpperIndexer.stop(brakeType::coast);
        }
      }
      
      if (bottomSwitchCount<btmIntake){
        LowerIndexer.spin(fwd, 100, percentUnits::pct);
        LeftRoller.spin(directionType::fwd, 100, percentUnits::pct);
        RightRoller.spin(directionType::fwd, 100, percentUnits::pct);
      }
      else if ((bottomSwitchCount>=btmIntake) && (flytime_bottom<=0)){
        flytime_bottom+=20;
        if(flytime_bottom>=0){
          LowerIndexer.stop(brakeType::coast);
          LeftRoller.stop(brakeType::coast);
          RightRoller.stop(brakeType::coast);
        }
      }

      if(topSwitchCount==topShoot && bottomSwitchCount==btmIntake && flytime_top>=200 && flytime_bottom>=0){
        LowerIndexer.stop(brakeType::coast);
        LeftRoller.stop(brakeType::coast);
        RightRoller.stop(brakeType::coast);
        UpperIndexer.stop(brakeType::coast);
        break;
      }
      wait(20,msec);
      cycle_timer += 20;
    }
    //LowerIndexer.stop(brakeType::coast);
    //LeftRoller.stop(brakeType::coast);
    //RightRoller.stop(brakeType::coast);
    //UpperIndexer.stop(brakeType::coast);
  }



void autonomous(void) {

//28.66 deg per inch

//int Kp=3;

LF.stop(brakeType::brake);
RF.stop(brakeType::brake);
LB.stop(brakeType::brake);
RB.stop(brakeType::brake);

LF.setStopping(brakeType::coast);
RF.setStopping(brakeType::coast);
LB.setStopping(brakeType::coast);
RB.setStopping(brakeType::coast);
//TwoBar.setStopping(vex::brakeType::brake);
int move_timeout=200;
float vel_change_pos[41];  
float left_vel_curve[41];
float right_vel_curve[41];
int next_change_pos=0;
float left_position;
float last_position;

LF.resetRotation();
RF.resetRotation();
LB.resetRotation();
RB.resetRotation();
// 1200 dps = 200 rpm

task imu_record();


//release hood
UpperIndexer.startRotateFor(90,rotationUnits::deg,100,velocityUnits::pct);
//release deflector
LowerIndexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
LeftRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
RightRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
//wait for deflector to raise
wait(400,msec);
//curve into goal 1

int turn_timeout=15000;

    vel_change_pos[0]= 0.5*1.28;    left_vel_curve[0]= 4;    right_vel_curve[0]=6;
    vel_change_pos[1]= 0.5*1.28;    left_vel_curve[1]= 8;    right_vel_curve[1]=12;
    vel_change_pos[2]= 1.0*1.28;    left_vel_curve[2]= 8;    right_vel_curve[2]=16;
    vel_change_pos[3]= 2.5*1.28;    left_vel_curve[3]= 16;    right_vel_curve[3]=24;
    vel_change_pos[4]= 2.0*1.28;    left_vel_curve[4]= 12;    right_vel_curve[4]=16;
    vel_change_pos[4]= 1.5*1.28;    left_vel_curve[4]= 8;    right_vel_curve[4]=12;
    vel_change_pos[5]= 0.75*1.28;   left_vel_curve[5]= 6;    right_vel_curve[5]=8;
    vel_change_pos[6]= 0;            left_vel_curve[6]= 0;    right_vel_curve[6]=0;
    vel_change_pos[7]= -1;    left_vel_curve[7]= -7;    right_vel_curve[7]=-7;
    vel_change_pos[8]= -1;    left_vel_curve[8]= -12;    right_vel_curve[8]=-12;
    vel_change_pos[9]= -1*1.28;    left_vel_curve[9]= -16;    right_vel_curve[9]=-16;
    vel_change_pos[10]= -2*1.28;    left_vel_curve[10]= -12;    right_vel_curve[10]=-24;
    vel_change_pos[11]= -4*1.28;    left_vel_curve[11]= -18;    right_vel_curve[11]=-30;
    vel_change_pos[12]= -8*1.28;    left_vel_curve[12]= -36;    right_vel_curve[12]=-36;
    vel_change_pos[13]= -6*1.28;    left_vel_curve[13]= -32;    right_vel_curve[13]=-32;
    vel_change_pos[14]= -4;    left_vel_curve[14]= -28;    right_vel_curve[14]=-28;
    vel_change_pos[15]= -3;    left_vel_curve[15]= -24;    right_vel_curve[15]=-24;
    vel_change_pos[16]= -3;    left_vel_curve[16]= -20;    right_vel_curve[16]=-20;
    vel_change_pos[17]= -1.5;    left_vel_curve[17]= -16;    right_vel_curve[17]= -16;
    vel_change_pos[18]= -1.5;    left_vel_curve[18]= -10;    right_vel_curve[18]= -10;
    vel_change_pos[19]= -1;    left_vel_curve[19]= -6;    right_vel_curve[19]= -6;
    vel_change_pos[20]= 0;    left_vel_curve[20]= 0;    right_vel_curve[20]= 0;
    vel_change_pos[21]= 1.0;    left_vel_curve[21]= 6;    right_vel_curve[21]= 6;
    vel_change_pos[22]= 1.5;    left_vel_curve[22]= 10;    right_vel_curve[22]= 10;
    vel_change_pos[23]= 1.5;    left_vel_curve[23]= 16;    right_vel_curve[23]= 16;
    vel_change_pos[24]= 3.0;    left_vel_curve[24]= 20;    right_vel_curve[24]= 20;
    vel_change_pos[25]= 3.0;    left_vel_curve[25]= 24;    right_vel_curve[25]= 24;
    vel_change_pos[26]= 4.0;    left_vel_curve[26]= 28;    right_vel_curve[26]= 28;
    vel_change_pos[27]= 6*1.28;    left_vel_curve[27]= 32;    right_vel_curve[27]= 32;
    vel_change_pos[28]= 9*1.28;    left_vel_curve[28]= 36;    right_vel_curve[28]= 36;
    vel_change_pos[29]= 4*1.28;    left_vel_curve[29]= 30;    right_vel_curve[29]= 18;
    vel_change_pos[30]= 2*1.28;    left_vel_curve[30]= 24;    right_vel_curve[30]= 12;
    vel_change_pos[31]= 1*1.28;    left_vel_curve[31]= 16;    right_vel_curve[31]= 16;
    vel_change_pos[32]= 1.0;    left_vel_curve[32]= 12;    right_vel_curve[32]= 12;
    vel_change_pos[33]= 1.0;    left_vel_curve[33]= 6;    right_vel_curve[33]= 6;
    vel_change_pos[34]= 0;    left_vel_curve[34]= 0;    right_vel_curve[34]= 0;
    vel_change_pos[35]= 0;    left_vel_curve[35]= 0;    right_vel_curve[35]= 0;

    //turn left for 360
    LF.resetRotation();
    RF.resetRotation();
    LB.resetRotation();
    RB.resetRotation();
    // 1200 dps = 200 rpm
 
    last_position=0.0;

    //float turn_max_sec;

    LF.spin(vex::directionType::fwd, inch2deg*left_vel_curve[0]*1.28, vex::velocityUnits::dps);
    RF.spin(vex::directionType::fwd, inch2deg*right_vel_curve[0]*1.28, vex::velocityUnits::dps);
    LB.spin(vex::directionType::fwd, inch2deg*left_vel_curve[0]*1.28, vex::velocityUnits::dps);
    RB.spin(vex::directionType::fwd, inch2deg*right_vel_curve[0]*1.28, vex::velocityUnits::dps);
      while((turn_timeout> 0)){
      left_position=deg2inch*LF.rotation(rotationUnits::deg);
      if (left_position >= (last_position + vel_change_pos[next_change_pos])){
      //if (0){
        if (vel_change_pos[next_change_pos+1] == 0)
        
          {
            RF.stop(brakeType::brake);
            RB.stop(brakeType::brake);
            LF.stop(brakeType::brake);
            LB.stop(brakeType::brake);
            last_position+=vel_change_pos[next_change_pos];
            next_change_pos++; 
          break; 
          }
        
        else
          { 
          last_position+=vel_change_pos[next_change_pos];
          next_change_pos++;  
          LF.setVelocity(inch2deg*left_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          RF.setVelocity(inch2deg*right_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          LB.setVelocity(inch2deg*left_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          RB.setVelocity(inch2deg*right_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          
          
          if (next_change_pos==5){
            UpperIndexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
          }
          
          
          }
      }
      this_thread::sleep_for(10);
      turn_timeout -= 10;
      }

    RF.stop(brakeType::brake);
    RB.stop(brakeType::brake);
    LF.stop(brakeType::brake);
    LB.stop(brakeType::brake);

LF.resetRotation();
RF.resetRotation();
LB.resetRotation();
RB.resetRotation();

LeftRoller.spin(fwd, 100, percentUnits::pct);
RightRoller.spin(fwd, 100, percentUnits::pct);
LowerIndexer.spin(fwd, 100, percentUnits::pct);


goalCycle(1600,3,3);

wait(300,msec);

turn_timeout=15000;
//turn left for 360
    LF.resetRotation();
    RF.resetRotation();
    LB.resetRotation();
    RB.resetRotation();
    // 1200 dps = 200 rpm
 
    last_position=0.0;

    //float turn_max_sec;

    LF.spin(vex::directionType::fwd, inch2deg*left_vel_curve[7]*1.28, vex::velocityUnits::dps);
    RF.spin(vex::directionType::fwd, inch2deg*right_vel_curve[7]*1.28, vex::velocityUnits::dps);
    LB.spin(vex::directionType::fwd, inch2deg*left_vel_curve[7]*1.28, vex::velocityUnits::dps);
    RB.spin(vex::directionType::fwd, inch2deg*right_vel_curve[7]*1.28, vex::velocityUnits::dps);

    next_change_pos = 7;
      while((turn_timeout> 0)){
      left_position=deg2inch*LF.rotation(rotationUnits::deg);
      if (left_position <= (last_position + vel_change_pos[next_change_pos])){
      //if (0){
        if (vel_change_pos[next_change_pos+1] == 0)
        
          {
            RF.stop(brakeType::brake);
            RB.stop(brakeType::brake);
            LF.stop(brakeType::brake);
            LB.stop(brakeType::brake);
            last_position+=vel_change_pos[next_change_pos];
            next_change_pos++; 
          break; 
          }
        
        else
          { 
          last_position+=vel_change_pos[next_change_pos];
          next_change_pos++;  
          LF.setVelocity(inch2deg*left_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          RF.setVelocity(inch2deg*right_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          LB.setVelocity(inch2deg*left_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          RB.setVelocity(inch2deg*right_vel_curve[next_change_pos]*1.28,velocityUnits::dps);
          
          
          if (next_change_pos==12){
            LowerIndexer.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
            LeftRoller.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
            RightRoller.spin(vex::directionType::rev, 100, vex::velocityUnits::pct); 
          }
          if (next_change_pos==18){
            
            LeftRoller.stop(vex::brakeType::coast); 
            RightRoller.stop(vex::brakeType::coast);
            LowerIndexer.stop(vex::brakeType::coast);
          }
          
          }
      }
      this_thread::sleep_for(10);
      turn_timeout -= 10;
      }

    LF.resetRotation();
    RF.resetRotation();
    LB.resetRotation();
    RB.resetRotation();



    vehicle_turn_color(-200, 15, 1);

    LowerIndexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    LeftRoller.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
    RightRoller.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct); 

    auton_drive_task = task(driveTask);
    auton_turn_task = task(turnTask); 

    drive(500);

    RF.stop(brakeType::coast);
    RB.stop(brakeType::coast);
    LF.stop(brakeType::coast);
    LB.stop(brakeType::coast);

    topSwitchCount = 0;
    bottomSwitchCount = 0;

    goalCycle(800, 1, 2);
    wait(500,msec);

    LowerIndexer.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);
    LeftRoller.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);
    RightRoller.spin(vex::directionType::rev, 10, vex::velocityUnits::pct); 

    drive(-300);

    turn(-100);
    auton_turn_task.stop();
    auton_drive_task.stop();
    //vehicle_turn_inpersec(6.0, 20);

    LeftRoller.stop(vex::brakeType::coast); 
    RightRoller.stop(vex::brakeType::coast);
    LowerIndexer.stop(vex::brakeType::coast);

    auton_drive_task = task(driveTask);
    
    drive(750);

    auton_drive_task.stop();


    vehicle_turn_color(114, 15, 1);

    LowerIndexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    LeftRoller.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
    RightRoller.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct); 

    auton_drive_task = task(driveTask);
    
    drive(590);
    RF.stop(brakeType::coast);
    RB.stop(brakeType::coast);
    LF.stop(brakeType::coast);
    LB.stop(brakeType::coast);

    topSwitchCount = 0;
    bottomSwitchCount = 0;
    driveAsync(-15);

    goalCycle(800, 2, 3);

    //wait(200,msec);
    driveAsync(-500);
    LeftRoller.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    RightRoller.spin(vex::directionType::rev, 50, vex::velocityUnits::pct); 

}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

int rawSpeedLeft;
int rawSpeedRight;

float gyroVal;
float gyroCorrect;
float lastCorrect = 0;
float leftVelLast = 0;
float rightVelLast = 0;
//double limitSpeedRight;
//double limitSpeedLeft;
tilt_lock=0;
move_lock=0;

LF.setStopping(brakeType::coast);
RF.setStopping(brakeType::coast);
LB.setStopping(brakeType::coast);
RB.setStopping(brakeType::coast);

//imu_write_task = vex::task( imu_write);
auton_drive_task.stop();
auton_turn_task.stop();

Brain.Screen.clearScreen();

Brain.Screen.drawImageFromFile("DaBabyScreen.png", 0, 0);
//auton_drive_task.interrupt();
//imu_sample_driver_task   = vex::task( imu_sample_driver);

  // User control code here, inside the loop
  while (1) {

    float tempRF = RF.temperature(temperatureUnits::fahrenheit);
    float tempRB = RB.temperature(temperatureUnits::fahrenheit);
    float tempLF = LF.temperature(temperatureUnits::fahrenheit);
    float tempLB = LB.temperature(temperatureUnits::fahrenheit);
    //Brain.Screen.printAt(0,220, "RF: %.1f, RB: %.1f, LF: %.1f, LB: %.1f", tempRF, tempRB, tempLF, tempLB);
    

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

//Roller Control
    if (!tilt_lock)
    if (!move_lock)
      {
        if(Controller1.ButtonL1.pressing()  ) { //If the A button is pressed...
            //...Spin the flipper motor forward.
            LeftRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
            RightRoller.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else if(Controller1.ButtonL2.pressing()) { //If the Y button is pressed...
            //...Spin the flipper motor backward.
            LeftRoller.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
            RightRoller.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
        }
        else { //If the A or Y button are not pressed...        
            //...Stop the flipper motor.
            LeftRoller.stop(vex::brakeType::coast); 
            RightRoller.stop(vex::brakeType::coast);
        }
  }
//Upper Rollers - Tilter Motor
    if (!tilt_lock)
    if (!move_lock)
      {
        if(Controller1.ButtonR1.pressing()  ) { //If the A button is pressed...
            //...Spin the flipper motor forward.
            UpperIndexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else if(Controller1.ButtonR2.pressing()  ) { //If the A button is pressed...
            //...Spin the flipper motor forward.
            UpperIndexer.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
        }
        else { //If the A or Y button are not pressed...        
            //...Stop the flipper motor.
            UpperIndexer.stop(vex::brakeType::coast); 
        }
  }

//Lower Roller - Two Bar
    if (!tilt_lock)
    if (!move_lock)
      {
        if(Controller1.ButtonL1.pressing()  ) { //If the A button is pressed...
            //...Spin the flipper motor forward.
            LowerIndexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else if(Controller1.ButtonL2.pressing()) { //If the Y button is pressed...
            //...Spin the flipper motor backward.
            LowerIndexer.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        else if(Controller1.ButtonUp.pressing()  ) { //If the A button is pressed...
            //...Spin the flipper motor forward.
            LowerIndexer.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        }
        else if(Controller1.ButtonLeft.pressing()  ) { //If the A button is pressed...
            //...Spin the flipper motor forward.
            LowerIndexer.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        else { //If the A or Y button are not pressed...        
            //...Stop the flipper motor.
            LowerIndexer.stop(vex::brakeType::coast); 
        }
  }

//Drive Control
  float speedMultiplier = 1;
  float minSpinRate = 10.0;
  float kCrazyNumber = 0.0025;
  float spinctrl =2.0;

  //rawSpeedLeft = speedMultiplier * Controller1.Axis3.value();
  //rawSpeedRight = speedMultiplier * Controller1.Axis2.value();
  rawSpeedLeft = (float)Controller1.Axis3.value()/1.27;
  rawSpeedRight = (float)Controller1.Axis2.value()/1.27;
  rawJoyLeft = (float)Controller1.Axis3.value()/1.27;
  rawJoyRight = (float)Controller1.Axis2.value()/1.27;
  //Using non-linear speed control to regulate the speed of the robot
  /*
  if (((fabs(rawJoyLeft) < 30) || (fabs(rawJoyRight) < 30)) && ((rawJoyLeft>0) && (rawJoyRight>0)))  {
    if (rawSpeedLeft > spinctrl*rawSpeedRight)
      {
      rawSpeedLeft = spinctrl*rawSpeedRight;
      }
    else if (rawSpeedRight > spinctrl*rawSpeedLeft)
      {
        rawSpeedRight = spinctrl*rawSpeedLeft;
      }  
  }

  */
  //Slew rate that is proportional to the amount of difference in the joysticks
  if ((fabs(rawSpeedLeft - leftVelLast)) > 30)
  {
    rawSpeedLeft = leftVelLast + ((rawSpeedLeft-leftVelLast)*0.3);
  }

  
  if ((fabs(rawSpeedRight - rightVelLast)) > 30)
  {
    rawSpeedRight = rightVelLast + ((rawSpeedRight-rightVelLast)*0.3);
  }

  leftVelLast = rawSpeedLeft;
  rightVelLast = rawSpeedRight;
  //Using non-linear speed control to regulate the speed of the robot
  /*if ((fabs(rawJoyLeft) > 100) && (fabs(rawJoyRight) > 100)){
    rawSpeedLeft = (57*(rawJoyLeft - 100) / 27) + 50;
    rawSpeedRight = (57*(rawJoyRight - 100) / 27) + 50;
  }
  else {
    rawSpeedLeft = 0.7*rawJoyLeft;
    rawSpeedRight = 0.7*rawJoyRight;
  }
  */


  //Uses Gyro to control spinouts.
  /*
  gyroVal = IMU.gyroRate(axisType::zaxis, velocityUnits::dps);
  

  if (fabs(gyroVal) > minSpinRate)
  {
    Brain.Screen.printAt(20,20, "Gyro Val: %f    ", gyroVal);
    gyroCorrect = 1.0 -(fabs(gyroVal)-minSpinRate) * kCrazyNumber;

    //both positive, decrease faster side
    if (((rawSpeedRight > 0 ) && (rawSpeedLeft > 0)) || ((rawSpeedRight < 0) &&(rawSpeedLeft < 0)))
    {
        // =both
        if (abs(rawSpeedRight) > abs(rawSpeedLeft))
        {
          rawSpeedRight = (int) (gyroCorrect*(float) rawSpeedRight);
          if (gyroCorrect > lastCorrect){
            Brain.Screen.printAt(20,60, "Decrease: %f    ", gyroCorrect);
          }
          
        }
        else
        {
          rawSpeedLeft = (int) (gyroCorrect*(float) rawSpeedLeft);
        }
    }
  }
  else
  {
    gyroCorrect = 1.0 - (fabs(gyroVal)-minSpinRate)*kCrazyNumber;
    rawSpeedRight = (int) (gyroCorrect*(float) rawSpeedRight);
    rawSpeedLeft = (int) (gyroCorrect*(float) rawSpeedLeft);
  }
  lastCorrect = gyroCorrect;
  */

      //rawSpeedLeft = expDrive(Controller1.Axis3.value(),cDriveExp,cJoyDead,cMotorMin);
      //rawSpeedRight = expDrive(Controller1.Axis2.value(),cDriveExp,cJoyDead,cMotorMin);
      //Set the left and right motor to spin forward using the controller Axis values as the velocity value.

 
     /*  if (fabs(IMU.gyroRate(axisType::zaxis, velocityUnits::dps)) > 20.0)
          {
            if (abs(rawSpeedRight) > abs(rawSpeedLeft))
                {
                limitSpeedRight=(int )RF.velocity(velocityUnits::pct);
                if (fabs(limitSpeedRight) < fabs(rawSpeedRight))
                  {
                  rawSpeedRight = limitSpeedRight; 
                  }
                }
            else 
              {
                 limitSpeedLeft=(int )LF.velocity(velocityUnits::pct);
                if (fabs(limitSpeedLeft) < fabs(rawSpeedRight))
                  {
                  rawSpeedLeft = limitSpeedLeft; 
                  }              }    


          }    
          */
      if (!tilt_lock)
        {
        LB.spin(vex::directionType::fwd, rawSpeedLeft, vex::velocityUnits::pct);
        RB.spin(vex::directionType::fwd, rawSpeedRight, vex::velocityUnits::pct);
        LF.spin(vex::directionType::fwd, rawSpeedLeft, vex::velocityUnits::pct);
        RF.spin(vex::directionType::fwd, rawSpeedRight, vex::velocityUnits::pct);
        }

    //Brain.Screen.printAt(20,120, "Top Cnt: %d, Bot Cnt: %d, Fnt Cnt: %d", topSwitchCount,bottomSwitchCount, frontSwitchCount);
    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  TopSwitch.pressed(TopSwitchIsPressed);
  BottomSwitch.pressed(BottomSwitchIsPressed);
  FrontSwitch.pressed(FrontSwitchIsPressed);
  TopSwitch.released(TopSwitchIsReleased);
  BottomSwitch.released(BottomSwitchIsReleased);

  //Controller1.ButtonY.pressed(tilt_profile_push);
 // Controller1.ButtonA.pressed(deploy_tray);
  //Controller1.ButtonA.pressed(deploy_tray);
  //Controller1.ButtonUp.pressed(liftlow);
  //Controller1.ButtonDown.pressed(liftdown);
  //Controller1.ButtonLeft.pressed(vehicle_scurve_left_fast);
  //Controller1.ButtonRight.pressed(tilt_abort);
  //Controller1.ButtonLeft.pressed(change_push);
  
  // Run the pre-autonomous function.
  pre_auton();
  imu_sample_task  = vex::task( imu_sample );

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
