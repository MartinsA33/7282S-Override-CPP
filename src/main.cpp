/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       akinb                                                     */
/*    Created:      9/17/2026, 3:08:57 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

brain Brain= brain();
controller Con = controller();

//drive train motors
motor motorL1 = motor(PORT1,ratio6_1, true);
motor motorL2 = motor(PORT2,ratio6_1, true);
motor motorL3 = motor(PORT3,ratio6_1, true);
motor motorR1 = motor(PORT10,ratio6_1, false);
motor motorR2 = motor(PORT9,ratio6_1, false);
motor motorR3 = motor(PORT8,ratio6_1, false);

motor_group drivetrainRight= motor_group(motorR1, motorR2, motorR3);
motor_group drivetrainLeft = motor_group(motorL1, motorL2, motorL3);

//sensors
inertial IMU = inertial(PORT20);
rotation TW = rotation(PORT6);
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
// Calculates the shortest path to turn.
float  getAlpha(float targetHeading){
   float alpha0= targetHeading - IMU.heading();
   float alpha1 = alpha0 + 360;
   float alpha_min = std::min(abs(alpha0), abs(alpha1));

   if( alpha_min == abs(alpha0) )
   {
     return alpha0;
   }
    else{
      return alpha1;
    }
}

//Make the robot turn to theset angle using PID controls
void turnTo (float targetDegrees){

   // proportionalCoefficient = 0.39
  // new kp
    float proportionalCoefficient = 0.39;

    float minSpeed = 2;

    float t = 3;

         while(true){

           float errorDegrees = getAlpha(targetDegrees);

           // print to Screen for debugging

           Brain.Screen.setCursor(1, 0);
           Brain.Screen.clearLine(1, color ::red);
           Brain.Screen.clearLine(2,  color :: green);
           Brain.Screen.clearLine(3, color :: blue);

           float imuHeading = IMU.heading();
          
          Brain.Screen.print("Heading = %.2f", imuHeading);
           Brain.Screen.newLine();
           Brain.Screen.print("Error = %.2f", errorDegrees);

           float speed = proportionalCoefficient * errorDegrees;

           // flips the sign of the minimum speed based on the direction of the error
           float errorSign = errorDegrees / abs(errorDegrees);

           float minSpeedCorrected = errorSign * minSpeed;

           drivetrainLeft.setVelocity(minSpeedCorrected + speed, percent);
           drivetrainRight.setVelocity(-(minSpeedCorrected + speed), percent);

           if (abs(errorDegrees)< 0.5){
            //if error is small
            t += -1;
           }
         else {
           t = 3;
         }
                 if (t == 0)
             {
               // stop motors
               drivetrainLeft.setVelocity(0, percent);
               drivetrainRight.setVelocity(0, percent);
               return;
             }

 wait(20, msec);
}
}

float robotY = 0.0;
float prevAngle = 0.0;
// Odometry for front and back movement
void updateOdom(){
//tracks current odom position
  float currentAngle = TW.position(degrees);

  //Calculate change in angles for each loop
  float D_Angle = currentAngle - prevAngle;

  //saves the current position for next iteration
  prevAngle = currentAngle;

  // calcuates the disance the robot moves and saves it to robotY
  float Distance = D_Angle * ((M_PI * 2) / 360);
  robotY += Distance;
}

float distanceToDegrees(float targetDistance){
  float D_distance = targetDistance - TW.position(deg);
  float travelDegrees = ( D_distance / ((M_PI * 2) / 360));
  return travelDegrees;
}

void driveTo(float position){
// proportional value for driving
float proportionalDriving = 3.0;
//minimum driving speed
float minDrivingSpeed = 5;

int t = 3;
while(true){
  
float drivingError = distanceToDegrees(position);

float drivingSpeed = proportionalDriving * minDrivingSpeed;

drivetrainLeft.setVelocity(minDrivingSpeed + drivingSpeed, percent);
drivetrainRight.setVelocity(minDrivingSpeed + drivingSpeed, percent);

//stops it after 3 ocillations at which it is with in the error range.
if (abs(drivingError)<1){
  t -= 1;
}else{
  t = 3;
}

//stops motion
  if (t==0){
    TW.resetPosition();
    drivetrainLeft.setVelocity(0, percent);
    drivetrainRight.setVelocity(0, percent);
    return; // ends loop and funtion
  }
 }
}

void autonomous(void){

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
  // User control code here, inside the loop
  while (1) {
    //right moters speed
float rms=Con.Axis2.position(percent);
//left motersspeed
float lms=Con.Axis3.position(percent);

drivetrainLeft.spin(forward, rms, percent);
drivetrainRight.spin(forward, lms, percent);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  TW.resetPosition();

  IMU.calibrate();
  while (IMU.isCalibrating() == true)
  {
    wait(20, msec);
    IMU.setHeading(0, degrees);
  }
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
