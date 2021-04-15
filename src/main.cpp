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

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
controller Controller1;

motor lDrive(PORT1, ratio18_1);
motor rDrive(PORT2, ratio18_1, true);
motor sDrive(PORT3, ratio18_1, true);



const float WHEEL_CIRCUMFERENCE = 31.9185812596;



// Groups go here using motor_group

void MoveMotors(float lInput, float rInput, float sInput)
{
  lDrive.spin(fwd, lInput, pct);
  rDrive.spin(fwd, rInput, pct);
  sDrive.spin(fwd, sInput, pct);
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
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
//Drive the robot a certain distance

void DriveDistance(int dist, float maxTime)
{
  lDrive.resetPosition();
  rDrive.resetPosition();

  //Constant Tuning Values
  const float Kp = 1;
  const float Kd = 0;
  const float Ki = 0;

  float rotationGoal = (dist / WHEEL_CIRCUMFERENCE) * 360;

  const float maxSpeed = 100;
  const float accelTime = 500;

  float distError = 0;
  float integral = 0;
  float derivative = 0;
  float lastError = 0;

  float motorSpeed = 0;
  
  float doneTime = 0;
  while(maxTime > doneTime / 1000)
  {
    distError = rotationGoal - lDrive.rotation(deg);

    integral += distError;

    if(distError > 200 || distError < -200)
    {
      integral  = 0;
    }

    derivative = distError - lastError;

    lastError = distError;

    motorSpeed = Kp * distError + Ki * integral + Kd * derivative;
  }



}
void autonomous(void) {
  DriveDistance(50, 5);
}


void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    int strafeSpeed = 0;
    if(Controller1.ButtonL1.pressing())
      strafeSpeed = -100;
    else if(Controller1.ButtonR1.pressing())
      strafeSpeed = 100;
    MoveMotors(Controller1.Axis3.value(), Controller1.Axis2.value(), strafeSpeed);


    //else
      //Arm.stop(hold);  

    //if(Controller1.ButtonR1.pressing())
      //Claw.spin(fwd, ClawSpeed, pct);

    //else if(Controller1.ButtonR2.pressing())
      //Claw.spin(fwd, -ClawSpeed, pct);

    //else
      //Claw.stop(hold);
    

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

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
