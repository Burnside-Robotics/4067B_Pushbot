#include "vex.h"

using namespace vex;

competition Competition;
controller Controller1;

motor rDrive(PORT1, ratio18_1);
motor lDrive(PORT2, ratio18_1, true);
motor sDrive(PORT3, ratio18_1, true);

void MoveMotors(float lInput, float rInput);


const float WHEEL_CIRCUMFERENCE = 31.9185812596;


// Groups go here using motor_group

void MoveMotors(float lInput, float rInput, float sInput)
{
  lDrive.spin(fwd, lInput, pct);
  rDrive.spin(fwd, rInput, pct);
  sDrive.spin(fwd, sInput, pct);
}


void pre_auton(void) {
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  DriveDistance(50, 5);
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
    int strafeSpeed = 0;
    if(Controller1.ButtonL1.pressing())
      strafeSpeed = -100;
    else if(Controller1.ButtonR1.pressing())
      strafeSpeed = 100;
    
    float lSpeed = Controller1.Axis2.value() + Controller1.Axis3.value()/2;
    float rSpeed = Controller1.Axis3.value() + Controller1.Axis2.value()/2;

    MoveMotors(lSpeed, rSpeed, strafeSpeed);
  
      

  


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