#include "vex.h"

using namespace vex;

competition Competition;
controller Controller1;

motor rDrive(PORT1, ratio18_1);
motor lDrive(PORT2, ratio18_1, true);
motor sDrive(PORT3, ratio36_1, true);

const float MOTOR_ACCEL_LIMIT = 8;

int s_lastL = 0;
int s_lastR = 0;

bool isFast = false;

const float WHEEL_CIRCUMFERENCE = 31.9185812596;


// Groups go here using motor_group



void setSideSpeeds(int lSpeed, int rSpeed, int strafeSpeed)
{
    if ((strafeSpeed - s_lastL) > MOTOR_ACCEL_LIMIT)
        strafeSpeed = s_lastL + MOTOR_ACCEL_LIMIT;
    if ((strafeSpeed - s_lastL) < -MOTOR_ACCEL_LIMIT)
        strafeSpeed = s_lastL - MOTOR_ACCEL_LIMIT;

    s_lastL = strafeSpeed;

    if (lSpeed == 0)
        lDrive.stop(brakeType::brake);
    else
        lDrive.spin(directionType::fwd, lSpeed, velocityUnits::pct);
    if (rSpeed == 0)
        rDrive.stop(brakeType::brake);
    else
        rDrive.spin(directionType::fwd, rSpeed, velocityUnits::pct);
    if (strafeSpeed == 0)
        sDrive.stop(brakeType::brake);
    else 
        sDrive.spin(directionType::fwd, strafeSpeed, velocityUnits::pct);
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
void WriteInfo()
{
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Fast Turning: ");
  Controller1.Screen.print(isFast ? "Enabled" : "Disabled");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Battery: ");
  Controller1.Screen.print(Brain.Battery.capacity());
  Controller1.Screen.print("%%");
}
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    int strafeSpeed = 0;
    if(Controller1.ButtonL1.pressing())
      strafeSpeed = -100;
    else if(Controller1.ButtonR1.pressing())
      strafeSpeed = 100;
    
    float lSpeed = 0;
    float rSpeed = 0;

    
    if (isFast)
    {
      lSpeed = Controller1.Axis2.value();
      rSpeed = Controller1.Axis3.value();
    }
    else
    {
      lSpeed = Controller1.Axis2.value() + Controller1.Axis3.value()/2;
      rSpeed = Controller1.Axis3.value() + Controller1.Axis2.value()/2;
    }
   

    if (Controller1.Axis2.value() != 0 && Controller1.Axis3.value() == 0)
      rSpeed = 0;
    else if (Controller1.Axis3.value() != 0 && Controller1.Axis2.value() == 0)
      lSpeed = 0;

    setSideSpeeds(lSpeed, rSpeed, strafeSpeed);
    wait(20, msec);
    
  }
}

void ToggleSpeed()
{
  isFast = !isFast;      
  WriteInfo();
}
int main() {
  WriteInfo();
  Controller1.ButtonY.pressed(ToggleSpeed);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}