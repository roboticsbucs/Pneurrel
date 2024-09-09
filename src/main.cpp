
#include "vex.h"
#include "side.h"
#include "speed_mode.h"
#include "helper.h"

/*
 *
 * BUTTON DOCS:
 *
 * Left Joystick - left tank drive
 * Right Joystick - right tank drive
 *
 * L1 - toggle puncher
 * L2 - rotate puncher one rotation
 * R1 - raise/stop lift
 * R2 - lower/stop lift
 * Up - toggle lift lock
 * Down - toggle wing
 * Left -
 * Right - move lift to optimal punching position
 * A - swap front of robot
 * B - play bee movie script on controller
 * X - swap speed mode
 * Y -
 *
 */

// drive train motors
vex::motor frontLeft{PORT12, PORT};
vex::motor backLeft{PORT10, PORT};
vex::motor frontRight{PORT2, STARBOARD};
vex::motor backRight{PORT1, STARBOARD};
vex::motor_group leftDrive{frontLeft, backLeft};
vex::motor_group rightDrive{frontRight, backRight};

// drive train config variables
Side frontSide = FORWARD;
SpeedMode speedMode = FULL;
bool fullSpeed = true;
constexpr double drivePctFull = 1.0;
constexpr double drivePctSlow = 0.5;

// pneumatics section

vex::pneumatics lock(TriportA);
bool isLocked = false;

void toggleLock()
{
  isLocked = !isLocked;
  lock.set(isLocked);
}

vex::pneumatics wing(TriportB);
bool isWingOut = false;

void toggleWing()
{
  isWingOut = !isWingOut;
  wing.set(isWingOut);
}

// end pneumatics section

// lift section

vex::motor leftLift(PORT6, STARBOARD);
vex::motor rightLift(PORT7, PORT);
vex::motor_group lift(leftLift, rightLift);
double liftSpeedPct = 60;

void liftUp() // run on R1
{
  lift.spin(directionType::fwd, liftSpeedPct, velocityUnits::pct);
}

void liftDown() // run on R2
{
  lift.spin(directionType::rev, liftSpeedPct, velocityUnits::pct);
}

void stopLift() // run on each released
{
  lift.stop();
}

double initialLiftPosition;
double liftPunchPosition;
double liftTouchPosition;

void liftToPunchPosition()
{
  lift.spinToPosition(liftPunchPosition, rotationUnits::deg);
}

void liftToTouchPosition()
{
  lift.spinToPosition(liftTouchPosition, rotationUnits::deg);
}

// end lift section

// puncher section

vex::motor puncher(PORT8, PORT);
double puncherPct = 60;
double puncherFullDegrees = 750;
bool puncherOn = false;

void togglePuncher()
{
  if (puncherOn)
    puncher.stop(); // already on, stop
  else
    spinMotor(puncher, puncherPct); // turn on
  puncherOn = !puncherOn;           // swap state
}

void rotatePuncher()
{
  if (puncherOn)
    puncherOn = false;                                   // will be off after use
  spinMotorFor(puncher, puncherFullDegrees, puncherPct); // rotate 360
}

// end puncher section

void tankDrive()
{
  // select the drive train speed percentage based on the speed mode
  double drivePct;
  switch (speedMode)
  {
  case FULL:
    drivePct = drivePctFull;
    break;
  case SLOW:
    drivePct = drivePctSlow;
    break;
  default:
    drivePct = 0;
  }

  // assign percentages to hardware sides
  double leftDrivePct;
  double rightDrivePct;
  switch (frontSide)
  {
  case FORWARD:
    leftDrivePct = drivePct * Controller.Axis3.position();
    rightDrivePct = drivePct * Controller.Axis2.position();
    break;
  case AFT:
    leftDrivePct = drivePct * Controller.Axis2.position();
    rightDrivePct = drivePct * Controller.Axis3.position();
    break;
  default:
    leftDrivePct = 0;
    rightDrivePct = 0;
  }

  // get hardware motor direction from virtual front of robot and spin
  directionType dirType = frontSide == FORWARD ? directionType::rev : directionType::fwd;
  leftDrive.spin(dirType, leftDrivePct, velocityUnits::pct);
  rightDrive.spin(dirType, rightDrivePct, velocityUnits::pct);
}

void pre_auton(void)
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  initialLiftPosition = lift.position(rotationUnits::deg);
  liftPunchPosition = initialLiftPosition + 500;
  liftTouchPosition = initialLiftPosition + 1500;
}

void autonomous(void)
{
  // TODO: auton code here

  // slight turn left
  leftDrive.spinFor(directionType::rev, 4, rotationUnits::rev, 100, velocityUnits::pct, false);
  rightDrive.spinFor(directionType::rev, 4, rotationUnits::rev, 70, velocityUnits::pct, true);

  // back and forward a bunch

  leftDrive.spinFor(directionType::fwd, 0.75, rotationUnits::rev, 50, velocityUnits::pct, false);
  rightDrive.spinFor(directionType::fwd, 0.75, rotationUnits::rev, 50, velocityUnits::pct, true);

  leftDrive.spinFor(directionType::rev, 1.25, rotationUnits::rev, 50, velocityUnits::pct, false);
  rightDrive.spinFor(directionType::rev, 1.25, rotationUnits::rev, 50, velocityUnits::pct, true);

  leftDrive.spinFor(directionType::fwd, 0.75, rotationUnits::rev, 50, velocityUnits::pct, false);
  rightDrive.spinFor(directionType::fwd, 0.75, rotationUnits::rev, 50, velocityUnits::pct, true);

  leftDrive.spinFor(directionType::rev, 1.25, rotationUnits::rev, 50, velocityUnits::pct, false);
  rightDrive.spinFor(directionType::rev, 1.25, rotationUnits::rev, 50, velocityUnits::pct, true);

  // end on back so no touchy
  leftDrive.spinFor(directionType::fwd, 0.75, rotationUnits::rev, 50, velocityUnits::pct, false);
  rightDrive.spinFor(directionType::fwd, 0.75, rotationUnits::rev, 50, velocityUnits::pct, true);
}

void drivercontrol(void)
{
  // drive train
  // swap front side
  Controller.ButtonA.pressed([]()
                             { frontSide = frontSide == FORWARD ? AFT : FORWARD; });
  // swap speed mode
  Controller.ButtonX.pressed([]()
                             { speedMode = speedMode == fullSpeed ? SLOW : FULL; });

  // pneumatics
  Controller.ButtonUp.pressed(toggleLock);
  Controller.ButtonDown.pressed(toggleWing);

  // lift
  Controller.ButtonR1.pressed(liftUp);
  Controller.ButtonR1.released(stopLift);
  Controller.ButtonR2.pressed(liftDown);
  Controller.ButtonR2.released(stopLift);
  // Controller.ButtonRight.pressed(liftToPunchPosition);

  // puncher
  Controller.ButtonL1.pressed(togglePuncher);
  Controller.ButtonL2.pressed(rotatePuncher);

  // beeeeeeeee
  Controller.ButtonB.pressed(beeee);

  while (true)
  {
    tankDrive();
    wait(20, msec);
  }
}

int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
    wait(100, msec);
}
