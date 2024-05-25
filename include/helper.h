#pragma once

#include "vex.h"

using namespace vex;

// helper method to start a spin, in helper.cpp
// need to terminate manually elsewhere
void spinMotor(motor&, double speedPct, bool dir = true);

// spin for a specified number of degrees
void spinMotorFor(motor&, double degrees, double speedPct);




































// beeee
void beeee();