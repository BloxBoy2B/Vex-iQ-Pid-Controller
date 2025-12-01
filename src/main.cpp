#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START IQ MACROS
#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
inertial BrainInertial = inertial();
motor Motor = motor(PORT1, false);

// generating and setting random seed
void initializeRandomSeed()
{
  wait(100, msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
      xAxis + yAxis + zAxis);
  // Set the seed
  srand(seed);
}

void vexcodeInit()
{

  // Initializing random seed.
  BrainInertial.calibrate();
  waitUntil(!BrainInertial.isCalibrating());
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// "when started" hat block
int whenStarted1()
{
  return 0;
}

#include "vex.h"

void MotorSpinUp(double targetRpm, double timeout, double spinForDuration)
{
  float kp = 0.5;
  float kd = 0.2;
  float ki = 0.1;

  float rpmError = 0;
  float intergral = 0;
  float previousError = 0;

  double threashold = 10;
  
  timer mytimer;

  while (true)
  {
    rpmError = targetRpm - Motor.velocity(rpm);
    intergral = intergral + rpmError;

    if (intergral > 400)
      intergral = 400;
    if (intergral < -400)
      intergral = -400;

    if (fabs(rpmError) < 2)
      intergral = 0;

    float derivitive = rpmError - previousError;
    previousError = rpmError;

    float power = (kp * rpmError) + (ki * intergral) + (kd * derivitive);

    if (power > 100)
      power = 100;
    if (power < -100)
      power = -100;

    Motor.spin(forward, power, percent);

    if (fabs(rpmError) < threashold && mytimer.time(msec) >= spinForDuration|| mytimer.time(msec) >= timeout)
    {
      Motor.stop(coast);
      break;
    }

    this_thread::sleep_for(20);
  }
}

int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //Example use case 

  MotorSpinUp(400, 7000, 4000);

  return 0;
}
