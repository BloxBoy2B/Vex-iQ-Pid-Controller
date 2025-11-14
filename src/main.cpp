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
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor LeftMotor = motor(PORT1, false);
motor RightMotor = motor(PORT6, true);


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}



void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

float angularError;
float targetheading;

// "when started" hat block
int whenStarted1() {
  return 0;
}

#include "vex.h"

// This function turns the robot to a given heading using PID
void turnToHeadingPID(int targetHeading) {
    float error = 0;
    float speed = 0;
    float robotHeading = 0;
    float integral = 0;
    float derivative = 0;
    float previous_error = 0;
    double kp = 0.5;
    double ki = 0.2;
    double kd = 0.3;

    // You can adjust this threshold for how close is 'good enough'
    int threshold = 2;

    while (true) {
        robotHeading = BrainInertial.heading(degrees); // Read the current heading

        error = targetHeading - robotHeading;
        if (error > 180) error -= 360; // Wrap error for easier turning
        if (error < -180) error += 360;

        integral += error;
        derivative = error - previous_error;
        previous_error = error;

        speed = (kp * error) + (ki * integral) + (kd * derivative);

        // Constrain speed (optional, prevents crazy motor values)
        if (speed > 50) speed = 50;
        if (speed < -50) speed = -50;

        // Spin left/right motors to turn
        LeftMotor.spin(forward, speed, percent);
        RightMotor.spin(forward, -speed, percent);

        // Break loop if heading is close enough
        if (fabs(error) <= threshold) {
            LeftMotor.stop();
            RightMotor.stop();
            break;
        }

        // Short delay
        this_thread::sleep_for(20);
    }
}