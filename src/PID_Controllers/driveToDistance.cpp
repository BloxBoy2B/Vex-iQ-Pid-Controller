#include "vex.h"
#include "driveToDistance.h"

using namespace vex;

// Constructor
driveToDistance::driveToDistance(float distanceProportional, float distanceIntegral,
                                float distanceDerivative, int distanceErrorThreshold,
                                float headingProportional, float headingIntegral,
                                float headingDerivative, int headingErrorThreshold)
{
    // Distance PID parameters
    kp = distanceProportional;
    ki = distanceIntegral;
    kd = distanceDerivative;
    threshold = distanceErrorThreshold;

    // Heading PID parameters
    kpHeading = headingProportional;
    kiHeading = headingIntegral;
    kdHeading = headingDerivative;
    headingToHoldThreshold = headingErrorThreshold;

    // Initialize state variables
    distanceIntegral = 0;
    distancePreviousError = 0;
    headingIntegral = 0;
    headingPreviousError = 0;
}

// Destructor
driveToDistance::~driveToDistance()
{
}

// Execute the PID control loop
void driveToDistance::execute(double targetDistance, double headingToHold, double timeOut)
{
    // Reset integral and previous error
    distanceIntegral = 0;
    distancePreviousError = 0;
    headingIntegral = 0;
    headingPreviousError = 0;

    timer myTimer;

    // Reset motor positions
    LeftMotor.setPosition(0, degrees);
    RightMotor.setPosition(0, degrees);

    while (true)
    {
        // Get average position of both motors (in degrees)
        float currentDistance = (LeftMotor.position(degrees) + RightMotor.position(degrees)) / 2.0;

        // Distance PID calculation
        float distanceError = targetDistance - currentDistance;

        distanceIntegral += distanceError;

        // Anti-windup for distance integral
        if (distanceIntegral > 400)
            distanceIntegral = 400;
        if (distanceIntegral < -400)
            distanceIntegral = -400;

        // Reset integral when close to target
        if (fabs(distanceError) < 5)
            distanceIntegral = 0;

        float distanceDerivative = distanceError - distancePreviousError;
        distancePreviousError = distanceError;

        float speed = (kp * distanceError) + (ki * distanceIntegral) + (kd * distanceDerivative);

        // Constrain speed
        if (speed > 50)
            speed = 50;
        if (speed < -50)
            speed = -50;

        // Heading PID calculation
        float currentHeading = BrainInertial.heading(degrees);
        float headingError = headingToHold - currentHeading;

        // Normalize heading error to -180 to 180 range
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;

        headingIntegral += headingError;

        // Anti-windup for heading integral
        if (headingIntegral > 100)
            headingIntegral = 100;
        if (headingIntegral < -100)
            headingIntegral = -100;

        // Reset integral when close to target heading
        if (fabs(headingError) < 2)
            headingIntegral = 0;

        float headingDerivative = headingError - headingPreviousError;
        headingPreviousError = headingError;

        float headingCorrection = (kpHeading * headingError) + (kiHeading * headingIntegral) + (kdHeading * headingDerivative);

        // Constrain heading correction
        if (headingCorrection > 20)
            headingCorrection = 20;
        if (headingCorrection < -20)
            headingCorrection = -20;

        // Apply differential drive
        float leftSpeed = speed + headingCorrection;
        float rightSpeed = speed - headingCorrection;

        LeftMotor.spin(forward, leftSpeed, percent);
        RightMotor.spin(forward, rightSpeed, percent);

        // Check if we're done (both distance and heading are within thresholds, or timeout)
        if ((fabs(distanceError) <= threshold && fabs(headingError) <= headingToHoldThreshold) ||
            myTimer.time(msec) >= timeOut)
        {
            LeftMotor.stop(brake);
            RightMotor.stop(brake);
            break;
        }

        // Short delay
        this_thread::sleep_for(20);
    }
}
