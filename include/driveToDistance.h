#ifndef DRIVETODISTANCE_H
#define DRIVETODISTANCE_H

#include "vex.h"

class driveToDistance
{
private:
    // Distance PID variables
    float distanceIntegral;
    float distancePreviousError;
    float kp;
    float ki;
    float kd;
    int threshold;

    // Heading PID variables
    float headingIntegral;
    float headingPreviousError;
    float kpHeading;
    float kiHeading;
    float kdHeading;
    int headingToHoldThreshold;

public:
    driveToDistance(float distanceProportional = 0.5, float distanceIntegral = 0.01,
                   float distanceDerivative = 0.3, int distanceErrorThreshold = 10,
                   float headingProportional = 0.3, float headingIntegral = 0.05,
                   float headingDerivative = 0.1, int headingErrorThreshold = 2);
    ~driveToDistance();

    void execute(double targetDistance, double headingToHold, double timeOut);
};

#endif

