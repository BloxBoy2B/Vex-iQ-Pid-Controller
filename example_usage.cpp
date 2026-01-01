#include "vex.h"
#include "turnToHeadingPID.h"
#include "driveToDistance.h"

using namespace vex;

int main() {
    vexcodeInit();

    // Create PID controllers with custom tuning
    TurnToHeadingPID turnPID(0.6, 0.15, 0.35, 2);
    driveToDistance drivePID(0.5, 0.01, 0.3, 10, 0.3, 0.05, 0.1, 2);

    // Example autonomous routine
    // Drive forward 1000 degrees while holding heading 0
    drivePID.execute(1000, 0, 5000);

    wait(1, seconds);

    // Turn to 90 degrees
    turnPID.execute(90, 3000);

    wait(1, seconds);

    // Drive forward another 800 degrees while holding heading 90
    drivePID.execute(800, 90, 4000);

    return 0;
}