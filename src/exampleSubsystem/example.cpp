#include "example.hpp"
#include "main.h"
#include "warbotTemplate/pid.hpp"
#include <cstdint>
#include <cmath>

double getArmAngle() {
    return armRotation.get_angle() / 100.0;
}

void groupControl(double goal) {
    double current = getArmAngle();
    double pidOutput = warbots::calculateAngularPID(current, goal, armPID, armDeadbandDeg);
    // Peaks at current=50 (roughly horizontal), 0 at current=0 or 100 (roughly vertical).
    double gravityFF = armGravityFF * std::sin((current / 100.0) * M_PI);
    outputMain = pidOutput + gravityFF;
    group.move((int32_t)outputMain);
}
void openclaw(){
    claw.move_absolute(700, 400);
}
void closeclaw(){
claw.move_absolute(0, 400);
}

// Blocking: drives the arm to targetDeg and returns once within toleranceDeg,
// or armPID.timeout elapses. Mirrors Drive's PID_driveInches/PID_turnDegrees
// for use in autons.
void armGoTo(double targetDeg, double toleranceDeg) {
    armPID.prev_error = 0;
    armPID.integral   = 0;

    double rampedGoal = getArmAngle();  // ease from current position, not a snap
    uint32_t startTime = pros::millis();

    while (true) {
        if (pros::millis() - startTime >= (uint32_t)armPID.timeout) break;

        double rampStep = targetDeg - rampedGoal;
        if (rampStep > armRampDegPerTick) rampStep = armRampDegPerTick;
        else if (rampStep < -armRampDegPerTick) rampStep = -armRampDegPerTick;
        rampedGoal += rampStep;

        double error = targetDeg - getArmAngle();
        while (error > 180.0)  error -= 360.0;
        while (error < -180.0) error += 360.0;
        if (std::fabs(error) < toleranceDeg) break;

        groupControl(rampedGoal);
        pros::delay(20);
    }

    group.move(0);
}