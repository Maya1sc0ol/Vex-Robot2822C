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