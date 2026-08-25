#include "example.hpp"
#include "main.h"
#include "warbotTemplate/pid.hpp"
#include <cstdint>

double getArmAngle() {
    double raw = armRotation.get_angle() / 100.0;
    if (!armAngleInitialized) {
        armAngleUnwrapped = raw;
        armAngleInitialized = true;
    } else {
        double delta = raw - armAnglePrevRaw;
        if (delta > 180.0) delta -= 360.0;
        else if (delta < -180.0) delta += 360.0;
        armAngleUnwrapped += delta;
    }
    armAnglePrevRaw = raw;
    return armAngleUnwrapped;
}

void groupControl(double goal) {
    outputMain = warbots::calculatePID(getArmAngle(), goal, armPID);
    group.move((int32_t)outputMain);
}
void openclaw(){
    claw.move_absolute(700, 400);
}
void closeclaw(){
claw.move_absolute(0, 400);
}