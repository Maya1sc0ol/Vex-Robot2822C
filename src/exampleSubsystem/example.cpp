#include "example.hpp"
#include "main.h"
#include "warbotTemplate/pid.hpp"
#include <cstdint>

void examplePIDFunction(double goal) {
    double current = arm.get_position();
    double output  = warbots::calculatePID(current, goal, armPID);
    arm.move((int32_t)output);
    arm2.move((int32_t)output);
}

void groupControl(double goal) {
    outputMain = warbots::calculatePID(arm.get_position(), goal,armPID);
    group.move((int32_t)outputMain);
}
void openclaw(){
    claw.move_absolute(-650, 50);
}
void closeclaw(){
claw.move_absolute(0, 50);
}