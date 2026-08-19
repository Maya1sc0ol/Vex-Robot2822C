#include "example.hpp"
#include "main.h"
#include "warbotTemplate/pid.hpp"
#include <cstdint>

void groupControl(double goal) {
    outputMain = warbots::calculatePID(arm.get_position(), goal,armPID);
    group.move((int32_t)outputMain);
}
void openclaw(){
    claw.move_absolute(700, 400);
}
void closeclaw(){
claw.move_absolute(0, 400);
}