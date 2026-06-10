#pragma once
#include "main.h"

namespace warbots{

    struct PIDconfigs {
        double kP;
        double kI;
        double kD;
        double timeout;
        double prev_error = 0;
        double integral   = 0;
    };

    inline double calculatePID(double current, double goal, PIDconfigs& config) {
        double error = goal - current;
        config.integral += error;
        double derivative = error - config.prev_error;
        config.prev_error = error;
        return (config.kP * error) + (config.kI * config.integral) + (config.kD * derivative);
    }

}
