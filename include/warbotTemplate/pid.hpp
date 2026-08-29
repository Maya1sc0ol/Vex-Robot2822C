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

    // Like calculatePID, but for feedback from an absolute angle sensor that wraps at 0/360:
    // folds the error into (-180, 180] so it always drives via the shorter path instead of
    // being thrown off by a seam crossing or a stale/mismatched reference frame.
    // deadbandDeg: once |error| drops below this, output 0 instead of continuing to chase the
    // last couple degrees - without it, shrinking P output can drop below what's needed to
    // overcome the mechanism's static friction, so it stalls just short and the resulting
    // stop/start motion (plus D reacting to it) chatters around the setpoint indefinitely.
    inline double calculateAngularPID(double currentAngle, double goal, PIDconfigs& config, double deadbandDeg = 0.0) {
        double error = goal - currentAngle;
        while (error > 180.0)  error -= 360.0;
        while (error < -180.0) error += 360.0;
        if (error < deadbandDeg && error > -deadbandDeg) {
            config.prev_error = error;
            return 0.0;
        }
        config.integral += error;
        double derivative = error - config.prev_error;
        config.prev_error = error;
        return (config.kP * error) + (config.kI * config.integral) + (config.kD * derivative);
    }

}
