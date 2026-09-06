#pragma once
#include "main.h"

namespace warbots{

    struct PIDconfigs {
        double kP;
        double kI;
        double kD;
        double timeout;
        double startI = 0.0;           // |error| must drop below this before kI starts accumulating;
                                        // 0 = accumulate immediately (matches old behavior). Lets kI
                                        // stay off during the fast far-from-target part of a move and
                                        // only kick in to push through static friction near the end.
        double prev_error = 0;
        double prev_measurement = 0;   // only used when calculatePID's derivativeOnMeasurement=true
        double integral   = 0;
        double integralClamp = 0.0;    // max |kI * integral| contribution; 0 = unclamped (old
                                        // behavior). odom_square_1.csv: kI=0.5/startI=2.5 let the
                                        // integral term wind up to ~37 of output by the time the
                                        // drive was already past target - a visible "slow down, then
                                        // extra juice" thrust profile and a slight overshoot. Clamping
                                        // keeps just enough integral to break static friction near the
                                        // end without letting it snowball.
    };

    // derivativeOnMeasurement: compute D from the change in `current` instead of the change in
    // `error`, avoiding "derivative kick" on the first tick of a fresh PID_driveInches/PID_turnDegrees
    // call (prev_error resets to 0, so tick 1 would otherwise compute derivative = error - 0 = error).
    // Left false by default because some callers (PID_swingToAngle) pass a fixed current=0.0 and
    // precompute error themselves, which would zero the D-term forever under this mode.
    inline double calculatePID(double current, double goal, PIDconfigs& config,
                                bool derivativeOnMeasurement = false) {
        double error = goal - current;

        if (config.kI != 0.0 && (config.startI <= 0.0 || std::fabs(error) < config.startI)) {
            config.integral += error;
        }

        double integralTerm = config.kI * config.integral;
        if (config.integralClamp > 0.0) {
            integralTerm = std::max(-config.integralClamp, std::min(config.integralClamp, integralTerm));
        }

        double derivative = derivativeOnMeasurement
            ? (config.prev_measurement - current)
            : (error - config.prev_error);

        config.prev_error = error;
        config.prev_measurement = current;

        return (config.kP * error) + integralTerm + (config.kD * derivative);
    }

    // Tunables for PID_driveInches()/PID_turnDegrees()'s tiered exit. All-zero (default) disables
    // every tier, so the loop falls back to a single instantaneous tolerance-or-timeout check.
    struct ExitConditions {
        int    smallExitMs    = 0;  // ms the caller's tight tolerance must hold before exiting;
                                     // 0 = exit on the first qualifying tick
        double bigError       = 0;  // looser fallback tolerance so a "close enough" move doesn't
                                     // have to wait out the full raw timeout; 0 = disabled
        int    bigExitMs      = 0;
        double velocityError  = 0;  // |change in traveled/turned per tick| under this = "stalled";
                                     // 0 = disabled
        int    velocityExitMs = 0;
    };

    // Folds a raw angular delta into (-180, 180] so it always represents the shorter path
    // around a 0/360 seam instead of the raw, potentially seam-crossing difference.
    inline double wrapAngleDeg(double error) {
        while (error > 180.0)  error -= 360.0;
        while (error < -180.0) error += 360.0;
        return error;
    }

    // Like calculatePID, but for feedback from an absolute angle sensor that wraps at 0/360:
    // folds the error into (-180, 180] so it always drives via the shorter path instead of
    // being thrown off by a seam crossing or a stale/mismatched reference frame.
    // deadbandDeg: once |error| drops below this, output 0 instead of continuing to chase the
    // last couple degrees - without it, shrinking P output can drop below what's needed to
    // overcome the mechanism's static friction, so it stalls just short and the resulting
    // stop/start motion (plus D reacting to it) chatters around the setpoint indefinitely.
    inline double calculateAngularPID(double currentAngle, double goal, PIDconfigs& config, double deadbandDeg = 0.0) {
        double error = wrapAngleDeg(goal - currentAngle);
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
