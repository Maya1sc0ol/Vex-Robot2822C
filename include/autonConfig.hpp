#pragma once
#include "warbotTemplate/pid.hpp"

// Single source of truth for the drive/turn PID gains + tiered-exit tuning used by every
// match auto (redAuto, blueAuto, skills, redLeft) - applied once in autonomous() (main.cpp)
// before the selector runs whichever routine was picked, so no individual auton has to
// remember to call setDrivePID()/setTurnPID() itself.
//
// Retune by running pidTuneTest()/pidTurnTuneTest() on hardware and copying the winning
// numbers here once validated. pidTuneTest, pidTurnTuneTest, and odomSquareTest deliberately
// keep their OWN separate local constants instead of these - their whole purpose is to
// experiment with different values before a set of them gets promoted to this file.
namespace autonConfig {

    // Validated in pidTuneTest() (see pid_tune_12/13.csv reasoning in autons.cpp).
    inline const warbots::PIDconfigs DRIVE_PID = {
        /*kP=*/6.0, /*kI=*/0.5, /*kD=*/0.2, /*timeout=*/3000.0, /*startI=*/2.5
    };
    inline const warbots::ExitConditions DRIVE_EXIT = {
        /*smallExitMs=*/150, /*bigError=*/1.5, /*bigExitMs=*/400,
        /*velocityError=*/0.003, /*velocityExitMs=*/300
    };

    // Validated in pidTurnTuneTest() (pid_turn_7.csv - both legs settled under 1deg via the
    // tight tolerance in ~1.1s once kI was raised and BIG/VELOCITY were disabled).
    inline const warbots::PIDconfigs TURN_PID = {
        /*kP=*/3.0, /*kI=*/0.2, /*kD=*/0.3, /*timeout=*/2000.0, /*startI=*/5.0
    };
    inline const warbots::ExitConditions TURN_EXIT = {
        /*smallExitMs=*/150, /*bigError=*/0, /*bigExitMs=*/0,
        /*velocityError=*/0, /*velocityExitMs=*/0
    };

}
