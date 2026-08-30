#include "main.h"
#include "pros/rtos.hpp"
#include "warbotTemplate/util.hpp"
#include <cstdio>

// --- Auton-wide constants ---
const double ARM_HOME_DEG        = 0.0;
const double ARM_FIRST_LEVEL_DEG = 25.0;
const double ARM_TOP_DEG         = 100.0;
const double ARM_TOLERANCE_DEG   = 3.0;
const int    CLAW_SETTLE_MS      = 500;  // time to let the claw finish moving before continuing

const double RED_APPROACH_IN     = 3.0;
const double RED_POST_TURN_DEG   = -45.0;
const double RED_EXIT_FORWARD_IN = 8.0;
const double RED_EXIT_BACK_IN    = -13.5;

// To add a new auto: declare it in autons.h, add one line below, implement it.
void register_autons() {
    selector.autons_add({
        // {shown on brain, function name} - first entry is the default selection
        {"PID Tune Test", pidTuneTest},
        {"Red Auto",  redAuto},
        {"Blue Auto", blueAuto},
        {"Skills",    skills},
        {"Red Left", redLeft},
        {"PID Turn Tune Test", pidTurnTuneTest}

    });
}

void redAuto(){
    closeclaw();
    pros::delay(CLAW_SETTLE_MS);
    armGoTo(ARM_FIRST_LEVEL_DEG, ARM_TOLERANCE_DEG);

    drive.PID_driveInches(RED_APPROACH_IN, 127, 0.5, true);

    openclaw();
    pros::delay(CLAW_SETTLE_MS);
    armGoTo(ARM_TOP_DEG, ARM_TOLERANCE_DEG);
    armGoTo(ARM_HOME_DEG, ARM_TOLERANCE_DEG);

    drive.PID_turnDegrees(RED_POST_TURN_DEG);
    drive.PID_driveInches(RED_EXIT_FORWARD_IN, 127, 0.5, true);
    drive.PID_driveInches(RED_EXIT_BACK_IN, 127, 0.5, true);
}


void redLeft(){
    drive.setMirrored(true);
    drive.PID_driveInches(12, 127, 0.5, true);

}

void blueAuto() {
}

void skills() {
}

// Drives out and back, logging motor/pose data every LOG_INTERVAL_MS to an SD card CSV.
void pidTuneTest() {
    const double TUNE_DISTANCE_IN  = 12.0;
    const int    PAUSE_MS          = 1000;
    const double TUNE_KP           = 6;
    const double TUNE_KI           = 0.0;
    const double TUNE_KD           = 0.2;
    const double TUNE_TIMEOUT_MS   = 3000.0;
    const int    TUNE_MAX_SPEED    = 90;     // 0-127
    const double TUNE_TOLERANCE_IN = 0.5;
    const int    LOG_INTERVAL_MS   = 10;     // matches PID_driveInches' internal loop rate
    const char*  LOG_FILE_PATH     = "/usd/pid_tune_log.csv";

    drive.setDrivePID({TUNE_KP, TUNE_KI, TUNE_KD, TUNE_TIMEOUT_MS});

    static volatile int  currentLeg;
    static volatile bool runLogger;
    currentLeg = 0;   // 1 = outbound leg, 2 = return leg
    runLogger  = true;

    // Baseline the horizontal tracker so logged lateral drift starts at zero for this test,
    // independent of whatever the sensor accumulated before this run.
    const double lateralStart = drive.getHorizontalTrackerInches();

    FILE* logFile = fopen(LOG_FILE_PATH, "w");
    if (logFile != nullptr) {
        fprintf(logFile, "timestamp_ms,leg,left_velocity,right_velocity,left_current,right_current,heading_deg,pose_x,pose_y,pose_angle,lateral_in,battery_pct\n");
    }

    pros::Task loggerTask([&]() {
        while (runLogger) {
            if (logFile != nullptr) {
                auto& pose = drive.getPose();
                fprintf(logFile, "%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\n",
                    pros::millis(), currentLeg,
                    drive.getLeftVelocity(), drive.getRightVelocity(),
                    drive.getLeftCurrent(), drive.getRightCurrent(),
                    drive.getHeading(), pose.x, pose.y, pose.angle,
                    drive.getHorizontalTrackerInches() - lateralStart,
                    pros::battery::get_capacity());
            }
            pros::delay(LOG_INTERVAL_MS);
        }
    });

    currentLeg = 1;
    drive.PID_driveInches(TUNE_DISTANCE_IN, TUNE_MAX_SPEED, TUNE_TOLERANCE_IN, true);

    pros::delay(PAUSE_MS);

    currentLeg = 2;
    drive.PID_driveInches(-TUNE_DISTANCE_IN, TUNE_MAX_SPEED, TUNE_TOLERANCE_IN, true);

    runLogger = false;
    pros::delay(LOG_INTERVAL_MS * 2);
    loggerTask.remove();

    if (logFile != nullptr) {
        fclose(logFile);
    }
}

// Rotates out and back, logging motor/heading data every LOG_INTERVAL_MS to an SD card CSV.
void pidTurnTuneTest() {
    const double TUNE_ANGLE_DEG     = 90.0;
    const int    PAUSE_MS           = 1000;
    const double TURN_KP            = 5.0;
    const double TURN_KI            = 0.0;
    const double TURN_KD            = 0.0;
    const double TURN_TIMEOUT_MS    = 2000.0;
    const int    TURN_MAX_SPEED     = 90;     // 0-127
    const double TURN_TOLERANCE_DEG = 1.0;
    const int    LOG_INTERVAL_MS    = 10;     // matches PID_turnDegrees' internal loop rate
    const char*  LOG_FILE_PATH      = "/usd/pid_turn_log.csv";

    drive.setTurnPID({TURN_KP, TURN_KI, TURN_KD, TURN_TIMEOUT_MS});

    static volatile int  currentLeg;
    static volatile bool runLogger;
    currentLeg = 0;   // 1 = outbound turn, 2 = return turn
    runLogger  = true;

    FILE* logFile = fopen(LOG_FILE_PATH, "w");
    if (logFile != nullptr) {
        fprintf(logFile, "timestamp_ms,leg,left_velocity,right_velocity,left_current,right_current,heading_deg,pose_angle,battery_pct\n");
    }

    pros::Task loggerTask([&]() {
        while (runLogger) {
            if (logFile != nullptr) {
                auto& pose = drive.getPose();
                fprintf(logFile, "%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f\n",
                    pros::millis(), currentLeg,
                    drive.getLeftVelocity(), drive.getRightVelocity(),
                    drive.getLeftCurrent(), drive.getRightCurrent(),
                    drive.getHeading(), pose.angle,
                    pros::battery::get_capacity());
            }
            pros::delay(LOG_INTERVAL_MS);
        }
    });

    currentLeg = 1;
    drive.PID_turnDegrees(TUNE_ANGLE_DEG, TURN_MAX_SPEED, TURN_TOLERANCE_DEG);

    pros::delay(PAUSE_MS);

    currentLeg = 2;
    drive.PID_turnDegrees(-TUNE_ANGLE_DEG, TURN_MAX_SPEED, TURN_TOLERANCE_DEG);

    runLogger = false;
    pros::delay(LOG_INTERVAL_MS * 2);
    loggerTask.remove();

    if (logFile != nullptr) {
        fclose(logFile);
    }
}
