#include "main.h"
#include "pros/rtos.hpp"
#include "warbotTemplate/util.hpp"
#include <cstdio>



// To add a new auto: declare it in autons.h, add one line below, implement it.
void register_autons() {
    selector.autons_add({
        //The first part in brackets that is in quotes is what is shown on the brain
        //The second part of the brackets is the function name so whatever you named it in autos.cpp
        {"Red Auto",  redAuto},
        {"Blue Auto", blueAuto},
        {"Skills",    skills},
        {"Red Left", redLeft},
        {"PID Tune Test", pidTuneTest}

    });
}

void redAuto(){// red alliance autonomous routine
  drive.PID_driveInches(30); //Forward, right, left, forward, backwards
  pros::delay(1000);
  drive.PID_driveInches(90);
  pros::delay(1000);
  drive.PID_driveInches(20);
  }
    

void redLeft(){
    //Use this line to code to mirror left auto to right auto or reversed!
    drive.setMirrored(true);
    drive.PID_driveInches(12);

}

void blueAuto() {
    // blue alliance autonomous routine
}

void skills() {
    // skills autonomous routine
}

// PID tuning test: drives straight 20in, pauses, drives straight back 20in,
// while logging motor/pose data every LOG_INTERVAL_MS to an SD card CSV.
// Edit the constants below to tune gains, change the pause, or adjust the run.
void pidTuneTest() {
    // --- Edit these to tune/run the test ---
    const double TUNE_DISTANCE_IN  = 20.0;   // one-way distance, in inches
    const int    PAUSE_MS          = 1000;   // pause between forward and backward legs
    const double TUNE_KP           = 0.6;
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

    FILE* logFile = fopen(LOG_FILE_PATH, "w");
    if (logFile != nullptr) {
        fprintf(logFile, "timestamp_ms,leg,left_velocity,right_velocity,left_current,right_current,heading_deg,pose_x,pose_y,pose_angle\n");
    }

    pros::Task loggerTask([&]() {
        while (runLogger) {
            if (logFile != nullptr) {
                auto& pose = drive.getPose();
                fprintf(logFile, "%u,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    pros::millis(), currentLeg,
                    drive.getLeftVelocity(), drive.getRightVelocity(),
                    drive.getLeftCurrent(), drive.getRightCurrent(),
                    drive.getHeading(), pose.x, pose.y, pose.angle);
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
