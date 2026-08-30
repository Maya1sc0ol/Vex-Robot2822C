#include "main.h"
#include "pros/rtos.hpp"
#include "warbotTemplate/util.hpp"
#include "warbotTemplate/logging.hpp"
#include <cstdio>

using HeadingHoldMode = warbots::Drive::HeadingHoldMode;

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

// Builds the standard full-sensor column set every tuning/diagnostic auton logs: per-side and
// per-motor drivetrain telemetry, full IMU, the horizontal tracking wheel, fused odometry pose,
// PID_driveInches' last output/correction, and battery - so any test run also doubles as
// odometry-validation data. currentLeg is captured by reference so the caller can update it
// (e.g. 1 = outbound, 2 = return) as the test progresses; lateralStart baselines the tracker
// reading so logged drift starts at zero for this run.
static std::vector<warbots::LogColumn> fullTelemetryColumns(volatile int& currentLeg, double lateralStart) {
    return {
        {"timestamp_ms",      [] { return (double)pros::millis(); }},
        {"leg",               [&currentLeg] { return (double)currentLeg; }},
        {"left_velocity",     [] { return drive.getLeftVelocity(); }},
        {"right_velocity",    [] { return drive.getRightVelocity(); }},
        {"left_current",      [] { return drive.getLeftCurrent(); }},
        {"right_current",     [] { return drive.getRightCurrent(); }},
        {"left_m0_current",   [] { return drive.getLeftMotorTelemetry(0).current; }},
        {"left_m0_voltage",   [] { return drive.getLeftMotorTelemetry(0).voltage; }},
        {"left_m0_temp",      [] { return drive.getLeftMotorTelemetry(0).temperature; }},
        {"left_m0_position",  [] { return drive.getLeftMotorTelemetry(0).position; }},
        {"left_m1_current",   [] { return drive.getLeftMotorTelemetry(1).current; }},
        {"left_m1_voltage",   [] { return drive.getLeftMotorTelemetry(1).voltage; }},
        {"left_m1_temp",      [] { return drive.getLeftMotorTelemetry(1).temperature; }},
        {"left_m1_position",  [] { return drive.getLeftMotorTelemetry(1).position; }},
        {"right_m0_current",  [] { return drive.getRightMotorTelemetry(0).current; }},
        {"right_m0_voltage",  [] { return drive.getRightMotorTelemetry(0).voltage; }},
        {"right_m0_temp",     [] { return drive.getRightMotorTelemetry(0).temperature; }},
        {"right_m0_position", [] { return drive.getRightMotorTelemetry(0).position; }},
        {"right_m1_current",  [] { return drive.getRightMotorTelemetry(1).current; }},
        {"right_m1_voltage",  [] { return drive.getRightMotorTelemetry(1).voltage; }},
        {"right_m1_temp",     [] { return drive.getRightMotorTelemetry(1).temperature; }},
        {"right_m1_position", [] { return drive.getRightMotorTelemetry(1).position; }},
        {"heading_deg",       [] { return drive.getHeading(); }},
        {"imu_rotation",      [] { return drive.getImuRotation(); }},
        {"imu_pitch",         [] { return drive.getImuPitch(); }},
        {"imu_roll",          [] { return drive.getImuRoll(); }},
        {"imu_yaw",           [] { return drive.getImuYaw(); }},
        {"gyro_x",            [] { return drive.getImuGyroRate().x; }},
        {"gyro_y",            [] { return drive.getImuGyroRate().y; }},
        {"gyro_z",            [] { return drive.getImuGyroRate().z; }},
        {"accel_x",           [] { return drive.getImuAccel().x; }},
        {"accel_y",           [] { return drive.getImuAccel().y; }},
        {"accel_z",           [] { return drive.getImuAccel().z; }},
        {"pose_x",            [] { return drive.getPose().x; }},
        {"pose_y",            [] { return drive.getPose().y; }},
        {"pose_angle",        [] { return drive.getPose().angle; }},
        {"tracker_in",        [lateralStart] { return drive.getHorizontalTrackerInches() - lateralStart; }},
        {"tracker_vel_in_s",  [] { return drive.getHorizontalTrackerVelocity(); }},
        {"pid_output",        [] { return drive.getLastDriveOutput(); }},
        {"pid_correction",    [] { return drive.getLastDriveCorrection(); }},
        {"battery_pct",       [] { return pros::battery::get_capacity(); }},
        {"battery_mV",        [] { return (double)pros::battery::get_voltage(); }},
        {"battery_mA",        [] { return (double)pros::battery::get_current(); }},
    };
}

// To add a new auto: declare it in autons.h, add one line below, implement it.
void register_autons() {
    selector.autons_add({
        // {shown on brain, function name} - first entry is the default selection
        {"PID Tune Test", pidTuneTest},
        {"Red Auto",  redAuto},
        {"Blue Auto", blueAuto},
        {"Skills",    skills},
        {"Red Left", redLeft},
        {"PID Turn Tune Test", pidTurnTuneTest},
        {"Odom Pod Straight Test", driveOdomPodTest},
        {"Encoder-Only Drive Test", driveEncoderOnlyTest},
        {"Odom Square Test", odomSquareTest}

    });
}

void redAuto(){
    closeclaw();
    pros::delay(CLAW_SETTLE_MS);
    armGoTo(ARM_FIRST_LEVEL_DEG, ARM_TOLERANCE_DEG);

    drive.PID_driveInches(RED_APPROACH_IN, 127, 0.5, HeadingHoldMode::IMU_HEADING);

    openclaw();
    pros::delay(CLAW_SETTLE_MS);
    armGoTo(ARM_TOP_DEG, ARM_TOLERANCE_DEG);
    armGoTo(ARM_HOME_DEG, ARM_TOLERANCE_DEG);

    drive.PID_turnDegrees(RED_POST_TURN_DEG);
    drive.PID_driveInches(RED_EXIT_FORWARD_IN, 127, 0.5, HeadingHoldMode::IMU_HEADING);
    drive.PID_driveInches(RED_EXIT_BACK_IN, 127, 0.5, HeadingHoldMode::IMU_HEADING);
}


void redLeft(){
    drive.setMirrored(true);
    drive.PID_driveInches(12, 127, 0.5, HeadingHoldMode::IMU_HEADING);

}

void blueAuto() {
}

void skills() {
}

// Drives out and back holding heading via the IMU, logging full drivetrain/IMU/tracker/pose
// telemetry to a fresh SD card CSV each run.
void pidTuneTest() {
    const double TUNE_DISTANCE_IN  = 12.0;
    const int    PAUSE_MS          = 1000;
    const double TUNE_KP           = 6;
    const double TUNE_KI           = 0.0;
    const double TUNE_KD           = 0.2;
    const double TUNE_TIMEOUT_MS   = 3000.0;
    const int    TUNE_MAX_SPEED    = 90;     // 0-127
    const double TUNE_TOLERANCE_IN = 0.5;

    drive.setDrivePID({TUNE_KP, TUNE_KI, TUNE_KD, TUNE_TIMEOUT_MS});

    static volatile int currentLeg;
    currentLeg = 0;   // 1 = outbound leg, 2 = return leg

    warbots::CsvLogger logger("pid_tune", fullTelemetryColumns(currentLeg, drive.getHorizontalTrackerInches()));
    logger.start();

    currentLeg = 1;
    drive.PID_driveInches(TUNE_DISTANCE_IN, TUNE_MAX_SPEED, TUNE_TOLERANCE_IN, HeadingHoldMode::IMU_HEADING);

    pros::delay(PAUSE_MS);

    currentLeg = 2;
    drive.PID_driveInches(-TUNE_DISTANCE_IN, TUNE_MAX_SPEED, TUNE_TOLERANCE_IN, HeadingHoldMode::IMU_HEADING);

    logger.stop();
}

// Rotates out and back, logging full drivetrain/IMU/tracker/pose telemetry to a fresh SD card
// CSV each run.
void pidTurnTuneTest() {
    const double TUNE_ANGLE_DEG     = 90.0;
    const int    PAUSE_MS           = 1000;
    const double TURN_KP            = 5.0;
    const double TURN_KI            = 0.0;
    const double TURN_KD            = 0.0;
    const double TURN_TIMEOUT_MS    = 2000.0;
    const int    TURN_MAX_SPEED     = 90;     // 0-127
    const double TURN_TOLERANCE_DEG = 1.0;

    drive.setTurnPID({TURN_KP, TURN_KI, TURN_KD, TURN_TIMEOUT_MS});

    static volatile int currentLeg;
    currentLeg = 0;   // 1 = outbound turn, 2 = return turn

    warbots::CsvLogger logger("pid_turn", fullTelemetryColumns(currentLeg, drive.getHorizontalTrackerInches()));
    logger.start();

    currentLeg = 1;
    drive.PID_turnDegrees(TUNE_ANGLE_DEG, TURN_MAX_SPEED, TURN_TOLERANCE_DEG);

    pros::delay(PAUSE_MS);

    currentLeg = 2;
    drive.PID_turnDegrees(-TUNE_ANGLE_DEG, TURN_MAX_SPEED, TURN_TOLERANCE_DEG);

    logger.stop();
}

// Drives out and back holding zero lateral drift via the horizontal tracking wheel (odom pod),
// instead of matching a heading via the IMU. Verify the correction direction on the bench: the
// robot should visibly steer back toward the starting line, not away from it - flip
// ODOM_POD_KP's sign if it does the opposite.
void driveOdomPodTest() {
    const double TEST_DISTANCE_IN  = 12.0;
    const int    PAUSE_MS          = 1000;
    const double TEST_KP           = 6;
    const double TEST_KI           = 0.0;
    const double TEST_KD           = 0.2;
    const double TEST_TIMEOUT_MS   = 3000.0;
    const int    TEST_MAX_SPEED    = 90;     // 0-127
    const double TEST_TOLERANCE_IN = 0.5;
    const double ODOM_POD_KP       = 2.0;    // proportional gain on lateral-drift correction

    drive.setDrivePID({TEST_KP, TEST_KI, TEST_KD, TEST_TIMEOUT_MS});

    static volatile int currentLeg;
    currentLeg = 0;   // 1 = outbound leg, 2 = return leg

    warbots::CsvLogger logger("odom_pod", fullTelemetryColumns(currentLeg, drive.getHorizontalTrackerInches()));
    logger.start();

    currentLeg = 1;
    drive.PID_driveInches(TEST_DISTANCE_IN, TEST_MAX_SPEED, TEST_TOLERANCE_IN, HeadingHoldMode::ODOM_LATERAL, ODOM_POD_KP);

    pros::delay(PAUSE_MS);

    currentLeg = 2;
    drive.PID_driveInches(-TEST_DISTANCE_IN, TEST_MAX_SPEED, TEST_TOLERANCE_IN, HeadingHoldMode::ODOM_LATERAL, ODOM_POD_KP);

    logger.stop();
}

// Drives out and back using only motor-encoder distance feedback - no heading or lateral
// correction at all - to see how far the robot drifts with nothing steering it straight.
void driveEncoderOnlyTest() {
    const double TEST_DISTANCE_IN  = 12.0;
    const int    PAUSE_MS          = 1000;
    const double TEST_KP           = 6;
    const double TEST_KI           = 0.0;
    const double TEST_KD           = 0.2;
    const double TEST_TIMEOUT_MS   = 3000.0;
    const int    TEST_MAX_SPEED    = 90;     // 0-127
    const double TEST_TOLERANCE_IN = 0.5;

    drive.setDrivePID({TEST_KP, TEST_KI, TEST_KD, TEST_TIMEOUT_MS});

    static volatile int currentLeg;
    currentLeg = 0;   // 1 = outbound leg, 2 = return leg

    warbots::CsvLogger logger("encoder_only", fullTelemetryColumns(currentLeg, drive.getHorizontalTrackerInches()));
    logger.start();

    currentLeg = 1;
    drive.PID_driveInches(TEST_DISTANCE_IN, TEST_MAX_SPEED, TEST_TOLERANCE_IN, HeadingHoldMode::NONE);

    pros::delay(PAUSE_MS);

    currentLeg = 2;
    drive.PID_driveInches(-TEST_DISTANCE_IN, TEST_MAX_SPEED, TEST_TOLERANCE_IN, HeadingHoldMode::NONE);

    logger.stop();
}

// Drives a square (four legs with 90-degree turns between), returning near the start. Straight
// legs alone can't exercise updatePose()'s lateral-coupling terms; physically measuring the
// robot's actual end position/heading against the logged pose after this run is how the fused
// odometry gets validated (there's no sensor-independent ground truth the firmware can check
// itself against).
void odomSquareTest() {
    const double SIDE_IN           = 24.0;
    const double TURN_DEG          = 90.0;
    const int    PAUSE_MS          = 500;
    const int    MAX_SPEED         = 90;     // 0-127
    const double TOLERANCE_IN      = 0.5;
    const double TURN_TOLERANCE_DEG = 1.0;

    static volatile int currentLeg;
    currentLeg = 0;   // 1-4 = drive legs, 5-8 = turns between them

    warbots::CsvLogger logger("odom_square", fullTelemetryColumns(currentLeg, drive.getHorizontalTrackerInches()));
    logger.start();

    for (int side = 0; side < 4; side++) {
        currentLeg = side * 2 + 1;
        drive.PID_driveInches(SIDE_IN, MAX_SPEED, TOLERANCE_IN, HeadingHoldMode::IMU_HEADING);
        pros::delay(PAUSE_MS);

        currentLeg = side * 2 + 2;
        drive.PID_turnDegrees(TURN_DEG, MAX_SPEED, TURN_TOLERANCE_DEG);
        pros::delay(PAUSE_MS);
    }

    logger.stop();
}
