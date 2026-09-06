#include "main.h"
#include "pros/rtos.hpp"
#include "warbotTemplate/util.hpp"
#include "warbotTemplate/logging.hpp"
#include <cstdio>

using HeadingHoldMode = warbots::Drive::HeadingHoldMode;

// --- Auton-wide constants ---
const double ARM_HOME_DEG        = 0.0;
const double ARM_FIRST_LEVEL_DEG = 25.0;
const double ARM_TOLERANCE_DEG   = 3.0;
const int    CLAW_SETTLE_MS      = 500;  // time to let the claw finish moving before continuing

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
        {"error_in",          [] { return drive.getLastDriveError(); }},
        {"drive_exit_reason", [] { return (double)drive.getLastDriveExitReason(); }},
        {"turn_output",       [] { return drive.getLastTurnOutput(); }},
        {"turn_correction",   [] { return drive.getLastTurnCorrection(); }},
        {"turn_error_deg",    [] { return drive.getLastTurnError(); }},
        {"turn_exit_reason",  [] { return (double)drive.getLastTurnExitReason(); }},
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
        {"Odom Square Test", odomSquareTest}

    });
}

// Exercises the full set of movement primitives a match auto is likely to reuse: forward/back
// driving, a point turn, arm positioning, and claw actuation. Absolute heading is tracked
// explicitly (same pattern as odomSquareTest()) so each straight leg self-corrects to the true
// intended heading instead of compounding a turn's residual overshoot into the next leg.
void redAuto(){
    double heading = 0.0;

    closeclaw();
    pros::delay(CLAW_SETTLE_MS);

    drive.PID_driveInches(12, 127, 0.5, HeadingHoldMode::IMU_HEADING, 0.4, 0.3, heading);

    heading -= 90.0;  // left turn (CW-positive convention)
    drive.PID_turnToHeading(heading, 127, 1.0);

    drive.PID_driveInches(12, 127, 0.5, HeadingHoldMode::IMU_HEADING, 0.4, 0.3, heading);

    armGoTo(ARM_FIRST_LEVEL_DEG, ARM_TOLERANCE_DEG);

    drive.PID_driveInches(4, 127, 0.5, HeadingHoldMode::IMU_HEADING, 0.4, 0.3, heading);

    openclaw();
    pros::delay(CLAW_SETTLE_MS);

    drive.PID_driveInches(-4, 127, 0.5, HeadingHoldMode::IMU_HEADING, 0.4, 0.3, heading);

    armGoTo(ARM_HOME_DEG, ARM_TOLERANCE_DEG);

    drive.PID_driveInches(-12, 127, 0.5, HeadingHoldMode::IMU_HEADING, 0.4, 0.3, heading);

    heading += 90.0;  // back to original heading
    drive.PID_turnToHeading(heading, 127, 1.0);

    drive.PID_driveInches(-12, 127, 0.5, HeadingHoldMode::IMU_HEADING, 0.4, 0.3, heading);
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
    const double TUNE_DISTANCE_IN  = 24.0;
    const int    PAUSE_MS          = 1000;
    const double TUNE_KP           = 6.0;     // reverted from 4.5: pid_tune_12/13.csv's new error_in
                                               // column shows the "never settles" failure is a smooth
                                               // monotonic creep, NOT an oscillation/limit-cycle as
                                               // first suspected from pid_tune_10/11.csv - softening kP
                                               // just made the approach slower with no benefit
    const double TUNE_KI           = 0.5;     // was 0.2 (0.4 before that) - the real bug: with KP=4.5
                                               // and START_I=1.0, error never dropped below 1.0in
                                               // before the 3s timeout in EITHER pid_tune_12/13 run, so
                                               // kI never accumulated at all; bumping slightly above the
                                               // original 0.4 to help it close faster once it does engage
    const double TUNE_KD           = 0.2;     // reverted from 0.35 - no oscillation ever showed up in
                                               // error_in, so there was nothing to damp
    const double TUNE_TIMEOUT_MS   = 3000.0;  // still the final safety net, not the normal exit
    const double TUNE_START_I      = 2.5;     // was 1.0 (2.0 originally) - raised above the original
                                               // value so kI has more runway to wind up and close the
                                               // last few inches before hitting the timeout
    const int    TUNE_MAX_SPEED    = 90;     // 0-127
    const double TUNE_TOLERANCE_IN = 0.5;

    // Tiered exit (see ExitConditions in pid.hpp): settle inside TUNE_TOLERANCE_IN for
    // TUNE_SMALL_EXIT_MS, or fall back to the looser TUNE_BIG_TOLERANCE_IN, or bail out if the
    // move visibly stalls (near-zero progress per tick) for TUNE_VELOCITY_EXIT_MS. Starting
    // points to bench-tune, not gospel.
    const int    TUNE_SMALL_EXIT_MS        = 150;
    const double TUNE_BIG_TOLERANCE_IN     = 1.5;
    const int    TUNE_BIG_EXIT_MS          = 400;
    const double TUNE_VELOCITY_IN_PER_TICK = 0.003;  // ~0.3 in/s (per-10ms-tick: 0.3 * 0.01 = 0.003) -
                                                       // was 0.03, which is actually 3 in/s and tripped
                                                       // the stall exit almost as soon as the robot
                                                       // decelerated out of its initial fast approach
    const int    TUNE_VELOCITY_EXIT_MS     = 300;

    drive.setDrivePID({TUNE_KP, TUNE_KI, TUNE_KD, TUNE_TIMEOUT_MS, TUNE_START_I});
    drive.setDriveExit({TUNE_SMALL_EXIT_MS, TUNE_BIG_TOLERANCE_IN, TUNE_BIG_EXIT_MS,
                         TUNE_VELOCITY_IN_PER_TICK, TUNE_VELOCITY_EXIT_MS});
    drive.resetPose();

    static volatile int currentLeg;
    currentLeg = 0;   // 1 = outbound leg, 2 = return leg

    warbots::CsvLogger logger("pid_tune", fullTelemetryColumns(currentLeg, drive.getHorizontalTrackerInches()));
    logger.start();

    currentLeg = 1;
    drive.PID_driveInches(TUNE_DISTANCE_IN, TUNE_MAX_SPEED, TUNE_TOLERANCE_IN, HeadingHoldMode::IMU_HEADING, 1.0);

    pros::delay(PAUSE_MS);

    currentLeg = 2;
    drive.PID_driveInches(-TUNE_DISTANCE_IN, TUNE_MAX_SPEED, TUNE_TOLERANCE_IN, HeadingHoldMode::IMU_HEADING, 1.0);

    logger.stop();
}

// Rotates out and back, logging full drivetrain/IMU/tracker/pose telemetry to a fresh SD card
// CSV each run.
void pidTurnTuneTest() {
    const double TUNE_ANGLE_DEG     = 90.0;
    const int    PAUSE_MS           = 1000;
    const double TURN_KP            = 3.0;     // RESET - pid_turn_1/2/3/4.csv all diverged
                                                // regardless of kP/kD/maxSpeed because of a real
                                                // sign bug in PID_turnDegrees (see drive.hpp fix
                                                // above): positive output drove the robot CCW while
                                                // pose.angle is CW-positive, so error grew without
                                                // bound no matter what these gains were set to. All
                                                // 4 rounds of "tuning" were chasing a phantom problem
                                                // - resetting to a sane starting point now that the
                                                // actual control loop can converge at all
    const double TURN_KI            = 0.2;     // pid_turn_5/6.csv: leg 2 (the -90 return turn)
                                                // kept stalling ~1-1.2deg short of target. kI=0.05
                                                // (pid_turn_6.csv) only got turn_output up to ~5 out
                                                // of 70 at that error size - still not enough to break
                                                // static friction, and error plateaued instead of
                                                // closing. Going noticeably stronger so integral can
                                                // actually punch through within the time it gets below.
    const double TURN_START_I       = 5.0;     // only accumulate integral once within 5deg of target,
                                                // so it doesn't wind up during the fast far-from-target
                                                // part of the turn
    const double TURN_KD            = 0.3;     // RESET - modest damping starting point, not the
                                                // KD=15 guess made while fighting the sign bug above
    const double TURN_TIMEOUT_MS    = 2000.0;
    const int    TURN_MAX_SPEED     = 70;     // RESET - no longer need to artificially cap speed to
                                               // limit momentum from a turn that could never actually
                                               // approach target; back to a normal working range
    const double TURN_TOLERANCE_DEG = 1.0;

    // Tiered exit (see ExitConditions in pid.hpp) - BIG and VELOCITY are disabled (0) for this
    // tuning pass. pid_turn_6.csv showed leg 2's BIG timer (tightened to 2.5deg/300ms in the last
    // round) started counting the instant error crossed under 2.5deg and hit its 300ms threshold
    // long before the error could ever crawl under SMALL's 1deg tolerance - it fired at 1.2deg of
    // real error with barely half the raw 2000ms timeout used up. BIG structurally requires far
    // less dwell time than it takes to close that last mile, so it will keep winning the race and
    // masking whether a given kI actually converges, no matter how kI is tuned. VELOCITY would have
    // the same problem here (the near-stalled creep at ~1.2deg reads as "stalled" too). Disabling
    // both leaves only SMALL (real success) and TIMEOUT (raw safety net) as exits, so the next log
    // gives a clean answer: does TURN_KI above actually close the gap before 2000ms, or not.
    const int    TURN_SMALL_EXIT_MS         = 150;
    const double TURN_BIG_TOLERANCE_DEG     = 0;
    const int    TURN_BIG_EXIT_MS           = 0;
    const double TURN_VELOCITY_DEG_PER_TICK = 0;
    const int    TURN_VELOCITY_EXIT_MS      = 0;

    drive.setTurnPID({TURN_KP, TURN_KI, TURN_KD, TURN_TIMEOUT_MS, TURN_START_I});
    drive.setTurnExit({TURN_SMALL_EXIT_MS, TURN_BIG_TOLERANCE_DEG, TURN_BIG_EXIT_MS,
                        TURN_VELOCITY_DEG_PER_TICK, TURN_VELOCITY_EXIT_MS});
    drive.resetPose();

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

// Drives a square (four legs with 90-degree turns between), returning near the start. Straight
// legs alone can't exercise updatePose()'s lateral-coupling terms; physically measuring the
// robot's actual end position/heading against the logged pose after this run is how the fused
// odometry gets validated (there's no sensor-independent ground truth the firmware can check
// itself against).
void odomSquareTest() {
    const double SIDE_IN  = 24.0;
    const double TURN_DEG = 90.0;
    const int    PAUSE_MS = 500;

    // Drive PID + tiered exit - the values validated in pidTuneTest(). Set explicitly here
    // (instead of relying on whatever the last-run auton left in the shared drive object)
    // so this test behaves the same regardless of run order or a cold boot.
    const double DRIVE_KP                   = 7.0;    // odom_square_2.csv: 6.0 gave a clean, fast,
                                                       // no-overshoot P response from 24in down to
                                                       // ~2.5in - bumped slightly for more sustained
                                                       // authority as the integral hands off, low
                                                       // overshoot risk since this region never
                                                       // saturated DRIVE_MAX_SPEED.
    const double DRIVE_KI                   = 0.5;
    const double DRIVE_KD                   = 0.2;
    const double DRIVE_TIMEOUT_MS           = 3500.0; // odom_square_2.csv: 3000 cut two of four legs
                                                       // off via TIMEOUT within ~1.5in of tolerance;
                                                       // bumped for margin while retuning, not itself
                                                       // a fix for the slow approach.
    const double DRIVE_START_I              = 2.5;
    const double DRIVE_INTEGRAL_CLAMP       = 24.0;   // odom_square_1.csv: unclamped, integral wound
                                                       // up to ~37 of output by the time the drive
                                                       // was already past target (pure P there would
                                                       // be ~-3) - visible as "slow down, then extra
                                                       // juice" and a slight overshoot past the 0.5in
                                                       // tolerance. odom_square_2.csv: clamping to 15
                                                       // fixed the overshoot but overcorrected - once
                                                       // error dropped under DRIVE_START_I the clamped
                                                       // integral held output constant at 15 while the
                                                       // shrinking kP term kept dropping, so commanded
                                                       // power actually *fell* on approach and all four
                                                       // legs crawled into TIMEOUT/BIG instead of a
                                                       // clean SMALL settle. 24 splits the difference
                                                       // between the two runs.
    const int    DRIVE_MAX_SPEED            = 90;     // 0-127
    const double DRIVE_TOLERANCE_IN         = 0.5;
    const int    DRIVE_SMALL_EXIT_MS        = 150;
    const double DRIVE_BIG_TOLERANCE_IN     = 1.5;
    const int    DRIVE_BIG_EXIT_MS          = 400;
    const double DRIVE_VELOCITY_IN_PER_TICK = 0.003;
    const int    DRIVE_VELOCITY_EXIT_MS     = 300;

    // Turn PID - the values validated in pidTurnTuneTest() (pid_turn_7.csv: both legs settled
    // under 1deg via the tight tolerance in ~1.1s, well inside the timeout, once kI was raised
    // and BIG/VELOCITY were disabled - see pidTurnTuneTest() for why BIG in particular can't be
    // trusted without its own bench pass). Turn max speed matches what was actually tuned at (70),
    // not the 90 used for straight legs.
    const double TURN_KP            = 3.0;
    const double TURN_KI            = 0.2;
    const double TURN_KD            = 0.3;
    const double TURN_TIMEOUT_MS    = 2000.0;
    const double TURN_START_I       = 5.0;
    const int    TURN_MAX_SPEED     = 70;
    const double TURN_TOLERANCE_DEG = 1.0;
    const int    TURN_SMALL_EXIT_MS = 150;

    drive.setDrivePID({DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_TIMEOUT_MS, DRIVE_START_I,
                       0, 0, 0, DRIVE_INTEGRAL_CLAMP});
    drive.setDriveExit({DRIVE_SMALL_EXIT_MS, DRIVE_BIG_TOLERANCE_IN, DRIVE_BIG_EXIT_MS,
                         DRIVE_VELOCITY_IN_PER_TICK, DRIVE_VELOCITY_EXIT_MS});
    drive.setTurnPID({TURN_KP, TURN_KI, TURN_KD, TURN_TIMEOUT_MS, TURN_START_I});
    drive.setTurnExit({TURN_SMALL_EXIT_MS, 0, 0, 0, 0});
    drive.resetPose();

    static volatile int currentLeg;
    currentLeg = 0;   // 1-4 = drive legs, 5-8 = turns between them

    warbots::CsvLogger logger("odom_square", fullTelemetryColumns(currentLeg, drive.getHorizontalTrackerInches()));
    logger.start();

    for (int side = 0; side < 4; side++) {
        // Absolute heading this side of the square should be driven/turned to (0/90/180/270) -
        // anchoring both the drive leg's heading hold and the turn to this fixed reference (instead
        // of each leg just holding/adding onto whatever the previous leg happened to land on) is
        // what stops per-turn overshoot from compounding across the square - see odom_square_7.csv,
        // where four uncorrected relative +90 turns drifted to +7.9deg by the last leg.
        double legHeading = side * TURN_DEG;

        currentLeg = side * 2 + 1;
        drive.PID_driveInches(SIDE_IN, DRIVE_MAX_SPEED, DRIVE_TOLERANCE_IN, HeadingHoldMode::IMU_HEADING,
                               0.4, 0.3, legHeading);
        pros::delay(PAUSE_MS);

        currentLeg = side * 2 + 2;
        drive.PID_turnToHeading(legHeading + TURN_DEG, TURN_MAX_SPEED, TURN_TOLERANCE_DEG);
        pros::delay(PAUSE_MS);
    }

    logger.stop();
}
