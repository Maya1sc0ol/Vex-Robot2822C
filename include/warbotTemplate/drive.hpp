#pragma once
#include "warbotTemplate/pid.hpp"
#include "warbotTemplate/util.hpp"
#include "main.h"
#include <cmath>
#include <limits>


using namespace warbots;
namespace warbots{
class Drive {
public:

//Constructor
// ratio = driving gear / driven gear (motor gear teeth / wheel gear teeth)
//   Direct drive = 1.  Example: 12T motor -> 36T wheel = 12.0/36.0 = 0.333
//   The odometry formula multiplies by this ratio because the encoder sits on the motor (driving) shaft.
Drive(std::vector<int> leftMotorPorts, std::vector<int> rightMotorPorts, double wheelDiameter, double ratio = 1.0, bool useImu = false, int imuPort = 0)
    : usingImu(useImu), storedImuPort(imuPort), storedWheelDiameter(wheelDiameter), storedGearRatio(ratio)
{
    for (int port : leftMotorPorts) {
        leftMotors.emplace_back(port);
    }
    for (int port : rightMotorPorts) {
        rightMotors.emplace_back(port);
    }
}
//This is to hold the Robot Pose
struct robotPose{
    double x;
    double y;
    double angle;
};

// Read the robot's current position (x = right inches, y = forward inches, angle = degrees CW from forward)
const robotPose& getPose() const {
    return pose;
}

// --- Diagnostic getters (for logging/tuning; average across each side's motors) ---
double getLeftVelocity()  { double v = 0; for (auto& m : leftMotors)  v += m.get_actual_velocity(); return v / leftMotors.size(); }
double getRightVelocity() { double v = 0; for (auto& m : rightMotors) v += m.get_actual_velocity(); return v / rightMotors.size(); }
double getLeftCurrent()   { double c = 0; for (auto& m : leftMotors)  c += m.get_current_draw();     return c / leftMotors.size(); }
double getRightCurrent()  { double c = 0; for (auto& m : rightMotors) c += m.get_current_draw();     return c / rightMotors.size(); }
double getHeading()       { return imu.has_value() ? imu->get_heading() : 0.0; }
// Raw horizontal tracking wheel position in inches (does not consume odometry's prev-value state,
// so it's safe to sample from tuning/logging code without disturbing updatePose()).
double getHorizontalTrackerInches() {
    if (!horizontalTracker.has_value()) return 0.0;
    return (horizontalTracker->get_position() / 36000.0) * M_PI * horizontalTrackerDiameter;
}
// Raw horizontal tracking wheel velocity in inches/sec (side-effect-free, like getHorizontalTrackerInches()).
double getHorizontalTrackerVelocity() {
    if (!horizontalTracker.has_value()) return 0.0;
    return (horizontalTracker->get_velocity() / 36000.0) * M_PI * horizontalTrackerDiameter;
}

// --- Per-motor diagnostics (for logging; each side has multiple physical motors that can
// individually stall/slip/overheat in ways a side-average can mask) ---
struct MotorTelemetry {
    double velocity = 0, current = 0, voltage = 0, temperature = 0, power = 0, torque = 0, efficiency = 0, position = 0;
};
MotorTelemetry getLeftMotorTelemetry(int index)  { return motorTelemetry(leftMotors, index); }
MotorTelemetry getRightMotorTelemetry(int index) { return motorTelemetry(rightMotors, index); }

// --- Full IMU diagnostics (for logging) ---
double getImuRotation() { return imu.has_value() ? imu->get_rotation() : 0.0; }  // continuous, unwrapped
double getImuPitch()    { return imu.has_value() ? imu->get_pitch()    : 0.0; }
double getImuRoll()     { return imu.has_value() ? imu->get_roll()     : 0.0; }
double getImuYaw()      { return imu.has_value() ? imu->get_yaw()      : 0.0; }
pros::imu_gyro_s_t  getImuGyroRate() { return imu.has_value() ? imu->get_gyro_rate() : pros::imu_gyro_s_t{0, 0, 0}; }
pros::imu_accel_s_t getImuAccel()    { return imu.has_value() ? imu->get_accel()     : pros::imu_accel_s_t{0, 0, 0}; }

// --- Last PID_driveInches()/PID_turnDegrees() output/correction (for diagnosing whether a
// divergence was PID-commanded/saturated vs. caused by something the controller didn't ask for) ---
double getLastDriveOutput()     const { return lastDriveOutput; }
double getLastDriveCorrection() const { return lastDriveCorrection; }
double getLastTurnOutput()      const { return lastTurnOutput; }
double getLastTurnCorrection()  const { return lastTurnCorrection; }
// Raw remaining error (target - current) each function was closing, in inches/degrees
// respectively - lets telemetry distinguish "oscillating around target" from "slowly creeping
// in" without having to recompute it from pose snapshots.
double getLastDriveError()      const { return lastDriveError; }
double getLastTurnError()       const { return lastTurnError; }

// Why PID_driveInches()/PID_turnDegrees() last exited its loop (for diagnosing whether it
// actually settled vs. got cut off by the raw timeout - see ExitConditions in pid.hpp).
enum class ExitReason { RUNNING, TIMEOUT, SMALL, BIG, VELOCITY, ABORT };
ExitReason getLastDriveExitReason() const { return lastDriveExitReason; }
ExitReason getLastTurnExitReason()  const { return lastTurnExitReason; }

//This function prints out the robots pose on the Brain Screen
void testingPose(){
std::string x = warbots::doubleToString(pose.x, 2);
std::string y = warbots::doubleToString(pose.y, 2);
std::string rotation = warbots::doubleToString(pose.angle, 2);
warbots::screenPrint("x:"+ x, 2);
warbots::screenPrint("y:"+ y, 2);
warbots::screenPrint("R:"+ rotation, 2);
}
//TrackWidth is the distance between the center of the left wheel to the center of the right wheel
void setTrackWidth(double inches) { 
    trackWidth = inches; 
}
//This is to configure the IMU if you are using one
void initImu(){
    if (usingImu) {
        imu.emplace(storedImuPort);
    }
}
//This is to configure the vertical Tracking Wheel if you have one
void addVerticalTrackingWheel(int port, double wheelDiameter){
    verticalTracker.emplace(port);
    verticalTracker->reset_position();
    verticalTrackerDiameter = wheelDiameter;
}

//This is to configure the vertical Tracking Wheel if you have one
void addHorizontalTrackingWheel(int port, double wheelDiameter){
    horizontalTracker.emplace(port);
    horizontalTracker->reset_position();
    horizontalTrackerDiameter = wheelDiameter;
}

/* ODOMETRY CONFIG---------------------------------------
Selects which sensors are used to track the robot's position.
Set with setOdomConfig(). Defaults to MOTOR_ENCODERS.
*/
enum odomConfig {
    MOTOR_ENCODERS     = 0,
    IMU_ONLY           = 1,
    VERTICAL_TRACKER   = 2,
    HORIZONTAL_TRACKER = 3,
    BOTH_TRACKERS      = 4,
    IMU_VERTICAL       = 5,
    IMU_HORIZONTAL     = 6,
    IMU_BOTH_TRACKERS  = 7
};

void setOdomConfig(odomConfig config){
    currentOdomConfig = config;
}

void setDrivePID(PIDconfigs config) {
    drivePIDConfig = config;
}

void setTurnPID(PIDconfigs config) {
    turnPIDConfig = config;
}

void setDriveExit(ExitConditions e) {
    driveExit = e;
}

void setTurnExit(ExitConditions e) {
    turnExit = e;
}

void setMirrored(bool m) {
    mirrored = m;
}

// Scales all driver-control output (see control()) by this factor, 0.0-1.0.
// Useful for capping top speed based on robot state (e.g. arm height).
void setSpeedScale(double scale) {
    speedScale = scale;
}

// Reset pose and sync all previous-value state so the first updatePose() call produces zero deltas.
void resetPose(double x = 0, double y = 0, double angle = 0) {
    pose = {x, y, angle};
    prevImuHeading = imu.has_value() ? imu->get_heading() : 0;

    double encToInches = (M_PI * storedWheelDiameter * storedGearRatio) / 360.0;
    double leftEnc = 0;
    double rightEnc = 0;
    for (auto& m : leftMotors)  {
        leftEnc  += m.get_position();
    }
    for (auto& m : rightMotors) {
        rightEnc += m.get_position();
    }
    prevLeftInches  = (leftEnc  / leftMotors.size())  * encToInches;
    prevRightInches = (rightEnc / rightMotors.size()) * encToInches;

    if (verticalTracker.has_value()){
        prevVerticalInches = (verticalTracker->get_position() / 36000.0) * M_PI * verticalTrackerDiameter;
    }
    if (horizontalTracker.has_value()){
        prevHorizontalInches = (horizontalTracker->get_position() / 36000.0) * M_PI * horizontalTrackerDiameter;
    }
}

/* updatePose() — call every 10 ms from a pros::Task.
   Coordinate system: +X = right, +Y = forward, angle 0 = facing forward, clockwise positive.
   Each case reads only the sensors required by the active odomConfig.
*/
void updatePose() {
    double deltaTheta_deg = 0;
    double deltaForward = 0;
    double deltaLateral = 0;

    switch (currentOdomConfig) {
        case MOTOR_ENCODERS: {
            auto [dL, dR] = motorDeltas();
            deltaForward = (dL + dR) / 2.0;
            if (trackWidth > 0) {
                deltaTheta_deg = (dR - dL) / trackWidth * (180.0 / M_PI);
            }
            break;
        }
        case IMU_ONLY: {
            auto [dL, dR] = motorDeltas();
            deltaForward = (dL + dR) / 2.0;
            deltaTheta_deg = imuDelta();
            break;
        }
        case VERTICAL_TRACKER: {
            deltaForward = verticalDelta();
            auto [dL, dR] = motorDeltas();
            if (trackWidth > 0) deltaTheta_deg = (dR - dL) / trackWidth * (180.0 / M_PI);
            break;
        }
        case HORIZONTAL_TRACKER: {
            auto [dL, dR] = motorDeltas();
            deltaForward = (dL + dR) / 2.0;
            if (trackWidth > 0) deltaTheta_deg = (dR - dL) / trackWidth * (180.0 / M_PI);
            deltaLateral = horizontalDelta();
            break;
        }
        case BOTH_TRACKERS: {
            deltaForward = verticalDelta();
            deltaLateral = horizontalDelta();
            auto [dL, dR] = motorDeltas();
            if (trackWidth > 0) deltaTheta_deg = (dR - dL) / trackWidth * (180.0 / M_PI);
            break;
        }
        case IMU_VERTICAL:
            deltaForward = verticalDelta();
            deltaTheta_deg = imuDelta();
            break;

        case IMU_HORIZONTAL: {
            auto [dL, dR] = motorDeltas();
            deltaForward = (dL + dR) / 2.0;
            deltaLateral = horizontalDelta();
            deltaTheta_deg = imuDelta();
            break;
        }

        case IMU_BOTH_TRACKERS:
            deltaForward = verticalDelta();
            deltaLateral = horizontalDelta();
            deltaTheta_deg = imuDelta();
            break;

    }

    double avgAngle_rad = (pose.angle + deltaTheta_deg / 2.0) * (M_PI / 180.0);
    pose.x += (deltaForward * std::sin(avgAngle_rad)) + (deltaLateral * std::cos(avgAngle_rad));
    pose.y += (deltaForward * std::cos(avgAngle_rad)) - (deltaLateral * std::sin(avgAngle_rad));
    pose.angle += deltaTheta_deg;

    while (pose.angle >= 360.0) {
        pose.angle -= 360.0;
    }
    while (pose.angle <    0.0) {
        pose.angle += 360.0;
    }
}

/* DRIVER CONTROL---------------------------------------
This code is used to let the user pick the type of driving they would like to use
The user can choose from 5 different configurations:

Tank drive: left joystick controls the left side of the drivetrain and the right joystick controls the right side of the drivetrain
Single Arcade: The left joystick controls both turning and driving
Flipped Single Arcade: The right joystick controls both turning and driving
Split Arcade: The left joystick controls forward and backwards, the right joystick controls turing left and right
Flipped Split Arcade: the left joystick controls turning left and right, the right joystick controls driving forward and back

*/
enum driveControlType{
    TANK = 0,
    SINGLE_ARCADE = 1,
    FLIPPED_SINGLE_ARCADE = 2,
    SPLIT_ARCADE = 3,
    FLIPPED_SPLIT_ARCADE = 4
};

void setDriveType(driveControlType type){
    currentControlType = type;
}

/* SWING TURN---------------------------------------
Selects which side of the drivetrain stays locked during a swing turn.
The locked side is held in place while the other side drives, so the robot
pivots around the stationary wheel.
*/


void control(pros::Controller& controller){
    int leftx = (int)(controller.get_analog(ANALOG_LEFT_X) * speedScale);
    int lefty = (int)(controller.get_analog(ANALOG_LEFT_Y) * speedScale);
    int rightx = (int)(controller.get_analog(ANALOG_RIGHT_X) * speedScale);
    int righty = (int)(controller.get_analog(ANALOG_RIGHT_Y) * speedScale);

    switch (currentControlType) {
        case TANK:
        moveLeftSide(lefty);
        moveRightSide(righty);
        break;
        
        case SINGLE_ARCADE:
        moveLeftSide(lefty + leftx);
        moveRightSide(lefty - leftx);
        break;
        
        case FLIPPED_SINGLE_ARCADE:
        moveLeftSide(righty + rightx);
        moveRightSide(righty - rightx);
        break;
        
        case SPLIT_ARCADE:
        moveLeftSide(lefty + rightx);
        moveRightSide(lefty - rightx);
        break;
        
        case FLIPPED_SPLIT_ARCADE:
        moveLeftSide(righty + leftx);
        moveRightSide(righty - leftx);
        break;
    }
}
/* Autonomous Functions
This code is used to control robots driving and turning in Auton
There is PID, Continuous Movement and more!!
*/

// Which signal (if any) PID_driveInches() uses to steer straight.
enum class HeadingHoldMode {
    NONE,         // pure motor-encoder distance PID, no lateral/heading feedback
    IMU_HEADING,  // correct against IMU-derived pose.angle drift from the starting heading
    ODOM_LATERAL  // correct against the horizontal tracker's raw lateral drift, driven toward zero
};

// Drive a set distance in inches using the PID config set via setDrivePID().
// inches            : target forward distance; negative = backward.
// maxSpeed          : motor power cap, 0..127.
// holdMode          : NONE = encoders only, IMU_HEADING = hold starting heading via IMU,
//                     ODOM_LATERAL = hold zero lateral drift via the horizontal tracking wheel.
// driveTolerance    : exit when within this many inches of the target (default 0.5").
// driveHeadingKp    : proportional gain for straight-drive correction (default 2.0).
// driveRampInPerTick: max inches the PID's setpoint may advance per 10ms tick (default 0.3, i.e.
//                     a 30in/s ramp rate) - see rampedGoal below.
void PID_driveInches(double inches, int maxSpeed = 127,
    double driveTolerance = 0.5, HeadingHoldMode holdMode = HeadingHoldMode::NONE,
    double driveHeadingKp = 0.4, double driveRampInPerTick = 0.3,
    double headingTarget = std::numeric_limits<double>::quiet_NaN()) {
        // Reset PID state so each call starts fresh.
        drivePIDConfig.prev_error = 0;
        drivePIDConfig.prev_measurement = 0;
        drivePIDConfig.integral   = 0;

        // Snapshot starting pose and time.
        double x0   = pose.x;
        double y0   = pose.y;
        double a0   = pose.angle;
        double a0_rad = a0 * (M_PI / 180.0);
        // IMU_HEADING holds this heading, not necessarily a0. Default (NaN) preserves the old
        // behavior of holding wherever the robot happened to be pointed when this call started -
        // if the caller knows the leg's true intended heading (e.g. 0/90/180/270 on a square),
        // passing it here lets this leg actively steer back to it instead of faithfully preserving
        // whatever residual error the previous turn left behind. a0/a0_rad above stay the *actual*
        // starting heading regardless - they project real displacement onto the direction the robot
        // is actually driving, which must not be swapped for the intended target.
        double headingHoldTarget = std::isnan(headingTarget) ? a0 : headingTarget;
        double lateral0 = (holdMode == HeadingHoldMode::ODOM_LATERAL) ? getHorizontalTrackerInches() : 0.0;
        uint32_t startTime = pros::millis();

        // A fresh call hands calculatePID() the *entire* target as error on tick one, so
        // kP*error saturates straight to maxSpeed in a single 10ms step - a step-function torque
        // command. odom_square_5.csv showed this landing as a real, repeatable ~5 degree chassis
        // pitch (front wheels lifting) within the first ~150-300ms of every drive leg, exactly
        // coincident with the wheels going 0->100in/s in 31-48ms. Same fix as armGoTo()'s
        // rampedGoal in example.cpp/main.cpp: ease the setpoint in gradually instead of slamming
        // the raw output, so early-tick error (and therefore commanded torque) grows in rather
        // than snapping to max. Re-check imu_pitch/gyro_y in the next log to confirm/retune.
        double rampedGoal = 0.0;

        // Tiered-exit state (see ExitConditions in pid.hpp) - local to this call since
        // PID_driveInches is a single reentrant blocking call with no persistent state between calls.
        const int kLoopDelayMs = 10;
        int smallTimer = 0, bigTimer = 0, velTimer = 0;
        double prevTraveled = 0;
        lastDriveExitReason = ExitReason::RUNNING;

        while (true) {
            // Timeout check - the final safety net if nothing below ever settles.
            if (pros::millis() - startTime >= (uint32_t)drivePIDConfig.timeout) {
                lastDriveExitReason = ExitReason::TIMEOUT;
                break;
            }
            if (checkAutonAbort()) {
                lastDriveExitReason = ExitReason::ABORT;
                break;
            }

            // Update odometry (honours whatever odomConfig was set in main).
            updatePose();

            // Project pose displacement onto the starting forward vector.
            double traveled = (pose.x - x0) * std::sin(a0_rad)
            + (pose.y - y0) * std::cos(a0_rad);
            double error = inches - traveled;
            lastDriveError = error;

            // Tight tolerance - must hold for smallExitMs before exiting (0 = exit immediately,
            // same as the old single-tick check).
            if (std::fabs(error) < driveTolerance) {
                smallTimer += kLoopDelayMs;
                if (smallTimer >= driveExit.smallExitMs) {
                    lastDriveExitReason = ExitReason::SMALL;
                    break;
                }
            } else {
                smallTimer = 0;
            }

            // Looser fallback tolerance, so a "close enough" move doesn't have to wait out the
            // full raw timeout to finish.
            if (driveExit.bigError > 0) {
                if (std::fabs(error) < driveExit.bigError) {
                    bigTimer += kLoopDelayMs;
                    if (bigTimer >= driveExit.bigExitMs) {
                        lastDriveExitReason = ExitReason::BIG;
                        break;
                    }
                } else {
                    bigTimer = 0;
                }
            }

            // Stall exit - catches a P(+D)-only controller creeping too slowly (or stopped
            // dead against static friction) to ever satisfy a tolerance before the raw timeout.
            if (driveExit.velocityError > 0) {
                double velocity = traveled - prevTraveled;
                if (std::fabs(velocity) < driveExit.velocityError) {
                    velTimer += kLoopDelayMs;
                    if (velTimer >= driveExit.velocityExitMs) {
                        lastDriveExitReason = ExitReason::VELOCITY;
                        break;
                    }
                } else {
                    velTimer = 0;
                }
            }
            prevTraveled = traveled;

            // Advance the ramped setpoint toward the real target, then feed *that* to the PID
            // instead of `inches` directly (see rampedGoal comment above).
            double rampStep = std::max(-driveRampInPerTick,
                std::min(driveRampInPerTick, inches - rampedGoal));
            rampedGoal += rampStep;

            // PID output and clamp to max speed.
            double output = calculatePID(traveled, rampedGoal, drivePIDConfig, /*derivativeOnMeasurement=*/true);
            output = std::max(-(double)maxSpeed, std::min((double)maxSpeed, output));

            // Optional straight-line correction, from whichever signal holdMode selects.
            double correction = 0.0;
            if (holdMode == HeadingHoldMode::IMU_HEADING) {
                correction = -driveHeadingKp * wrap180(headingHoldTarget - pose.angle);
            } else if (holdMode == HeadingHoldMode::ODOM_LATERAL) {
                correction = driveHeadingKp * (getHorizontalTrackerInches() - lateral0);
            }
            // Clamp like output - without this, a growing heading error can produce a
            // correction that completely overwhelms output, spinning the robot instead of
            // just steering it straight.
            correction = std::max(-(double)maxSpeed, std::min((double)maxSpeed, correction));
            lastDriveOutput = output;
            lastDriveCorrection = correction;

            moveLeftSide ((int)(output - correction));
            moveRightSide((int)(output + correction));

            pros::delay(kLoopDelayMs);
        }

        // Actively brake to a stop (instead of just cutting power and coasting), then release
        // back to coast so a later joystick/PID call isn't fighting a locked drivetrain.
        for (auto& m : leftMotors)  m.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        for (auto& m : rightMotors) m.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        moveLeftSide(0);
        moveRightSide(0);
        pros::delay(kLoopDelayMs);
        for (auto& m : leftMotors)  m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        for (auto& m : rightMotors) m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    //Moves the robot using a maintained velocity instead of using PID
    /*
    This function takes in:
    inches: how many inches you want to drive
    speed: the speed that is maintained the entire time that the robot is driving
    exitRangeIn: How far you want to be from the goal to exit the function
    holdHeading: do you want angle correction while driving?
    holdHeadingkp: if you are using hold heading, the kP for holdHeading
    */
   void continuousDrive(double inches, int speed, double exitRangeIn, 
    bool holdHeading, double holdHeadingkp){
        drivePIDConfig.prev_error = 0;
        drivePIDConfig.integral   = 0;
        
        double startingX = pose.x;
        double startingY = pose.y;
        double startingAngle = pose.angle;
        double startingAngleR = startingAngle * (M_PI/180.0);
        
        uint32_t startTime = pros::millis();
        
        while(true){
            //Checking if function has been longer than timeout
            if(pros::millis() - startTime >= (uint32_t)drivePIDConfig.timeout) {
                break;
            }
            if (checkAutonAbort()) break;
            //Updates the pose
            updatePose();
            double traveled = (pose.x - startingX) * std::sin(startingAngleR)
            + (pose.y - startingY) * std::cos(startingAngleR);
            
            //checks if traveled distance is greater than exitCondition
            if(std::fabs(inches-traveled) < exitRangeIn){
                break;
            }
            
            double correction = holdHeading ? -holdHeadingkp * wrap180(startingAngle - pose.angle) : 0.0;
            
            moveLeftSide((int)speed - correction);
            moveRightSide((int)speed + correction);
            
            pros::delay(10);
        }
        
    }
    
    // Turn a set number of degrees using the PID config set via setTurnPID().
    // degrees      : how much to turn; positive = CW, negative = CCW.
    // maxSpeed     : motor power cap, 0..127.
    // turnTolerance: exit when within this many degrees of the target (default 1.0°).
    void PID_turnDegrees(double degrees, int maxSpeed = 127, double turnTolerance = 1.0) {
        turnPIDConfig.prev_error = 0;
        turnPIDConfig.prev_measurement = 0;
        turnPIDConfig.integral   = 0;

        if (mirrored) degrees = -degrees;

        // A point turn has ~zero real translation, so any horizontal-tracker reading during
        // one is scrub/offset artifact (the tracker isn't mounted exactly at the pivot), not
        // real lateral motion - odom_square_3.csv showed 14-17in of spurious pose displacement
        // per 90deg turn under IMU_HORIZONTAL, even though the raw drive encoders stayed
        // symmetric (confirming it really was a pure pivot). Drop to IMU_ONLY for the turn's
        // duration so heading still comes from the IMU but the tracker can't corrupt pose.x/y,
        // then restore whatever odomConfig the caller had set.
        odomConfig savedOdomConfig = currentOdomConfig;
        currentOdomConfig = odomConfig::IMU_ONLY;

        double a0 = pose.angle;
        uint32_t startTime = pros::millis();

        const int kLoopDelayMs = 10;
        int smallTimer = 0, bigTimer = 0, velTimer = 0;
        double prevTurned = 0;
        lastTurnExitReason = ExitReason::RUNNING;

        while (true) {
            if (pros::millis() - startTime >= (uint32_t)turnPIDConfig.timeout) {
                lastTurnExitReason = ExitReason::TIMEOUT;
                break;
            }
            if (checkAutonAbort()) {
                lastTurnExitReason = ExitReason::ABORT;
                break;
            }

            updatePose();

            // Signed heading change since the start, wrapped to [-180, 180].
            double turned = wrap180(pose.angle - a0);
            double error = degrees - turned;
            lastTurnError = error;

            if (std::fabs(error) < turnTolerance) {
                smallTimer += kLoopDelayMs;
                if (smallTimer >= turnExit.smallExitMs) {
                    lastTurnExitReason = ExitReason::SMALL;
                    break;
                }
            } else {
                smallTimer = 0;
            }

            if (turnExit.bigError > 0) {
                if (std::fabs(error) < turnExit.bigError) {
                    bigTimer += kLoopDelayMs;
                    if (bigTimer >= turnExit.bigExitMs) {
                        lastTurnExitReason = ExitReason::BIG;
                        break;
                    }
                } else {
                    bigTimer = 0;
                }
            }

            if (turnExit.velocityError > 0) {
                double velocity = turned - prevTurned;
                if (std::fabs(velocity) < turnExit.velocityError) {
                    velTimer += kLoopDelayMs;
                    if (velTimer >= turnExit.velocityExitMs) {
                        lastTurnExitReason = ExitReason::VELOCITY;
                        break;
                    }
                } else {
                    velTimer = 0;
                }
            }
            prevTurned = turned;

            double output = calculatePID(turned, degrees, turnPIDConfig, /*derivativeOnMeasurement=*/true);
            output = std::max(-(double)maxSpeed, std::min((double)maxSpeed, output));
            lastTurnOutput = output;
            lastTurnCorrection = 0.0;  // no separate steering term for a point-turn

            // Positive output = CW: left side forward, right side backward. (Matches pose.angle's
            // CW-positive convention, fed by the IMU - see updatePose()/imuDelta(). This mapping
            // was previously flipped, which drove the robot CCW for positive error and caused a
            // sign-inverted feedback loop: pose.angle moved the opposite way the controller
            // expected, so error grew without bound regardless of gains - see SPLIT_ARCADE in
            // control() for the same CW convention done correctly.)
            moveLeftSide ( (int)output);
            moveRightSide(-(int)output);

            pros::delay(kLoopDelayMs);
        }

        currentOdomConfig = savedOdomConfig;

        // The tracking wheels kept physically scrubbing through the turn (a point turn isn't a
        // true pivot, so they read real but non-representative distance) while verticalDelta()/
        // horizontalDelta() weren't being called to consume it. Re-sync their prev*Inches
        // baselines now so that backlog gets discarded instead of landing as one huge spurious
        // deltaForward/deltaLateral on savedOdomConfig's next updatePose() call - see
        // odom_square_4.csv, which showed the IMU_ONLY swap above just relocating the ~17-18in
        // spurious pose jump from "smeared across the turn" to "dumped in one frame at the start
        // of the next drive leg" instead of eliminating it.
        verticalDelta();
        horizontalDelta();

        // Actively brake to a stop, then release back to coast (see PID_driveInches for why).
        for (auto& m : leftMotors)  m.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        for (auto& m : rightMotors) m.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        moveLeftSide(0);
        moveRightSide(0);
        pros::delay(kLoopDelayMs);
        for (auto& m : leftMotors)  m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        for (auto& m : rightMotors) m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    // Point-turn to an ABSOLUTE heading (unlike PID_turnDegrees, which is relative to wherever
    // the robot happens to be pointed when called). Computes the shortest-path delta from the
    // current pose.angle and drives that through the existing point-turn PID, so each turn
    // self-corrects toward the true target instead of compounding onto whatever the previous
    // turn's overshoot left behind - odom_square_7.csv showed four relative +90 turns drifting
    // to +7.9deg by the end of the square instead of closing back to 0.
    // Known limitation: does not account for `mirrored` - PID_turnDegrees applies its own mirror
    // flip internally, so calling this under setMirrored(true) would double-flip the delta. No
    // current caller combines the two (redLeft(), the only mirrored routine, doesn't turn).
    void PID_turnToHeading(double targetAngle, int maxSpeed = 127, double turnTolerance = 1.0) {
        double delta = wrap180(targetAngle - pose.angle);
        PID_turnDegrees(delta, maxSpeed, turnTolerance);
    }

    enum swingSide {
        LOCK_LEFT  = 0,  // left side held, right side drives
        LOCK_RIGHT = 1   // right side held, left side drives
    };

    // Swing-turn to an ABSOLUTE heading by pivoting around one locked side.
    // targetAngle  : absolute heading in degrees (0 = forward, CW positive).
    // lockedSide   : which side stays still (LOCK_LEFT or LOCK_RIGHT).
    // maxSpeed     : motor power cap on the driving side, 0..127.
    // turnTolerance: exit when within this many degrees of target (default 1.0).
    void PID_swingToAngle(double targetAngle, swingSide lockedSide,
        int maxSpeed = 127, double turnTolerance = 1.0) {
    turnPIDConfig.prev_error = 0;
    turnPIDConfig.integral   = 0;

    if (mirrored) {
        targetAngle = -targetAngle;
        lockedSide  = (lockedSide == LOCK_LEFT) ? LOCK_RIGHT : LOCK_LEFT;
    }

    // Hold the locked side in place with brake mode; restore coast on exit.
    if (lockedSide == LOCK_LEFT) {
        for (auto& m : leftMotors)  {
            m.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
    } else {
        for (auto& m : rightMotors) {
            m.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
    }

    uint32_t startTime = pros::millis();

    while (true) {
        if (pros::millis() - startTime >= (uint32_t)turnPIDConfig.timeout) {
            break;
        }
        if (checkAutonAbort()) break;

        updatePose();

        // Shortest signed error to the absolute target heading, [-180,180].
        double error = wrap180(targetAngle - pose.angle);
        if (std::fabs(error) < turnTolerance) {
            break;
        } 

        double output = calculatePID(0.0, error, turnPIDConfig); // goal-current = error
        output = std::max(-(double)maxSpeed, std::min((double)maxSpeed, output));

        // Drive only the free side; hold the locked side at 0.
        if (lockedSide == LOCK_LEFT) {
            moveLeftSide(0);
            moveRightSide(-(int)output); // +error (need CW) -> right side backward
        } else {
            moveLeftSide((int)output);   // +error (need CW) -> left side forward
            moveRightSide(0);
        }

        pros::delay(10);
    }

    moveLeftSide(0);
    moveRightSide(0);

    // Restore coast so the held side doesn't stay locked for later moves.
    for (auto& m : leftMotors)  m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    for (auto& m : rightMotors) m.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

private:

// --- PID configs (set via setters) ---
PIDconfigs drivePIDConfig = {0, 0, 0, 3000};
PIDconfigs turnPIDConfig  = {0, 0, 0, 3000};
ExitConditions driveExit  = {};
ExitConditions turnExit   = {};

// --- Driver control speed cap (set via setSpeedScale()) ---
double speedScale = 1.0;

// --- Configuration (set at construction) ---
double storedWheelDiameter = 0;
double storedGearRatio = 1.0;  // driving / driven (motor teeth / wheel teeth); multiplied in encToInches
double trackWidth = 0;
bool usingImu = false;
int storedImuPort = 0;

// --- Hardware ---
std::vector<pros::Motor> leftMotors;
std::vector<pros::Motor> rightMotors;
std::optional<pros::Imu> imu;
std::optional<pros::Rotation> verticalTracker;
double verticalTrackerDiameter = 0;
std::optional<pros::Rotation> horizontalTracker;
double horizontalTrackerDiameter = 0;

// --- State ---
robotPose pose = {0.0, 0.0, 0.0};
bool mirrored = false;
pros::motor_brake_mode_e_t currentBrakeMode = pros::E_MOTOR_BRAKE_COAST;
int currentMA = 2500;
odomConfig currentOdomConfig = odomConfig::MOTOR_ENCODERS;
driveControlType currentControlType = SPLIT_ARCADE;

// --- Odometry tracking (previous values for delta calculations) ---
double prevLeftInches = 0;
double prevRightInches = 0;
double prevVerticalInches = 0;
double prevHorizontalInches = 0;
double prevImuHeading = 0;

// --- Last PID_driveInches()/PID_turnDegrees() output/correction (set each loop tick; see public getters) ---
double lastDriveOutput = 0;
double lastDriveCorrection = 0;
double lastTurnOutput = 0;
double lastTurnCorrection = 0;
double lastDriveError = 0;
double lastTurnError = 0;
ExitReason lastDriveExitReason = ExitReason::RUNNING;
ExitReason lastTurnExitReason  = ExitReason::RUNNING;

// Returns telemetry for one motor in the given side's vector; out-of-range index returns a zeroed struct.
MotorTelemetry motorTelemetry(std::vector<pros::Motor>& motors, int index) {
    if (index < 0 || index >= (int)motors.size()) return MotorTelemetry{};
    pros::Motor& m = motors[index];
    MotorTelemetry t;
    t.velocity    = m.get_actual_velocity();
    t.current     = m.get_current_draw();
    t.voltage     = m.get_voltage();
    t.temperature = m.get_temperature();
    t.power       = m.get_power();
    t.torque      = m.get_torque();
    t.efficiency  = m.get_efficiency();
    t.position    = m.get_position();
    return t;
}

// Returns {deltaLeft, deltaRight} in inches and advances the stored prev-values.
std::pair<double,double> motorDeltas() {
    double encToInches = (M_PI * storedWheelDiameter * storedGearRatio) / 360.0;
    double lEnc = 0, rEnc = 0;
    for (auto& m : leftMotors)  lEnc += m.get_position();
    for (auto& m : rightMotors) rEnc += m.get_position();
    double lIn = (lEnc / leftMotors.size())  * encToInches;
    double rIn = (rEnc / rightMotors.size()) * encToInches;
    double dL = lIn - prevLeftInches, dR = rIn - prevRightInches;
    prevLeftInches = lIn; prevRightInches = rIn;
    return {dL, dR};
}

// Returns heading delta in degrees (clockwise positive).
double imuDelta() {
    if (!imu.has_value()) {
        return 0;
    }
    double h = imu->get_heading();
    double raw = h - prevImuHeading;
    if (raw >  180) {
        raw -= 360;
    }
    if (raw < -180) {
        raw += 360;
    }
    prevImuHeading = h;
    return raw;
}

// Returns forward distance delta in inches from the vertical tracking wheel.
double verticalDelta() {
    if (!verticalTracker.has_value()) {
        return 0;
    }
    double vIn = (verticalTracker->get_position() / 36000.0) * M_PI * verticalTrackerDiameter;
    double d = vIn - prevVerticalInches;
    prevVerticalInches = vIn;
    return d;
}

// Returns lateral distance delta in inches from the horizontal tracking wheel (positive = rightward).
double horizontalDelta() {
    if (!horizontalTracker.has_value()) {
        return 0;
    }
    double hIn = (horizontalTracker->get_position() / 36000.0) * M_PI * horizontalTrackerDiameter;
    double d = hIn - prevHorizontalInches;
    prevHorizontalInches = hIn;
    return d;
}
// Normalise a heading difference to the range [-180, 180] degrees.
static double wrap180(double deg) {
    while (deg >  180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

//These make the robot sides drive.
void moveLeftSide(int power)  {
    for (auto& m : leftMotors) {
        m.move(power);
    }
}
void moveRightSide(int power) {
    for (auto& m : rightMotors) {
        m.move(power);
    }
}


}; // class Drive
} // namespace warbots
