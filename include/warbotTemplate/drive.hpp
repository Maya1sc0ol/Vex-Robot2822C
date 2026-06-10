#pragma once
#include "warbotTemplate/pid.hpp"
#include "warbotTemplate/util.hpp"
#include "main.h"


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

void setMirrored(bool m) {
    mirrored = m;
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
    int leftx = controller.get_analog(ANALOG_LEFT_X);
    int lefty = controller.get_analog(ANALOG_LEFT_Y);
    int rightx = controller.get_analog(ANALOG_RIGHT_X);
    int righty = controller.get_analog(ANALOG_RIGHT_Y);
    
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

// Drive a set distance in inches using the PID config set via setDrivePID().
// inches        : target forward distance; negative = backward.
// maxSpeed      : motor power cap, 0..127.
// holdHeading   : if true, apply proportional correction to keep the starting heading.
// driveTolerance: exit when within this many inches of the target (default 0.5").
// driveHeadingKp: proportional gain for straight-drive heading correction (default 2.0).
void PID_driveInches(double inches, int maxSpeed = 127,
    double driveTolerance = 0.5, bool holdHeading = false, 
    double driveHeadingKp = 2.0) {
        // Reset PID state so each call starts fresh.
        drivePIDConfig.prev_error = 0;
        drivePIDConfig.integral   = 0;
        
        // Snapshot starting pose and time.
        double x0   = pose.x;
        double y0   = pose.y;
        double a0   = pose.angle;
        double a0_rad = a0 * (M_PI / 180.0);
        uint32_t startTime = pros::millis();
        
        while (true) {
            // Timeout check.
            if (pros::millis() - startTime >= (uint32_t)drivePIDConfig.timeout) break;
            
            // Update odometry (honours whatever odomConfig was set in main).
            updatePose();
            
            // Project pose displacement onto the starting forward vector.
            double traveled = (pose.x - x0) * std::sin(a0_rad)
            + (pose.y - y0) * std::cos(a0_rad);
            
            // Tolerance check.
            if (std::fabs(inches - traveled) < driveTolerance) break;
            
            // PID output and clamp to max speed.
            double output = calculatePID(traveled, inches, drivePIDConfig);
            output = std::max(-(double)maxSpeed, std::min((double)maxSpeed, output));
            
            // Optional heading correction: steer back to the starting angle.
            double correction = holdHeading ? driveHeadingKp * wrap180(a0 - pose.angle) : 0.0;
            
            moveLeftSide ((int)(output - correction));
            moveRightSide((int)(output + correction));
            
            pros::delay(10);
        }
        
        // Stop both sides when done.
        moveLeftSide(0);
        moveRightSide(0);
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
            //Updates the pose
            updatePose();
            double traveled = (pose.x - startingX) * std::sin(startingAngleR)
            + (pose.y - startingY) * std::cos(startingAngleR);
            
            //checks if traveled distance is greater than exitCondition
            if(std::fabs(inches-traveled) < exitRangeIn){
                break;
            }
            
            double correction = holdHeading ? holdHeadingkp * wrap180(startingAngle - pose.angle) : 0.0;
            
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
        turnPIDConfig.integral   = 0;
        
        if (mirrored) degrees = -degrees;
        
        double a0 = pose.angle;
        uint32_t startTime = pros::millis();
        
        while (true) {
            if (pros::millis() - startTime >= (uint32_t)turnPIDConfig.timeout) break;
            
            updatePose();
            
            // Signed heading change since the start, wrapped to [-180, 180].
            double turned = wrap180(pose.angle - a0);
            
            if (std::fabs(degrees - turned) < turnTolerance) break;
            
            double output = calculatePID(turned, degrees, turnPIDConfig);
            output = std::max(-(double)maxSpeed, std::min((double)maxSpeed, output));
            
            // Positive output = CW: right side forward, left side backward.
            moveLeftSide (-(int)output);
            moveRightSide( (int)output);
            
            pros::delay(10);
        }
        
        moveLeftSide(0);
        moveRightSide(0);
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
