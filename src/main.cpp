#include "main.h"
#include <algorithm>

// ==================== CONSTANTS ====================
// Tune everything here without digging through the rest of the code.

// Drive PID (used by PID_driveInches / continuousDrive)
const double DRIVE_KP = 0.6, DRIVE_KI = 0.0, DRIVE_KD = 0.2, DRIVE_TIMEOUT_MS = 3000.0;

// Turn PID (used by PID_turnDegrees / PID_swingToAngle)
const double TURN_KP = 5.0, TURN_KI = 0.0, TURN_KD = 0.0, TURN_TIMEOUT_MS = 2000.0;

// Arm PID (used by groupControl() every opcontrol loop)
const double ARM_KP = 1.0, ARM_KI = 0.01, ARM_KD = 0.1, ARM_TIMEOUT_MS = 2000.0;

// Arm setpoints, in encoder ticks. Index 0 MUST be 0: the arm starts folded
// all the way down at power-on, and arm.tare_position_all() (see initialize())
// zeros the encoder wherever it physically is at that moment - so "down" = 0
// by construction, and every other setpoint here is just "how far up from home."
const double ARM_POSITIONS[5] = {0, 800, 1400, 1900, 2400};

// Drivetrain speed cap vs. arm position - linear between these two points.
// Below ARM_SPEED_LIMIT_LOW_POS: full speed. Above ARM_SPEED_LIMIT_HIGH_POS: half speed.
const double ARM_SPEED_LIMIT_LOW_POS   = 0.0;     // arm position for full drive speed
const double ARM_SPEED_LIMIT_HIGH_POS  = 2400.0;  // arm position for half drive speed
const double DRIVE_SPEED_SCALE_AT_LOW  = 1.0;
const double DRIVE_SPEED_SCALE_AT_HIGH = 0.5;

// Precision driving: hold A to drop drive speed to this scale, overriding
// the arm-height-based scale above (for lining up precise scores/placements).
const double PRECISION_SPEED_SCALE = 0.3;

// Horizontal odom pod (tracks lateral/strafe drift). Rotation sensor on port 6.
const int    HORIZONTAL_TRACKER_PORT     = 6;
const double HORIZONTAL_TRACKER_DIAMETER = 2.75;  // inches

// TODO(distance-sensor slowdown): planned feature, not yet implemented.
// Add a Distance sensor (free port - NOT port 6, now used by the horizontal
// odom pod above) and, while held A / during scoring
// approach, scale drive speed down as the sensor's reading gets smaller (close
// to a target = slower). This must only be ACTIVE when the arm is at/above
// whatever setpoint index first lifts it out of the sensor's field of view -
// below that, the arm itself would false-trigger the sensor. We don't know
// that threshold arm position yet; measure it on hardware once the real arm
// heights/ARM_POSITIONS values are set, then gate this feature on it (e.g.
// only active when armPositionIndex >= that measured index).
// Also filter the raw sensor reading before trusting it: gate on
// get_confidence() (0-63; PROS notes confidence is only meaningful at closer
// range), take a median-of-3 (or short moving average) to reject single-frame
// spikes, clamp how fast the reading is allowed to change per loop tick, and
// on any rejected/low-confidence reading fail safe toward "far" (full speed)
// rather than "close" (slow/stop) - a bad reading should never freeze the robot.
// =====================================================

warbots::Drive drive(
	{-8, -3},  // Left Motors ID
	{5, 10},  // Right Motors ID
	3.25,  // Wheel Diameter
	450,   // Gear Ratio = driving gear / driven gear (motor gear teeth / wheel gear teeth)
	     // Direct drive = 1. Example: 12T motor gear -> 36T wheel gear = 12/36 = 0.333
	true,  // Are you using an IMU on the Robot?
	1   // If you are using an IMU, put the motor port here, if you are not using an IMU, leave at 0
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::Task logo_task([]() {
		while (true) {
			drawLogo();
			pros::delay(20);
			warbots::screenPrint("Warbot-Template", 7, pros::E_TEXT_LARGE_CENTER);
			
		}
	});

	drive.addHorizontalTrackingWheel(HORIZONTAL_TRACKER_PORT, HORIZONTAL_TRACKER_DIAMETER);

	//PID gains are tuned in the CONSTANTS section at the top of this file.
	drive.setDrivePID({DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_TIMEOUT_MS});
	drive.setTurnPID( {TURN_KP,  TURN_KI,  TURN_KD,  TURN_TIMEOUT_MS});
	armPID = {ARM_KP, ARM_KI, ARM_KD, ARM_TIMEOUT_MS};

	drive.setTrackWidth(10.8);
	drive.setOdomConfig(warbots::Drive::odomConfig::IMU_HORIZONTAL);
	drive.initImu();
	drive.resetPose();
	
	arm.tare_position_all();
	register_autons();
    pros::lcd::initialize();
	pros::delay(2000);
    logo_task.remove();
	selector.init();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	selector.selected_auton_call();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	drive.setDriveType(warbots::Drive::SPLIT_ARCADE);

	int armPositionIndex = 0;  // index into ARM_POSITIONS; 0 = home

	while (true) {
		groupControl(setGoal);
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			openclaw();
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			closeclaw();
		}
		// Arm position stepping: R1 = up a setpoint, R2 = down a setpoint, LEFT = home.
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
			if (armPositionIndex < 4) armPositionIndex++;
			setGoal = ARM_POSITIONS[armPositionIndex];
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			if (armPositionIndex > 0) armPositionIndex--;
			setGoal = ARM_POSITIONS[armPositionIndex];
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			armPositionIndex = 0;
			setGoal = ARM_POSITIONS[armPositionIndex];
		}

		// Cap drive speed based on live arm height: full speed at/below
		// ARM_SPEED_LIMIT_LOW_POS, half speed at/above ARM_SPEED_LIMIT_HIGH_POS,
		// linearly interpolated in between.
		double armPos = arm.get_position(0);
		double t = (armPos - ARM_SPEED_LIMIT_LOW_POS) / (ARM_SPEED_LIMIT_HIGH_POS - ARM_SPEED_LIMIT_LOW_POS);
		t = std::max(0.0, std::min(1.0, t));
		double speedScale = DRIVE_SPEED_SCALE_AT_LOW + t * (DRIVE_SPEED_SCALE_AT_HIGH - DRIVE_SPEED_SCALE_AT_LOW);

		// Hold A for precision driving - overrides the arm-height scale above.
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			speedScale = PRECISION_SPEED_SCALE;
		}
		drive.setSpeedScale(speedScale);

		warbots::drawLogo();
		drive.updatePose();
		warbots::screenPrint("ARM" + warbots::doubleToString(arm.get_position(0), 2) , 4);
		warbots::screenPrint("PID" + warbots::doubleToString(outputMain, 2) , 5);
		drive.control(master);
		pros::delay(20);
	}
}
