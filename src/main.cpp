#include "main.h"
#include <algorithm>

const double DRIVE_KP = 0.6, DRIVE_KI = 0.0, DRIVE_KD = 0.2, DRIVE_TIMEOUT_MS = 3000.0;
const double TURN_KP = 5.0, TURN_KI = 0.0, TURN_KD = 0.0, TURN_TIMEOUT_MS = 2000.0;
const double ARM_KP = 3.5, ARM_KI = 0.0, ARM_KD = 0.35, ARM_TIMEOUT_MS = 2000.0;

// Max degrees setGoal is allowed to move per opcontrol tick (20ms) toward the
// target setpoint, instead of jumping straight to it. Bounds how big a step the
// PID ever has to react to at once, so a button press can't hand it a big
// instantaneous error - the direct cause of the slam/overshoot "jerk" seen when
// setGoal jumped instantly across the full range.
const double ARM_RAMP_DEG_PER_TICK = 10.0;

// Extra output added on top of the arm PID to fight gravity torque, which peaks
// when the arm is roughly horizontal (around ARM_POSITIONS' midpoint, ~50) and
// is near 0 at the folded/raised extremes (~0 and ~100). This is why movement
// "through perpendicular" was weaker than movement near home.
const double ARM_GRAVITY_FF_MAX = 25.0;

// Recalibrated after the rotation sensor's absolute zero was reset with the arm
// at true home - was {236.00, 261.75, 287.50, 313.25, 339.00} against the old
// (now stale) zero reference. Evenly spaced across the same ~100-degree span.
const double ARM_POSITIONS[5] = {0.00, 25.00, 50.00, 75.00, 100.00};

const double ARM_SPEED_LIMIT_LOW_POS   = 0.0;
const double ARM_SPEED_LIMIT_HIGH_POS  = 100.0;
const double DRIVE_SPEED_SCALE_AT_LOW  = 1.0;
const double DRIVE_SPEED_SCALE_AT_HIGH = 0.5;

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
	{-8, -3},  // left motors
	{5, 10},   // right motors
	3.25,      // wheel diameter (in)
	450,       // gear ratio (motor:wheel)
	true,      // use IMU
	1          // IMU port
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

	drive.setDrivePID({DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_TIMEOUT_MS});
	drive.setTurnPID( {TURN_KP,  TURN_KI,  TURN_KD,  TURN_TIMEOUT_MS});
	armPID = {ARM_KP, ARM_KI, ARM_KD, ARM_TIMEOUT_MS};
	armGravityFF = ARM_GRAVITY_FF_MAX;
	group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	drive.setTrackWidth(10.8);
	drive.setOdomConfig(warbots::Drive::odomConfig::IMU_HORIZONTAL);
	drive.initImu();
	drive.resetPose();
	
	// Raw sensor data already increases as the arm physically rises (confirmed after
	// the fresh zero-reset at home), so no software mirroring is needed - reversing
	// here would make get_angle() report the opposite of ARM_POSITIONS' 0->100 scale.
	armRotation.set_reversed(false);
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

	int armPositionIndex = 0;  // 0 = home
	double armTargetGoal = ARM_POSITIONS[armPositionIndex];
	setGoal = getArmAngle();  // ramp from wherever the arm actually is, not a snap to home

	while (true) {
		// Ease setGoal toward armTargetGoal by at most ARM_RAMP_DEG_PER_TICK this
		// tick, so groupControl() never sees a big instantaneous step.
		double rampStep = armTargetGoal - setGoal;
		if (rampStep > ARM_RAMP_DEG_PER_TICK) rampStep = ARM_RAMP_DEG_PER_TICK;
		else if (rampStep < -ARM_RAMP_DEG_PER_TICK) rampStep = -ARM_RAMP_DEG_PER_TICK;
		setGoal += rampStep;

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
			armTargetGoal = ARM_POSITIONS[armPositionIndex];
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
			if (armPositionIndex > 0) armPositionIndex--;
			armTargetGoal = ARM_POSITIONS[armPositionIndex];
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			armPositionIndex = 0;
			armTargetGoal = ARM_POSITIONS[armPositionIndex];
		}

		// Cap drive speed based on live arm height: full speed at/below
		// ARM_SPEED_LIMIT_LOW_POS, half speed at/above ARM_SPEED_LIMIT_HIGH_POS,
		// linearly interpolated in between.
		double armPos = getArmAngle();
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
		auto& pose = drive.getPose();
		warbots::screenPrint("X" + warbots::doubleToString(pose.x, 2), 1);
		warbots::screenPrint("Y" + warbots::doubleToString(pose.y, 2), 2);
		warbots::screenPrint("A" + warbots::doubleToString(pose.angle, 2), 3);
		warbots::screenPrint("ARM" + warbots::doubleToString(armPos, 2) , 4);
		warbots::screenPrint("PID" + warbots::doubleToString(outputMain, 2) , 5);
		drive.control(master);
		pros::delay(20);
	}
}
