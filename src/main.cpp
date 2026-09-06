#include "main.h"
#include "autonConfig.hpp"
#include <algorithm>

const double ARM_KP = 3.5, ARM_KI = 0.0, ARM_KD = 0.35, ARM_TIMEOUT_MS = 2000.0;

const double ARM_RAMP_DEG_PER_TICK = 10.0;
const double ARM_GRAVITY_FF_MAX = 25.0;
const double ARM_POSITIONS[5] = {0.00, 25.00, 50.00, 75.00, 100.00};

const double ARM_SPEED_FLAT_ZONE_POS   = 50.0;
const double ARM_SPEED_LIMIT_HIGH_POS  = 100.0;
const double DRIVE_SPEED_SCALE_AT_LOW  = 1.0;
const double DRIVE_SPEED_SCALE_AT_MID  = 0.5;
const double DRIVE_SPEED_SCALE_AT_HIGH = 0.3;

const double PRECISION_SPEED_SCALE = 0.3;

const int    HORIZONTAL_TRACKER_PORT     = 6;
const double HORIZONTAL_TRACKER_DIAMETER = 2.75;

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

warbots::Drive drive(
	{-8, -3},
	{4, 10},
	3.25,
	0.267,
	true,
	1
);

void initialize() {
	group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	pros::Task logo_task([]() {
		while (true) {
			drawLogo();
			pros::delay(20);
			warbots::screenPrint("Warbot-Template", 7, pros::E_TEXT_LARGE_CENTER);
			warbots::screenPrint("ARM" + warbots::doubleToString(getArmAngle(), 2), 6);
		}
	});

	drive.addHorizontalTrackingWheel(HORIZONTAL_TRACKER_PORT, HORIZONTAL_TRACKER_DIAMETER);

	armPID = {ARM_KP, ARM_KI, ARM_KD, ARM_TIMEOUT_MS};
	armGravityFF = ARM_GRAVITY_FF_MAX;
	armRampDegPerTick = ARM_RAMP_DEG_PER_TICK;

	drive.setTrackWidth(10.8);
	drive.setOdomConfig(warbots::Drive::odomConfig::IMU_HORIZONTAL);
	drive.initImu();
	drive.resetPose();

	armRotation.set_reversed(false);
	register_autons();
    pros::lcd::initialize();
	pros::delay(2000);
    logo_task.remove();
	selector.init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// Applied here, once, so every match auto (redAuto/blueAuto/skills/redLeft) gets the same
	// tuned drive/turn PID without each having to call setDrivePID()/setTurnPID() itself - see
	// autonConfig.hpp for where to retune. Tuning/diagnostic autons (pidTuneTest,
	// pidTurnTuneTest, odomSquareTest) still override these with their own local constants
	// immediately after this runs, so they're unaffected.
	drive.setDrivePID(autonConfig::DRIVE_PID);
	drive.setTurnPID(autonConfig::TURN_PID);
	drive.setDriveExit(autonConfig::DRIVE_EXIT);
	drive.setTurnExit(autonConfig::TURN_EXIT);

	selector.selected_auton_call();
}

void opcontrol() {
	drive.setDriveType(warbots::Drive::SPLIT_ARCADE);

	int armPositionIndex = 0;
	double armTargetGoal = ARM_POSITIONS[armPositionIndex];
	setGoal = getArmAngle();
	armGravityFFScale = 0.0;

	while (true) {
		// Disabled for now - bench-test auton trigger removed from teleop.
		// Re-enable by uncommenting when we want to run autons from teleop again.
		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		// 	warbots::autonAbortRequested = false;
		// 	autonomous();
		//
		// 	setGoal = getArmAngle();
		// 	armGravityFFScale = 0.0;
		// 	int nearest = 0;
		// 	double bestDist = std::fabs(ARM_POSITIONS[0] - setGoal);
		// 	for (int i = 1; i < 5; i++) {
		// 		double dist = std::fabs(ARM_POSITIONS[i] - setGoal);
		// 		if (dist < bestDist) { bestDist = dist; nearest = i; }
		// 	}
		// 	armPositionIndex = nearest;
		// 	armTargetGoal = ARM_POSITIONS[armPositionIndex];
		// }

		double rampStep = warbots::wrapAngleDeg(armTargetGoal - setGoal);
		if (rampStep > armRampDegPerTick) rampStep = armRampDegPerTick;
		else if (rampStep < -armRampDegPerTick) rampStep = -armRampDegPerTick;
		setGoal += rampStep;

		groupControl(setGoal);
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			openclaw();
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			closeclaw();
		}
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

		double armPos = getArmAngle();
		double speedScale;
		if (armPos <= ARM_SPEED_FLAT_ZONE_POS) {
			speedScale = DRIVE_SPEED_SCALE_AT_LOW;
		} else {
			double t = (armPos - ARM_SPEED_FLAT_ZONE_POS) / (ARM_SPEED_LIMIT_HIGH_POS - ARM_SPEED_FLAT_ZONE_POS);
			t = std::max(0.0, std::min(1.0, t));
			speedScale = DRIVE_SPEED_SCALE_AT_MID + t * (DRIVE_SPEED_SCALE_AT_HIGH - DRIVE_SPEED_SCALE_AT_MID);
		}

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
