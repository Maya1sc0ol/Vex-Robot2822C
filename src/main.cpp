#include "main.h"

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

	//Have Rotation Sensors/Odom Pods on your drivetrain?
	//Add them Here!!

	//Edit These Values here to configure and tune PID!!!
	//                 kP,  kI,  kD,   timeout
	drive.setDrivePID({0.6, 0.0, 0.2, 3000.0});
	drive.setTurnPID( {5.0, 0.0, 0.0, 2000.0});

	drive.setTrackWidth(10.8);
	drive.setOdomConfig(warbots::Drive::odomConfig::IMU_ONLY);
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

	while (true) {
		groupControl(setGoal);
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			openclaw();
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
			closeclaw();
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			setGoal = 0;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			setGoal = 800;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			setGoal = 1400;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			setGoal = 1900;
		}
		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			setGoal = 2400;
		}
		warbots::drawLogo();
		drive.updatePose();
		warbots::screenPrint("ARM" + warbots::doubleToString(arm.get_position(0), 2) , 4);
		warbots::screenPrint("PID" + warbots::doubleToString(outputMain, 2) , 5);
		drive.control(master);
		pros::delay(20);
	}
}
