#pragma once
#include "main.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
//ADD THIS FILE To main.h so you can access it in your .cpp file!!!

//Create and Configure your devices for your subsystem here

//Make sure to put inline at the front so it only creates once!!!!
inline pros::Motor arm(-11);
inline pros::Motor arm2(-14);
inline pros::Motor claw(9);
inline pros::MotorGroup group{-11,-14};
inline pros::Rotation armRotation(18);
inline warbots::PIDconfigs armPID = {1, 0.01, 0.1, 2000.0};
inline double outputMain;
inline double setGoal;
// Once within this many degrees of the goal, stop actively correcting rather than
// chasing encoder backlash/jump the loop can't actually resolve. Paired with the
// arm's E_MOTOR_BRAKE_HOLD brake mode (set in initialize()), 0 output here means
// the motor firmware locks the shaft in place instead of coasting/sagging.
inline double armDeadbandDeg = 5.0;

// Extra output added on top of the PID, shaped to peak when the arm is roughly
// horizontal (midway through ARM_POSITIONS' 0-100 range, where gravity torque on
// a pivoting arm is worst) and taper to ~0 at both ends (near-vertical, where
// gravity fights the arm least). Set from ARM_GRAVITY_FF_MAX in initialize().
inline double armGravityFF = 0.0;

// Max degrees the arm's commanded goal is allowed to move per control tick,
// shared by opcontrol()'s ramp and armGoTo() so a big target change never hands
// the PID a big instantaneous error. Set from ARM_RAMP_DEG_PER_TICK in initialize().
inline double armRampDegPerTick = 10.0;

//Create Functions down here, they will be accessible in your example.cpp file for you to define
void openclaw();
void closeclaw();
void groupControl(double goal);
double getArmAngle();
void armGoTo(double targetDeg, double toleranceDeg = 3.0);