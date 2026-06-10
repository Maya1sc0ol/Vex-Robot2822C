#pragma once
#include "main.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
//ADD THIS FILE To main.h so you can access it in your .cpp file!!!

//Create and Configure your devices for your subsystem here

//Make sure to put inline at the front so it only creates once!!!!
inline pros::Motor arm(-11);
inline pros::Motor arm2(-14);
inline pros::Motor claw(9);
inline pros::MotorGroup group{-11,-14};
inline warbots::PIDconfigs armPID = {1, 0.01, 0.1, 2000.0};
inline double outputMain;
inline double setGoal;
//Create Functions down here, they will be accessible in your example.cpp file for you to define
void examplePIDFunction(double goal);
void openclaw();
void closeclaw();
void groupControl(double goal);