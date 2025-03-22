#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor intake_lower(-18);
inline pros::Motor intake_upper(-16);
inline pros::Optical opticalSensor(1);

inline pros::MotorGroup intakeMotorGroup({18,16});

inline pros::adi::Pneumatics mobo_piston('B',false); 

inline pros::adi::Pneumatics some_piston('F',false);  
inline pros::adi::Pneumatics some_piston2('E',false); 

inline pros::adi::Pneumatics doinker_piston('A',false); 
