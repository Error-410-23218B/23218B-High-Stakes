/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */

pros::MotorGroup LeftDrivetrain({-8,-9,-10});
pros::MotorGroup RightDrivetrain({1,2,3});

pros::Optical opticalSensor(20);



pros::adi::Pneumatics intake_piston({17,'B'},false); 
pros::adi::Pneumatics arm_piston({17,'D'},false);

pros::adi::Pneumatics mobo_piston({17,'A'},false);  
pros::adi::Pneumatics mobo_piston2({17,'C'},false);  

pros::adi::DigitalOut led1('E',HIGH);

pros::adi::DigitalOut led2('F',HIGH);

// pros::adi::DigitalOut intake_piston(std::make_pair(17,'B')); 
// pros::adi::DigitalOut  arm_piston(std::make_pair(17,'D'));

// pros::adi::DigitalOut  mobo_piston(std::make_pair(17,'A'));  
// pros::adi::DigitalOut  mobo_piston2(std::make_pair(17,'C'));  

pros::adi::DigitalIn limitSwitch(std::make_pair(17,'E'));

pros::Motor intake_lower(18);
pros::Motor intake_upper(19);
pros::Motor arm_motor(-11);

pros::MotorGroup intakeMotorGroup({18,19});

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
