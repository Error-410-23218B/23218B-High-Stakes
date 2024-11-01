#include "main.h"
#include "lemlib/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */



lemlib::Drivetrain drivetrain(&LeftDrivetrain, // left motor group
                              &RightDrivetrain, // right motor group
                              13.61,	 // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);


pros::adi::Encoder left_encoder('G', 'H');	
pros::adi::Encoder right_encoder('A', 'B');
pros::adi::Encoder back_encoder('C', 'D',true);

lemlib::TrackingWheel left_tracking_wheel(&left_encoder, lemlib::Omniwheel::NEW_275, 3.24);

lemlib::TrackingWheel right_tracking_wheel(&right_encoder, lemlib::Omniwheel::NEW_275, -2.75);


lemlib::TrackingWheel back_tracking_wheel(&back_encoder, lemlib::Omniwheel::NEW_275, -2.8);

pros::Imu imu(7);


lemlib::OdomSensors sensors(&left_tracking_wheel, 
							nullptr,
                            &back_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu// inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              1, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.55, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

	pros::Controller master(pros::E_CONTROLLER_MASTER);


ASSET(matchauton_txt);

void initialize() {

pros::c::ext_adi_port_set_config(17,'A',pros::E_ADI_DIGITAL_OUT);
pros::c::ext_adi_port_set_config(17,'C',pros::E_ADI_DIGITAL_OUT);
pros::c::ext_adi_port_set_config(17,'B',pros::E_ADI_DIGITAL_OUT);
pros::c::ext_adi_port_set_config(17,'D',pros::E_ADI_DIGITAL_OUT);

chassis.calibrate();
chassis.setPose(0, 0, 0);


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

void opticalSensor(color){
	while(true)
{
if(blue)	
{
	if (opticalSensor.get_rgb().blue > opticalSensor.get_rgb().red && opticalSensor.get_rgb().blue > opticalSensor.get_rgb().green && opticalSensor.get_rgb().blue > 180 && pros::competition::is_autonomous() )
	{
		chassis.cancelAllMotions();
	}

}

else (opticalSensor.get_rgb().red > opticalSensor.get_rgb().blue && opticalSensor.get_rgb().red > opticalSensor.get_rgb().green && opticalSensor.get_rgb().red > 180 && pros::competition::is_autonomous()){

}
	}
}

void competition_initialize() {
		
}

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

void angularTest(){
	
	
	chassis.turnToHeading(90,4000);
	 master.set_text(2,0,std::to_string(chassis.getPose().theta));

}

void linearTest(){
	chassis.moveToPose(24,48,0,5000);
	
}


void redAutonomous(){
	pros::Task opticalTask(opticalSensor,false);
	chassis.setPose(72,0,180);
	
	chassis.moveToPose(72,10,180,3000,{.forwards = false});
	intake_upper.move(-127);
	pros::delay(500);
	intake_upper.move(0);
	pros::delay(650);
	arm_motor.move_velocity(100);
	pros::delay(1750);
	arm_motor.move_velocity(0);

	
	chassis.moveToPose(66.5,35.5,155,4000,{.forwards = false});
	
	pros::delay(3000);
	mobo_piston.extend();
	mobo_piston2.extend();
	chassis.moveToPose(12,36,270,3000);
	
	
	// mobo_piston.extend();
	// mobo_piston2.extend();
	// chassis.moveToPose(72,16,180,3000);

	// arm_motor.move_relative(-20,100);
	// pros::delay(2000);
	// chassis.moveToPose(48,48,180,3000,{.forwards = false});
	// mobo_piston.extend();
	// mobo_piston2.extend();
	// intakeMotorGroup.move(127);
	// pros::delay(2000);
	// chassis.turnToHeading(270,2000);
	// chassis.moveToPoint(24,48,3000);
	// chassis.moveToPose(72,24,0,3000);
	// chassis.moveToPose(72,48,0,3000);
}

void blueAutonomous(){
	pros::Task opticalTask(opticalSensor,true);
	chassis.setPose(72,0,180);
	
	chassis.moveToPose(72,10,180,3000,{.forwards = false});
	intake_upper.move(-127);
	pros::delay(500);
	intake_upper.move(0);
	pros::delay(650);
	arm_motor.move_velocity(100);
	pros::delay(1750);
	arm_motor.move_velocity(0);

	
	chassis.moveToPose(66.5,35.5,155,4000,{.forwards = false});
	
	pros::delay(3000);
	mobo_piston.extend();
	mobo_piston2.extend();
	chassis.moveToPose(12,36,270,3000);
	
	
	// mobo_piston.extend();
	// mobo_piston2.extend();
	// chassis.moveToPose(72,16,180,3000);

	// arm_motor.move_relative(-20,100);
	// pros::delay(2000);
	// chassis.moveToPose(48,48,180,3000,{.forwards = false});
	// mobo_piston.extend();
	// mobo_piston2.extend();
	// intakeMotorGroup.move(127);
	// pros::delay(2000);
	// chassis.turnToHeading(270,2000);
	// chassis.moveToPoint(24,48,3000);
	// chassis.moveToPose(72,24,0,3000);
	// chassis.moveToPose(72,48,0,3000);
}

ASSET(skills1_txt);
ASSET(skills2_txt);
ASSET(match1_txt);
ASSET(match2_txt);
void skillsAutonomous(){
	intakeGroup.move(127);
	chassis.follow(skills1_txt,10,10000);
	mobo_piston.extend();
	mobo_piston2.extend();
	chassis.follow(skills2_txt,10,20000);
	mobo_piston.retract();
	mobo_piston2.retract();


}

void matchAutonomousPursuit(){
	chassis.follow(match1_txt,10,3000);
	arm_motor.move(-127);
	pros::delay(1000);
	arm_motor.move(127);
	pros::delay(500)
	intakeMotorGroup.move(127);
	chassis.follow(match2_txt,10,5000);
	mobo_piston.extend();
	mobo_piston2.extend();
	
}

void returnTest(){
	chassis.moveToPose(0,0,0,5000);

}

rd::Selector selector({
    {"Red Auton", redAutonomous},
    {"Blue auton", blueAutonomous},
    {"Skills auton", skillsAutonomous},
});


void autonomous() {
	selector.run_auton();
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

void opcontrol()
{

	bool moboPistonBool = false;
	bool intakeBool = false;
	bool intakeLowerBool = false;
	bool intakeUpperBool = false;

	bool pneumaticsBool = false;

	int goalHeight = 0;
	bool armBool = false;
	bool armPBool = false;
	

	while (true)
	{
			 master.set_text(2,0,std::to_string(chassis.getPose().x));
	std::cout << std::to_string(chassis.getPose().y);
		// Read the controller buttons
		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		int left = power + turn;
		int right = power - turn;
		LeftDrivetrain.move(left);
		RightDrivetrain.move(right);


	std::cout << arm_motor.get_position();
		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		// {

		// 	goalHeight += 128;
		// 	armControl->setTarget(goalHeight);
		// 	pros::delay(5);
		// }

		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		// {
		// 	armControl->setMaxVelocity(100);
		// 	goalHeight -= 132;
		// 	armControl->setTarget(goalHeight);
		// 	pros::delay(5);
		// }



		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		// {
		// 	goalHeight += 1;

		// 	armControl->setTarget(goalHeight);
		// }



		// if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		// {
		// 	goalHeight -= 1;
		// 	armControl->setTarget(goalHeight);
		// }

	if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){

		arm_motor.move_velocity(100);
	}

	else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){

		arm_motor.move_velocity(-100);

	}

	else{

		arm_motor.move_voltage(0);
	}









		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			intakeBool = !intakeBool;
			intakeMotorGroup.move(intakeBool ? 127 : 0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			intakeLowerBool = !intakeLowerBool;
			intake_lower.move(intakeLowerBool ? -127 : 0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			intakeUpperBool = !intakeUpperBool;
			intake_upper.move(intakeUpperBool ? -127 : 0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
		{
			mobo_piston.toggle();
			mobo_piston2.toggle();

			
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
			arm_piston.toggle();
	
		}


if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
{
	angularTest();
}

if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
{
	linearTest();
}
	
if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
{
	returnTest();
}

if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
{
	arm_motor.move_velocity(100);
	pros::delay(1750);
	arm_motor.move_velocity(0);


}

		// intake_piston.set_value(opticalSensor.get_rgb().red > opticalSensor.get_rgb().blue && opticalSensor.get_rgb().red > opticalSensor.get_rgb().green ?HIGH:LOW);

		// mobo_piston.set_value(mobo_limit_switch.get_value() == HIGH ? HIGH : LOW);
	}
}
