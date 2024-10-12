#include "main.h"

rd::Console console;


void autonred(){
	console.println("Red Auton!");
}

void autonblue(){
		console.println("Blue Auton!");

}

void skillsauton(){
		console.println("Skills Auton!");
}


rd::Selector selector({
	{"Auton Red", &autonred},
	{"Auton Blue", &autonblue},
	{"Skills Run", &skillsauton}
});


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */





void initialize() {



	intake_lower.set_voltage_limit(6);
	intake_upper.set_voltage_limit(6);

	// Odometry initialise
chassis =	
okapi::ChassisControllerBuilder()
	.withLogger(
		std::make_shared<okapi::Logger>(
			okapi::TimeUtilFactory::createDefault().getTimer(),
			"/ser/sout",
			okapi::Logger::LogLevel::error
		)
	)
	.withMotors(
		{1,2,3},
		{-4,-5,-6}
	)
	.withDimensions(okapi::AbstractMotor::gearset::blue, {{3.25_in,11.5_in},okapi::imev5GreenTPR})
	.withSensors(
		okapi::ADIEncoder{'A','B'},
		okapi::ADIEncoder{'C','D',true},
		okapi::ADIEncoder{'E','F'}

	)
	.withOdometry({{2.75_in, 7_in, 1_in, 2.75_in},okapi::quadEncoderTPR})
	.buildOdometry();
	

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
void autonomous() {
		chassis->setState({0_in,0_in,0_deg});
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
void opcontrol() {


	bool moboPistonBool = false;
	bool intakeLowerBool = false;
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	okapi::ControllerButton btnUp(okapi::ControllerDigital::R1);
	okapi::ControllerButton btnDown(okapi::ControllerDigital::R2);
  	int goalHeight = 0;



	while (true) {
		
		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		int right = power + turn;
		int left = power - turn;
		LeftDrivetrain.move(left);
		RightDrivetrain.move(right);		

	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		goalHeight++;
		armControl->setTarget(goalHeight);
	}
	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		goalHeight--;
		armControl->setTarget(goalHeight);

	}
	else{
		armControl->setTarget(goalHeight);
	}
	

	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
		moboPistonBool = !moboPistonBool;
		mobo_piston.set_value(moboPistonBool?HIGH:LOW);
	}

	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
	{
		intakeLowerBool = !intakeLowerBool;
		intake_lower.move(intakeLowerBool?127:0);
	}

	disk_reject_piston.set_value(opticalSensor.get_rgb().red > opticalSensor.get_rgb().blue && opticalSensor.get_rgb().red > opticalSensor.get_rgb().green ?HIGH:LOW);
	
		

	mobo_piston.set_value(mobo_limit_switch.get_value() == HIGH ? HIGH : LOW);
	
	



	pros::delay(20);
	}	
}
	