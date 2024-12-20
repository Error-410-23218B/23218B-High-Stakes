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

lemlib::Drivetrain drivetrain(&LeftDrivetrain,			  // left motor group
							  &RightDrivetrain,			  // right motor group
							  13.61,					  // 10 inch track width
							  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
							  450,						  // drivetrain rpm is 450
							  2							  // horizontal drift is 2 (for now)
);

pros::adi::Encoder left_encoder('G', 'H');
pros::adi::Encoder right_encoder('A', 'B');
pros::adi::Encoder back_encoder('C', 'D', true);

lemlib::TrackingWheel left_tracking_wheel(&left_encoder, lemlib::Omniwheel::NEW_275, 3.24);

lemlib::TrackingWheel right_tracking_wheel(&right_encoder, lemlib::Omniwheel::NEW_275, -2.75);

lemlib::TrackingWheel back_tracking_wheel(&back_encoder, lemlib::Omniwheel::NEW_275, -2.8);

pros::Imu imu(7);

lemlib::OdomSensors sensors(&left_tracking_wheel,
							nullptr,
							&back_tracking_wheel, // horizontal tracking wheel 1
							nullptr,			  // horizontal tracking wheel 2, set to nullptr as we don't have a second one
							&imu				  // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
											  0,   // integral gain (kI)
											  1,   // derivative gain (kD)
											  3,   // anti windup
											  1,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.55, // proportional gain (kP)
											  0,	// integral gain (kI)
											  10,	// derivative gain (kD)
											  0,	// anti windup
											  0,	// small error range, in inches
											  0,	// small error range timeout, in milliseconds
											  0,	// large error range, in inches
											  0,	// large error range timeout, in milliseconds
											  0		// maximum acceleration (slew)
);
lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
);

void initialize()
{
	pros::c::ext_adi_port_set_config(17, 'A', pros::E_ADI_DIGITAL_OUT);
	pros::c::ext_adi_port_set_config(17, 'C', pros::E_ADI_DIGITAL_OUT);
	pros::c::ext_adi_port_set_config(17, 'B', pros::E_ADI_DIGITAL_OUT);
	pros::c::ext_adi_port_set_config(17, 'D', pros::E_ADI_DIGITAL_OUT);
	pros::c::ext_adi_port_set_config(17, 'E', pros::E_ADI_DIGITAL_IN);
	led1.set_value(HIGH);
	led2.set_value(HIGH);
	opticalSensor.set_led_pwm(100);
	chassis.calibrate();
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

void matchAutonomous()
{

	chassis.setPose(72, 0, 180);
	intake_upper.move_voltage(-12000);
	pros::delay(500);
	intake_upper.move_voltage(0);
	chassis.moveToPose(50, 50, 130, 3000, {.forwards = false}, false);
	mobo_piston.extend();
	mobo_piston2.extend();
	arm_motor.move_voltage(12000);
	pros::delay(1000);
	arm_motor.move_voltage(0);
	intakeMotorGroup.move_velocity(12000);
	pros::delay(1000);
	chassis.moveToPose(31, 38, 270, 3000,{},false);
	// chassis.moveToPoint(36, 32, 3000, {.forwards = false,.maxSpeed = 75   });
	pros::delay(1500);
	intakeMotorGroup.move(0);
	intake_piston.extend();
	chassis.moveToPose(82,-14,180,4000,{},false);
	arm_motor.move_voltage(-12000);
	pros::delay(600);
	arm_motor.move_voltage(0);
	chassis.moveToPose(85,24,45,3000,{.forwards = false});

}

















void matchAutonomous2()
{
	
	chassis.setPose(72, 0, 180);
	intake_upper.move_voltage(-12000);
	pros::delay(500);
	intake_upper.move_voltage(0);
	pros::delay(500);
	chassis.moveToPose(50, 50, 130, 3000, {.forwards = false}, false);
	mobo_piston.extend();
	mobo_piston2.extend();
	arm_motor.move_voltage(12000);
	pros::delay(1000);
	arm_motor.move_voltage(0);
	intakeMotorGroup.move_velocity(12000);
	pros::delay(1000);
	chassis.moveToPose(30, 38, 270, 3000);
	// pros::delay(1500);
	// chassis.moveToPoint(36, 32, 3000, {.forwards = false,.maxSpeed = 75   });
	intakeMotorGroup.move_velocity(12000);
	intake_piston.extend();

	chassis.moveToPose(85,-15,180,4000); 
	

}


void huhAuton(){
	 chassis.setPose(72, 0, 0);
	intake_upper.move_voltage(-12000);
	pros::delay(500);
	intake_upper.move_voltage(0);
	chassis.moveToPose(72,32,0,3000);
}

ASSET(skills1_txt);
ASSET(skills2_txt);

void skillsAutonomous()
{
	intakeMotorGroup.move(127);
	chassis.follow(skills1_txt, 10, 10000);
	mobo_piston.extend();
	mobo_piston2.extend();
	chassis.follow(skills2_txt, 10, 20000);
	mobo_piston.retract();
	mobo_piston2.retract();
}

void skillAutonEtc()
{

	chassis.setPose(99,0,145);
	intake_upper.move_voltage(-12000);
	pros::delay(500);
	intake_upper.move_voltage(0);
	intakeMotorGroup.move_velocity(200);
	mobo_piston.extend();
	mobo_piston2.extend();
	chassis.turnToHeading(90,2000);
	chassis.moveToPoint(108,0,3000);
	chassis.turnToHeading(0,2000);
	chassis.moveToPoint(108,10,3000);
	chassis.turnToHeading(90,2000);
	chassis.moveToPoint(130,10,3000);
	chassis.moveToPose(130,-3,0,2000,{.forwards = false});
	chassis.turnToHeading(325,2000);
	chassis.moveToPoint(133,-3,2000,{.forwards = false});
	pros::delay(750);
	mobo_piston.retract();
	mobo_piston2.retract();
	chassis.moveToPoint(80,5,4000);
	// chassis.moveToPose(61,12,90,3000,{.forwards = false});
	// pros::delay(750);
	// mobo_piston.extend();
	// mobo_piston2.extend();
	
 




}

void autonomous()
{
	skillAutonEtc();
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
bool opticalBool = false;

void opticalLed()
{
	while (opticalBool)
	{
		led1.set_value(HIGH);
		pros::delay(1000);
		led1.set_value(LOW);
	}
}

void optical()
{

	while (true)
	{
		if (opticalBool)
		{
			intakeMotorGroup.move_voltage(6000);

			if (((opticalSensor.get_hue() > 200) || (opticalSensor.get_hue() < 20)))
			{

				intakeMotorGroup.move_voltage(0);
				opticalSensor.set_led_pwm(0);
				// pros::delay(500);
				intake_upper.move_relative(-2500, 200);
				pros::delay(600);
				arm_piston.extend();
				arm_motor.move_relative(-40, 100);
				opticalSensor.set_led_pwm(100);
				pros::delay(1000);
			}
		}
	}
}

void opcontrol()
{

	bool moboPistonBool = false;
	bool intakeBool = false;
	bool intakeLowerBool = false;
	bool intakeUpperBool = false;

	bool pneumaticsBool = false;
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	int goalHeight = 0;
	bool armBool = false;
	bool armPBool = false;
	bool ledBool = true;

	while (true)
	{

		// Read the controller buttons
		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		int left = power + turn;
		int right = power - turn;
		LeftDrivetrain.move(left);
		RightDrivetrain.move(right);

		std::cout << arm_motor.get_position();

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{

			arm_motor.move_velocity(100);
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{

			arm_motor.move_velocity(-100);
		}

		else
		{

			arm_motor.move_voltage(0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			intakeBool = !intakeBool;
			intake_lower.move(intakeBool ? 127 : 0);
			intake_upper.move(intakeBool ? 127 : 0);
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
			ledBool = !ledBool;
			led1.set_value(ledBool);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
		{
			opticalBool = !opticalBool;
			led2.set_value(!opticalBool);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			intake_piston.toggle();
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			arm_piston.toggle();
		}

		if (opticalBool)
		{
					master.set_text(2, 0, "Optical On");

  
			if (((opticalSensor.get_hue() > 200) || (opticalSensor.get_hue() < 20)))
			{
				arm_piston.extend();
				pros::delay(350);
				opticalSensor.set_led_pwm(0);
				intakeMotorGroup.move_voltage(0);
				pros::delay(500);
				intake_upper.move_relative(-2500, 200);
				pros::delay(600);
				arm_motor.move_relative(-40, 100);
				opticalSensor.set_led_pwm(100);
				pros::delay(1000);

			}
		}
		else{
								master.set_text(2, 0, "Optical Off");

		}

		if (limitSwitch.get_new_press())
		{
			pros::delay(500);
			mobo_piston.extend();
			mobo_piston2.extend();
			ledBool = false;
			led1.set_value(ledBool);
		}

		// intake_piston.set_value(opticalSensor.get_rgb().red > opticalSensor.get_rgb().blue && opticalSensor.get_rgb().red > opticalSensor.get_rgb().green ?HIGH:LOW);

		// mobo_piston.set_value(mobo_limit_switch.get_value() == HIGH ? HIGH : LOW);
	}
}
