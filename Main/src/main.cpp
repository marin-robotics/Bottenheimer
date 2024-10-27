#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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

	// Ring Lift and Mogo Mech

	pros::Motor liftMotor(15);
	pros::Motor intakeMotor(7);
	pros::ADIDigitalOut mogoMechDown(6);
	pros::ADIDigitalOut mogoMechUp(2);
	int liftVoltage;
	int intakeVoltage;
	bool mogoDown = false;
	bool mogoUp = false;

	// Drivetrain

	pros::Motor leftDrive1(12);
	pros::Motor leftDrive2(13);
	pros::Motor leftDrive3(14);
	pros::Motor rightDrive1(8);
	pros::Motor rightDrive2(9);
	pros::Motor rightDrive3(10);

	leftDrive1.set_reversed(true); // Pros wouldn't accept the bool arg in motor definition
	leftDrive3.set_reversed(true);
	rightDrive2.set_reversed(true);

	float leftDriveSpeed; // a float for quadratic input scaling (currently not in use, but doesn't hurt)
	float rightDriveSpeed;

	while (true) {

        // Movement Logic

		liftVoltage = 0;  // reset voltage to stop motor if no button is pressed to override this value
		intakeVoltage = 0;

        // Lift logic

		if (master.get_digital(DIGITAL_R2)) {
			liftVoltage = -127;
		}
		if (master.get_digital(DIGITAL_R1)) {
			liftVoltage = 127;
		}

        // Intake logic

		if (master.get_digital(DIGITAL_L1)) {
			intakeVoltage = 127;
		}
		if (master.get_digital(DIGITAL_L2)) {
			intakeVoltage = -127;
		}

        // Mogo mech logic

		if (master.get_digital(DIGITAL_X)) {
			mogoDown = true;
			mogoUp = false;
		}
		if (master.get_digital(DIGITAL_Y)) {
			mogoDown = false;
			mogoUp = true;
		}
		if (master.get_digital(DIGITAL_B)) {
			mogoDown = false;
			mogoUp = false;
		}

		// Drive Logic

		leftDriveSpeed = 0;
		rightDriveSpeed = 0;
		
		leftDriveSpeed += master.get_analog(ANALOG_LEFT_Y);
		rightDriveSpeed += master.get_analog(ANALOG_LEFT_Y);

		leftDriveSpeed +=  master.get_analog(ANALOG_RIGHT_X);
		rightDriveSpeed -=  master.get_analog(ANALOG_RIGHT_X);

/**		

			// Quadratic Input Scaling

		if (leftDriveSpeed > 0) {
			leftDriveSpeed /= 127;
			leftDriveSpeed *= leftDriveSpeed;
			leftDriveSpeed *= 127;
		} else if (leftDriveSpeed < 0) {
			leftDriveSpeed /= 127;
			leftDriveSpeed *= leftDriveSpeed;
			leftDriveSpeed *= -127;
		}
		if (rightDriveSpeed > 0) {
			rightDriveSpeed /= 127;
			rightDriveSpeed *= rightDriveSpeed;
			rightDriveSpeed *= 127;
		} else if (rightDriveSpeed < 0) {
			rightDriveSpeed /= 127;
			rightDriveSpeed *= rightDriveSpeed;
			rightDriveSpeed *= -127;
		}

*/

        // Movement Exec

		liftMotor.move(liftVoltage);
		intakeMotor.move(intakeVoltage);

		if (mogoDown) {
			mogoMechDown.set_value(HIGH);
		} else {
			mogoMechDown.set_value(LOW);
		}
		if (mogoUp) {
			mogoMechUp.set_value(HIGH);
		} else {
			mogoMechUp.set_value(LOW);
		}

		rightDrive1.move(rightDriveSpeed);
		rightDrive2.move(rightDriveSpeed);
		rightDrive3.move(rightDriveSpeed);
		leftDrive1.move(leftDriveSpeed);
		leftDrive2.move(leftDriveSpeed);
		leftDrive3.move(leftDriveSpeed);

		pros::delay(20); // frametime: 20ms = 50hz
	}
}