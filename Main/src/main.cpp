#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Ring Lift and Mogo Mech

pros::Motor liftMotor(15);
pros::Motor intakeMotor(7);
pros::ADIDigitalOut mogoMechDown(6);
pros::ADIDigitalOut mogoMechUp(2);
pros::Motor wallstake1 (2);
pros::Motor wallstake2 (3);
int liftVoltage;
int intakeVoltage;
bool mogoDown = false;
bool mogoUp = false;
int wallstakeDegrees = 0;
int wallstakeCurrentPos = 0;

// Drivetrain

pros::MotorGroup leftDrive({-12, 13, -14});
pros::MotorGroup rightDrive({8, -9, 10});
pros::MotorGroup wallstake({-2, 3});

float leftDriveSpeed; // a float for quadratic input scaling (currently not in use, but doesn't hurt)
float rightDriveSpeed;

int chaosVariable;

bool safety = false; // disables driving other than the slow driving with arrows if true

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
	pros::lcd::set_text(1, "Behold Botteneimer!");

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

	while (true) {

        // Movement Logic

		liftVoltage = 0;  // reset voltage to stop motor if no button is pressed to override this value
		intakeVoltage = 0;
		wallstakeCurrentPos = wallstake1.get_position();
		

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

		// Lift arm logic

		if (master.get_digital(DIGITAL_UP)) {
			wallstakeDegrees = -600;
		}
		if (master.get_digital(DIGITAL_DOWN)) {
			wallstakeDegrees = 0;
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
		
		if (not safety) {
			leftDriveSpeed += master.get_analog(ANALOG_LEFT_Y);
			rightDriveSpeed += master.get_analog(ANALOG_LEFT_Y);

			leftDriveSpeed +=  master.get_analog(ANALOG_RIGHT_X);
			rightDriveSpeed -=  master.get_analog(ANALOG_RIGHT_X);
		}

			// slow movement to allow testing and avoid catastrophe

		/**if (master.get_digital(DIGITAL_UP)) {
			rightDriveSpeed = 20;
			leftDriveSpeed = 20;
		}
		if (master.get_digital(DIGITAL_DOWN)) {
			rightDriveSpeed = -20;
			leftDriveSpeed = -20;
		}
		*/
		if (master.get_digital(DIGITAL_LEFT)) {
			rightDriveSpeed = 20;
			leftDriveSpeed = -20;
		}
		if (master.get_digital(DIGITAL_RIGHT)) {
			rightDriveSpeed = -20;
			leftDriveSpeed = 20;
		}

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

		// Chaos Button
		if (not safety) {
			chaosVariable += 1;
			if (master.get_digital(DIGITAL_A) and chaosVariable % 10 == 0) {
				rightDriveSpeed = ((rand() % 254)-127);
				leftDriveSpeed = ((rand() % 254)-127);
			}
		}


        // Movement Exec

		liftMotor.move(liftVoltage);
		intakeMotor.move(intakeVoltage);

		if (abs(wallstakeDegrees - wallstakeCurrentPos) > 400) {
			wallstake.move_absolute(wallstakeDegrees, 80);
		} else {
			wallstake.move_absolute(wallstakeDegrees, 80);
		}

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

		rightDrive.move(rightDriveSpeed);
		leftDrive.move(leftDriveSpeed);

		pros::delay(20); // frametime: 20ms = 50hz
	}
}