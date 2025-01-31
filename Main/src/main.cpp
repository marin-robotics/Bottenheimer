#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Ring Lift and Mogo Mech

pros::Motor liftMotor(15);
pros::Motor intakeMotor(7);
pros::ADIDigitalOut mogoMechDown(6); // ignore its suggestion, this works
pros::ADIDigitalOut mogoMechUp(2);
pros::ADIDigitalOut rightDoinkDown();
pros::ADIDigitalOut leftDoinkDown();
pros::Motor wallStake1 (2);
pros::Motor wallStake2 (3);
int liftVoltage;
int intakeVoltage;
bool mogoDown = false;
bool mogoUp = false;
bool leftDoink = false;
bool rightDoink = false;
bool aPressed = false;
bool leftPressed = false;


// Drivetrain

pros::MotorGroup leftDrive({-12, 13, -14});
pros::MotorGroup rightDrive({8, -9, 10});

float leftDriveSpeed; // a float for quadratic input scaling (currently not in use, but doesn't hurt)
float rightDriveSpeed;

// Redirect

pros::MotorGroup wallStake({-2, 3});

int wallStakeDegrees = 0;
int wallStakeCurrentPos = 0;

// Extras

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
 * from where it left off..2
 */
void autonomous() {
	mogoMechUp.set_value(HIGH);
	mogoMechDown.set_value(LOW);
	wallStake.move_absolute(-500, 80);
	pros::delay(500);
	leftDrive.move(25);
	rightDrive.move(25);
	pros::delay(1075);
	leftDrive.move(0);
	rightDrive.move(0);
	pros::delay(300);
	wallStake.move_absolute(0, 127);
	pros::delay(1500);
	leftDrive.move(-68);
	rightDrive.move(-80);
	wallStake.move_absolute(-350, 80);
	pros::delay(300);
	leftDrive.move(-80);
	rightDrive.move(-68);
	pros::delay(300);
	leftDrive.move(-80);
	rightDrive.move(-80);
	pros::delay(250);
	leftDrive.move(0);
	rightDrive.move(0);
	mogoMechUp.set_value(LOW);
	mogoMechDown.set_value(HIGH);
	pros::delay(750);
	leftDrive.move(60);
	rightDrive.move(-60);
	pros::delay(540);
	leftDrive.move(0);
	rightDrive.move(0);
	intakeMotor.move(127);
	pros::delay(250);
	leftDrive.move(50);
	rightDrive.move(50);
	pros::delay(700);
	leftDrive.move(0);
	rightDrive.move(0);
	liftMotor.move(127);
	pros::delay(2000);
	leftDrive.move(60);
	rightDrive.move(-60);
	pros::delay(450);
	leftDrive.move(50);
	rightDrive.move(50);
	pros::delay(250);
	liftMotor.move(0);
	pros::delay(300);
	leftDrive.move(0);
	rightDrive.move(0);
	liftMotor.move(127);
	pros::delay(1500);
	intakeMotor.move(0);
	pros::delay(1500);
	liftMotor.move(0);
	leftDrive.move(-30);
	rightDrive.move(-30);
	wallStake.move_absolute(-450, 80);
	pros::delay(250);
	leftDrive.move(60);
	rightDrive.move(-60);
	pros::delay(525);
	leftDrive.move(50);
	rightDrive.move(50);
	pros::delay(1800);
	leftDrive.move(0);
	rightDrive.move(0);
	wallStake.move(0);
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

	while (true) {

        // Movement Logic

		liftVoltage = 0;  // reset voltage to stop motor if no button is pressed to override this value
		intakeVoltage = 0;
		wallStakeCurrentPos = wallStake1.get_position();

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
			wallStakeDegrees = -600;
		}
		if (master.get_digital(DIGITAL_RIGHT)) {
			wallStakeDegrees = -350;
		}
		if (master.get_digital(DIGITAL_DOWN)) {
			wallStakeDegrees = 0;
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

		// Doinker Logic

		if (master.get_digital(DIGITAL_A) and (not aPressed) and (not rightDoink)) {
			rightDoink = true;
			aPressed = true;
		}
		if (master.get_digital(DIGITAL_A) and (not aPressed) and rightDoink) {
			rightDoink = false;
			aPressed = true;
		}
		if (not master.get_digital(DIGITAL_A) and (aPressed)) {
			aPressed = false;
		}


		if (master.get_digital(DIGITAL_LEFT) and (not leftPressed) and (not leftDoink)) {
			leftDoink = true;
			leftPressed = true;
		}
		if (master.get_digital(DIGITAL_LEFT) and (not leftPressed) and leftDoink) {
			leftDoink = false;
			leftPressed = true;
		}
		if (not master.get_digital(DIGITAL_LEFT) and (leftPressed)) {
			leftPressed = false;
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

/**
		if (master.get_digital(DIGITAL_LEFT)) {
			rightDriveSpeed = 20;
			leftDriveSpeed = -20;
		}
		if (master.get_digital(DIGITAL_RIGHT)) {
			rightDriveSpeed = -20;
			leftDriveSpeed = 20;
		}	
*/
			// Quadratic Input Scaling
/**
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

		wallStake.move_absolute(wallStakeDegrees, 80);

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

		if (rightDoink) {
			rightDoinkDown.set_value(HIGH);
		} else {
			rightDoinkDown.set_value(LOW);
		}
		if (leftDoink) {
			leftDoinkDown.set_value(HIGH);
		} else {
			leftDoinkDown.set_value(LOW);
		}


		rightDrive.move(rightDriveSpeed);
		leftDrive.move(leftDriveSpeed);

		pros::delay(20); // frametime: 20ms = 50hz
	}
}