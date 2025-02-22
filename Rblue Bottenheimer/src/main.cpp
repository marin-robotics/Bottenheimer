#include "main.h"
#include <math.h>
#include "pros/vision.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Ring Lift and Mogo Mech

pros::Motor liftMotor(15);
pros::Motor intakeMotor(2);
pros::ADIDigitalOut mogoMechDown(3); // ignore its suggestion, this works
pros::ADIDigitalOut mogoMechUp(2);
pros::ADIDigitalOut rightDoinkDown(1);
pros::ADIDigitalOut leftDoinkDown(8);
pros::ADIDigitalIn liftButton(4);
pros::Motor brownMech (4);
pros::Vision visionSensor (8);
pros::Gps gps1(20, .223, -.223);
pros::gps_status_s_t gpsData;
int liftVoltage;
int intakeVoltage;

float xpos;
float xvel;
float ypos;
float yvel;
float heading;

bool mogoDown = false;
bool mogoUp = false;

bool leftDoink = false;
bool rightDoink = false;
bool aPressed = false;
bool leftPressed = false;

// Color Sensor

int ejectTimer = 10;
int shakeLength = 25;
int shakeVoltage = 0;
bool buttonPressed = false;
bool wrongRing = false;

// Drivetrain

pros::MotorGroup leftDrive({-12, 13, -14});
pros::MotorGroup rightDrive({5, -9, 10});

float leftDriveSpeed; // a float for quadratic input scaling (currently not in use, but doesn't hurt)
float rightDriveSpeed;
float forwardSpeed;

// Lady Brown Mech

int brownMechDegrees = 0;
int brownMechState = 0;
bool upPressed = false;
bool rightPressed = false;

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

void updategps() {
	xpos = gps1.get_position_x();
	xvel = gps1.get_gyro_rate_x();
	ypos = gps1.get_position_y();
	yvel = gps1.get_gyro_rate_y();
	heading = gps1.get_heading();
}


void autonomous() {
	mogoMechUp.set_value(LOW);
	mogoMechDown.set_value(HIGH);
	brownMech.move_absolute(0, 25);
	leftDrive.move(43);
	rightDrive.move(56);
	pros::delay(260);
	leftDrive.move(0);
	rightDrive.move(0);
	brownMech.move_absolute(-1500, -127);
	pros::delay(1250);
	leftDrive.move(-70);
	rightDrive.move(-20);
	pros::delay(450);
	brownMech.move_velocity(127);
	leftDrive.move(-60);
	rightDrive.move(-60);
	pros::delay(150);
	leftDrive.move(-20);
	rightDrive.move(-70);
	pros::delay(400);
	brownMech.move_velocity(0);
	brownMech.set_zero_position(0);
	leftDrive.move(-90);
	rightDrive.move(-90);
	pros::delay(700);
	mogoMechUp.set_value(HIGH);
	mogoMechDown.set_value(LOW);
	pros::delay(150);
	leftDrive.move_relative(0, 25);
	rightDrive.move_relative(0, 25);
	pros::delay(200);
	leftDrive.move(60);
	rightDrive.move(60);
	pros::delay(350);
	leftDrive.move(-80);
	rightDrive.move(80);
	pros::delay(450);
	leftDrive.move(60);
	rightDrive.move(60);
	intakeMotor.move(127);
	pros::delay(850);
	leftDrive.move(0);
	rightDrive.move(0);
	liftMotor.move(-127);
	intakeMotor.move(0);
	pros::delay(1250);
	leftDrive.move(-50);
	rightDrive.move(-50);
	pros::delay(300);
	liftMotor.move(0);
	leftDrive.move(-80);
	rightDrive.move(80);
	pros::delay(450);
	intakeMotor.move(127);
	leftDrive.move(40);
	rightDrive.move(40);
	pros::delay(400);
	leftDrive.move(0);
	rightDrive.move(0);
	pros::delay(400);
	intakeMotor.move(0);
	liftMotor.move(-127);
	pros::delay(1500);
	liftMotor.move(0);
	

	
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
	brownMech.set_zero_position(0);
	updategps();


	pros::vision_signature_s_t blueSig = pros::Vision::signature_from_utility (1, -3347, -1023, -2185, 4129, 11257, 7693, 1.000, 0);
	pros::vision_signature_s_t redSig = pros::Vision::signature_from_utility (2, 3213, 10923, 7068, -753, 355, -199, 1.100, 0);
	visionSensor.set_signature(1, &blueSig);
	visionSensor.set_signature(2, &redSig);

	while (true) {

		// Movement Logic

		liftVoltage = 0;  // reset voltage to stop motor if no button is pressed to override this value
		intakeVoltage = 0;

		// Intake logic

		if (master.get_digital(DIGITAL_L1)) {
			intakeVoltage = 127;
		}
		if (master.get_digital(DIGITAL_L2)) {
			intakeVoltage = -127;
			liftVoltage = 60;
		}

		// Lady Brown logic

		if (master.get_digital(DIGITAL_UP) and (not upPressed)) {
			if (brownMechState < 4) {
				brownMechState += 1;
			}
			upPressed = true;
		}
		if (not master.get_digital(DIGITAL_UP) and (upPressed)) {
			upPressed = false;
		}

		if (master.get_digital(DIGITAL_RIGHT) and (not rightPressed)) {
			if (brownMechState > 0) {
				brownMechState -= 1;
			}
			rightPressed = true;
		}
		if (not master.get_digital(DIGITAL_RIGHT) and (rightPressed)) {
			rightPressed = false;
		}

		if (master.get_digital(DIGITAL_DOWN)) {
			brownMechState = 0;
		}

		if (brownMechState == 0) {
			brownMechDegrees = 0;
		} else if (brownMechState == 1) {
			brownMechDegrees = -375;
		} else if (brownMechState == 2) {
			brownMechDegrees = -1035;
		} else if (brownMechState == 3) {
			brownMechDegrees = -1365;
		} else if (brownMechState == 4) {
			brownMechDegrees = -1865;
		}

		// Hotkeys for lift and brown

		if (master.get_digital(DIGITAL_R2)) {
			liftVoltage = -80;
			brownMechState = 1;
		}

		if (master.get_digital(DIGITAL_R1)) {
			liftVoltage = -127;
			if (brownMechState == 1) {
				brownMechState = 2;
			}
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

		// Vision Sensor Logic
		

		pros::vision_object_s_t redRing = visionSensor.get_by_sig(0, 2);
		pros::vision_object_s_t blueRing = visionSensor.get_by_sig(0, 1);

		if (redRing.width > 100) {
			wrongRing = true;
		} else if (blueRing.width > 100) {
			wrongRing = false;
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
		
		leftDriveSpeed += master.get_analog(ANALOG_LEFT_Y);
		rightDriveSpeed += master.get_analog(ANALOG_LEFT_Y);

		leftDriveSpeed +=  master.get_analog(ANALOG_RIGHT_X);
		rightDriveSpeed -=  master.get_analog(ANALOG_RIGHT_X);

/**
		if (forwardSpeed > 0) {
			leftDriveSpeed += pow((forwardSpeed/127), 2) * 127;
			rightDriveSpeed += pow((forwardSpeed/127), 2) * 127;
			// leftDriveSpeed += ((forwardSpeed / 127) * (forwardSpeed / 127)) * 127;
			// rightDriveSpeed += ((forwardSpeed / 127) * (forwardSpeed / 127)) * 127;
		} else if (forwardSpeed < 0) {
			leftDriveSpeed -= pow((forwardSpeed/127), 2) * 127;
			rightDriveSpeed -= pow((forwardSpeed/127), 2) * 127;
			// leftDriveSpeed += ((forwardSpeed / 127) * (forwardSpeed / 127)) * -127;
			// rightDriveSpeed += ((forwardSpeed / 127) * (forwardSpeed / 127)) * -127;
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
		if (wrongRing && (ejectTimer > 25) && liftButton.get_value()) {
			ejectTimer = 0;

		}
		ejectTimer += 1;

		
		if ((ejectTimer < 10) && (ejectTimer > 1)) {
			liftVoltage = shakeVoltage;
		}

		liftMotor.move(liftVoltage);
		intakeMotor.move(intakeVoltage);

		brownMech.move_absolute(brownMechDegrees, 127);

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

