#include "main.h"
#include "SubSystems/Chassis.hpp"
#include "SubSystems/Intake.hpp"
#include "SubSystems/Indexer.hpp"
#include "SubSystems/Flywheel.hpp"
#include "SubSystems/Vision.hpp"
#include "SubSystems/Optical.hpp"

#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include <string>

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

	pros::Motor FRONT_LEFT(9, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor FRONT_RIGHT(10, pros::E_MOTOR_GEARSET_18, true);
	pros::Motor BACK_LEFT(13, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor BACK_RIGHT(2, pros::E_MOTOR_GEARSET_18, true);
	// FRONT_LEFT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// FRONT_RIGHT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// BACK_LEFT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	// BACK_RIGHT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	Chassis chassis(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

	pros::Motor imtr = pros::Motor (20);
	imtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	Intake intake = Intake(imtr);

	pros::Motor inmtr = pros::Motor (6);
	inmtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	Indexer indexer = Indexer(inmtr);
	// pros::Imu Inertial(6);
	pros::Rotation RotationLeft(7, true);
	pros::Rotation RotationRight(12, true);

	// Inertial.reset();
	RotationLeft.reset_position();
	RotationRight.reset_position();

	double rlmult = 0.001989;
	double rrmult = 0.001978;

	//Hello

	// pros::Motor FLY_LEFT(4, pros::E_MOTOR_GEARSET_06, true);
	// pros::Motor FLY_RIGHT(5, pros::E_MOTOR_GEARSET_06, false);
	okapi::MotorGroup fmtrs = okapi::MotorGroup({-4, 5});
	Flywheel flywheel = Flywheel(fmtrs);

	pros::Vision visionSensor (8, pros::E_VISION_ZERO_CENTER);
	Vision vision (visionSensor);

	//pros::vision_signature_s_t BLUE_GOAL = {1, {1, 0, 0}, 1.600, -3197, -1033, -2116, 3137, 9595, 6366, 0, 0};
	pros::vision_signature_s_t RED_GOAL = {1, {1, 0, 0}, 1.000, 2029, 8495, 5262, -203, 769, 284, 0, 0};
	
	pros::Optical opticalSensor(19);
	Optical optical(opticalSensor);
	
	bool togggle = false;

	chassis.brake();

	bool bluerepeat = false;
	bool redrepeat = false;

	while(true) {
		chassis.XDriveTelOp(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		intake.moveIntake(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
		indexer.moveIndexer(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1), master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
		flywheel.flywheelTelOp(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP), master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT), master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT), master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN), master.get_digital(pros::E_CONTROLLER_DIGITAL_X));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) == true || togggle == true){
			chassis.aimbotMovement(vision.aimbot(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y), togggle));
		}

		intake.roller(optical.roller(master.get_digital(pros::E_CONTROLLER_DIGITAL_A), master.get_digital(pros::E_CONTROLLER_DIGITAL_Y), bluerepeat, redrepeat));

		// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == true){
		// 	while(opticalSensor.get_hue() > 140){
		// 		imtr.move_velocity(-100);
		// 	}
		// 	std::string temp = std::to_string(opticalSensor.get_hue());
		// 	pros::lcd::set_text(0, "Front Left Motor: " + temp);
		// }
		// else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) == true){
		// 	while(opticalSensor.get_hue() < 60){
		// 		imtr.move_velocity(-100);
		// 	}
		// }

		std::string temp = std::to_string(FRONT_LEFT.get_temperature());
		pros::lcd::set_text(0, "Front Left Motor: " + temp);
		temp = std::to_string(FRONT_RIGHT.get_temperature());
		pros::lcd::set_text(1, "Front Right Motor: " + temp);
		temp = std::to_string(BACK_LEFT.get_temperature());
		pros::lcd::set_text(2, "Back Left Motor: " + temp);
		temp = std::to_string(BACK_RIGHT.get_temperature());
		pros::lcd::set_text(3, "Back Right Motor: " + temp);
		temp = std::to_string(fmtrs.getTemperature());
		pros::lcd::set_text(4, "Flywheel Motors: " + temp);
		temp = std::to_string(imtr.get_temperature());
		pros::lcd::set_text(5, "Intake Motor: " + temp);
		temp = std::to_string(inmtr.get_temperature());
		pros::lcd::set_text(6, "Indexer Motor: " + temp);
		pros::delay(20);
	}
}