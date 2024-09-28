#include "lemlib/api.hpp"
#include "main.h"

pros::Motor front_left_motor(-13, pros::E_MOTOR_GEAR_BLUE); 
pros::Motor middle_left_motor(-14, pros::E_MOTOR_GEAR_BLUE); 
pros::Motor back_left_motor(15, pros::E_MOTOR_GEAR_BLUE); 
pros::Motor front_right_motor(16, pros::E_MOTOR_GEAR_BLUE); 
pros::Motor middle_right_motor(17, pros::E_MOTOR_GEAR_BLUE); 
pros::Motor back_right_motor(-18, pros::E_MOTOR_GEAR_BLUE); 
pros::Imu imu(20);

pros::MotorGroup left_motor_group({ front_left_motor, middle_left_motor, back_left_motor });

pros::MotorGroup right_motor_group({ front_right_motor, middle_right_motor, back_right_motor });

lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 11.5, lemlib::Omniwheel::OLD_325, 360, 2);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

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
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if (abs(leftY) < 5) {
			leftY = 0;
		}

		if (abs(rightX) < 5) {
			rightX = 0;
		}

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}
