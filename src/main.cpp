#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <iostream>
using namespace std;
using namespace pros;
using namespace lemlib;
// controller
Controller controller(E_CONTROLLER_MASTER);

// motor groups
/*
red = 100 rpm
green = 200 rpm
blue = 600 rpm
*/
MotorGroup leftMotors({1, -3, 5}, MotorGearset::green);   // left motor group
MotorGroup rightMotors({6, -8, 10}, MotorGearset::green); // right motor group

// parameter variables
int inertial_sensor_port(12);

int horizontal_tracking_wheel_port(19);
int vertical_tracking_wheel_port(-11);

float horizontal_tracking_wheel_offset(
    -5.75); // in inches, negative if the wheel is to the back of the tracking
            // center, positive if it's to the front
float vertical_tracking_wheel_offset(-2.5); // in inches, negative if the wheel

int track_width(15); // in inches
int drivetrain_rpm(200);
int horizontal_drift(
    2); // higher values make the robot move faster but causes more overshoot on
        // turns. Recommended value of 2 if not using traction wheels, 8 if
        // using traction wheels

// parameter lists
float linear_PID[3] = {10, 0, 25.5}; // kP, kI, kD for linear motion
float angular_PID[3] = {2, 0, 10.5}; // kP, kI, kD for angular motion

float throttle_curve[3] = {
    3, 10, 1.019}; // joystick deadband out of 127, minimum output where
                   // drivetrain will move out of 127, expo curve gain
float steer_curve[3] = {
    3, 10, 1.019}; // joystick deadband out of 127, minimum output where
                   // drivetrain will move out of 127, expo curve gain

// Inertial Sensor on port
Imu imu(inertial_sensor_port);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
Rotation horizontalEnc(horizontal_tracking_wheel_port);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
Rotation verticalEnc(vertical_tracking_wheel_port);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot
// (negative)
TrackingWheel horizontal(&horizontalEnc, Omniwheel::NEW_275,
                         horizontal_tracking_wheel_offset);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot
// (negative)
TrackingWheel vertical(&verticalEnc, Omniwheel::NEW_275,
                       vertical_tracking_wheel_offset);

// drivetrain settings
Drivetrain drivetrain(&leftMotors,  // left motor group
                      &rightMotors, // right motor group
                      track_width,
                      Omniwheel::NEW_4, // using new 4" omnis
                      drivetrain_rpm, horizontal_drift);

// lateral motion controller
ControllerSettings
    linearController(linear_PID[0], // proportional gain (kP)
                     linear_PID[1], // integral gain (kI)
                     linear_PID[2], // derivative gain (kD)
                     3,             // anti windup
                     1,             // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3,   // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     20   // maximum acceleration (slew)
    );

// angular motion controller
ControllerSettings
    angularController(angular_PID[0], // proportional gain (kP)
                      angular_PID[1], // integral gain (kI)
                      angular_PID[2], // derivative gain (kD)
                      3,              // anti windup
                      1,              // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0    // maximum acceleration (slew)
    );

// sensors for odometry
OdomSensors sensors(&vertical, // vertical tracking wheel
                    nullptr, // vertical tracking wheel 2, set to nullptr as we
                             // don't have a second one
                    &horizontal, // horizontal tracking wheel
                    nullptr, // horizontal tracking wheel 2, set to nullptr as
                             // we don't have a second one
                    &imu     // inertial sensor
);

// input curve for throttle input during driver control
ExpoDriveCurve throttleCurve(
    throttle_curve[0], // joystick deadband out of 127
    throttle_curve[1], // minimum output where drivetrain will move out of 127
    throttle_curve[2]  // expo curve gain
);

// input curve for steer input during driver control
ExpoDriveCurve steerCurve(
    steer_curve[0], // joystick deadband out of 127
    steer_curve[1], // minimum output where drivetrain will move out of 127
    steer_curve[2]  // expo curve gain
);

// create the chassis
Chassis chassis(drivetrain, linearController, angularController, sensors,
                &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  lcd::initialize();   // initialize brain screen
  chassis.calibrate(); // calibrate sensors

  // the default rate is 50. however, if you need to change the rate, you
  // can do the following.
  // bufferedStdout().setRate(...);
  // If you use bluetooth or a wired connection, you will want to have a rate of
  // 10ms

  // for more information on how the formatting for the loggers
  // works, refer to the fmtlib docs

  // thread to for brain screen and position logging
  Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      lcd::print(0, "X: %f", chassis.getPose().x);         // x
      lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // log position telemetry
      telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      delay(50);
    }
  });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(entire_path_1_txt);
ASSET(path_section_1_1_txt);
ASSET(path_section_1_2_txt);
ASSET(path_section_1_3_txt);
ASSET(path_section_1_4_txt);
ASSET(path_section_1_5_txt);

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the
 * features LemLib has to offer
 */
void autonomous() {
  chassis.setPose(-59.871, 2.42, 77.82);
  //chassis.follow(entire_path_1_txt, 15,2000); // follow the path with the robot's heading tangent to the path
  chassis.follow(path_section_1_1_txt, 15, 2000); // goes to red scoring
  //scores preload
  
  chassis.follow(path_section_1_2_txt, 15, 2000); //goes to pick up blue/red cone
  //picks up cone

  chassis.follow(path_section_1_3_txt, 15, 2000); // goes to red scoring
  //scores cone

  chassis.follow(path_section_1_4_txt, 15, 2000); // goes to pick up red/yellow cone
  //picks up cone
  
  chassis.follow(path_section_1_5_txt, 15, 2000); // goes to red scoring
  //scores cone
  
}

/**
 * Runs in driver control
 */
void opcontrol() {
  // controller
  // loop to continuously update motors
  while (true) {
    // get joystick positions
    int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
    // move the chassis with curvature drive
    chassis.arcade(leftY, rightX);
    // delay to save resources
    delay(10);
  }
}