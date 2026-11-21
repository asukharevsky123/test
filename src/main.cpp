#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
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
Motor rightFront(-11, MotorGearset::green); // right front motor
Motor rightBack(-1, MotorGearset::green); // right back motor
Motor leftFront(20, MotorGearset::green); // left front motor
Motor leftBack(2, MotorGearset::green); // left back motor
MotorGroup rightMotors({-1, -11}, MotorGearset::green);  // right motor group
MotorGroup leftMotors({2,20}, MotorGearset::green); // left motor group
Motor fourBar_right(-3, MotorGearset::green); // four bar lift right motor
Motor fourBar_left(9, MotorGearset::green); // four bar lift left motor
MotorGroup fourBar({-3,9}, MotorGearset::green); // four bar lift
Motor claw(5, MotorGearset::green); // claw motor

// parameter variables
int inertial_sensor_port(12);

int horizontal_tracking_wheel_port(19);
int vertical_tracking_wheel_port(-11);

float horizontal_tracking_wheel_offset(-5.75); // in inches, negative if the wheel is to the back of the tracking
            // center, positive if it's to the front
float vertical_tracking_wheel_offset(-2.5); // in inches, negative if the wheel

float track_width(12.5); // in inches
int drivetrain_rpm(200);
int horizontal_drift(2); // higher values make the robot move faster but causes more overshoot on
        // turns. Recommended value of 2 if not using traction wheels, 8 if
        // using traction wheels

// parameter lists
float linear_PID[3] = {10, 2, 11}; // kP, kI, kD for linear motion
float angular_PID[3] = {5, -2, 15}; // kP, kI, kD for angular motion

float throttle_curve[3] = {3, 10, 1.019}; // joystick deadband out of 127, minimum output where
                   // drivetrain will move out of 127, expo curve gain
float steer_curve[3] = {3, 10, 1.019}; // joystick deadband out of 127, minimum output where
                   // drivetrain will move out of 127, expo curve gain

bool highSens(true);

// Inertial Sensor on port
Imu imu(inertial_sensor_port);
/*
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
*/
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
                     3, // anti windup
                     1, // small error range, in inches
                     100, // small error range timeout, in milliseconds
                     3, // large error range, in inches
                     500, // large error range timeout, in milliseconds
                     20 // maximum acceleration (slew)
    );

// angular motion controller
ControllerSettings
    angularController(angular_PID[0], // proportional gain (kP)
                      angular_PID[1], // integral gain (kI)
                      angular_PID[2], // derivative gain (kD)
                      3,              // anti windup
                      1,              // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3, // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0  // slew rate — limits how fast motor power can change
    );

// sensors for odometry
OdomSensors sensors(nullptr, //&vertical, // vertical tracking wheel
                    nullptr, // vertical tracking wheel 2, set to nullptr as we
                             // don't have a second one
                    nullptr, //&horizontal, // horizontal tracking wheel
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
    chassis.calibrate(); // calibrate sensors (~3 seconds, robot must be still)
    fourBar.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);
    claw.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    
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
            double motor_temps[7] = {rightFront.get_temperature(), rightBack.get_temperature(), leftFront.get_temperature(), leftBack.get_temperature(), fourBar_right.get_temperature(), fourBar_left.get_temperature(), claw.get_temperature()};
            for (int temp(0); temp<7; temp++){
                motor_temps[temp] = motor_temps[temp]*9/5 + 32; // convert to fahrenheit
            }
            lcd::print(0, "X: %f", chassis.getPose().x);         // x
            lcd::print(1, "Y: %f", chassis.getPose().y);         // y
            lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            lcd::print(3, "RF: %.0f, RB: %.0f, LF: %.0f, LB: %.0f", motor_temps[0], motor_temps[1], motor_temps[2], motor_temps[3]);
            lcd::print(4, "4BR: %.0f, 4BL: %.0f, Claw: %.0f",motor_temps[4], motor_temps[5], motor_temps[6]);
            lcd::print(5, "FourBar: %d", controller.get_digital(E_CONTROLLER_DIGITAL_L1) - controller.get_digital(E_CONTROLLER_DIGITAL_L2));
            lcd::print(6, "Claw: %d", controller.get_digital(E_CONTROLLER_DIGITAL_R1) - controller.get_digital(E_CONTROLLER_DIGITAL_R2));
            lcd::print(7, "HighSens: %s", highSens ? "true" : "false");
            //lcd::print(6")
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

ASSET(left_auton_path_1_txt);


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the
 * features LemLib has to offer
 */
void autonomous() {
    fourBar.move(100);
    delay(200);
    fourBar.brake();

    delay(500);

    claw.move(100);
    delay(500);
    claw.brake();

    /*
    fourBar.move(100);
    delay(750);
    fourBar.brake();
    delay(250);
    
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, -10, 1200, {.minSpeed = 80});
    delay(250);
    chassis.moveToPoint(0, 0, 1000, {.forwards = false});
    delay(250);

    //chassis.setPose(0,0,0);
    chassis.moveToPoint(0, -10, 1200, {.minSpeed = 80});
    delay(250);
    chassis.moveToPoint(0, 0, 1000, {.forwards = false});
    delay(250);
    
    //chassis.setPose(0,0,0);
    chassis.moveToPoint(0, -10, 1200, {.minSpeed = 80});
    delay(250);
    chassis.moveToPoint(0, 0, 1000, {.forwards = false});
    delay(250);
    */
    /*
    fourBar.move(100);
    delay(250);
    fourBar.brake();
    
    chassis.setPose(-60.479, 3.94, 66.835);
    chassis.follow(left_auton_path_1_txt, 15, 3000); // goes to red scoring

    fourBar.move(-20);
    delay(250);
    fourBar.brake();
    
    claw.move(-100);
    delay(500);
    claw.brake();
    */

    //chassis.setPose(0, 0, 0);
    //chassis.moveToPoint(0, 24, 10000);
    //chassis.turnToHeading(90, 10000);
}

/*
 * Runs in driver control
 */
void opcontrol() {
  // controller
  // loop to continuously update motors
  while (true) {
    // get joystick positions
    int turn = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
    int throttle = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

    int fourBar_up = controller.get_digital(E_CONTROLLER_DIGITAL_L1);
    int fourBar_down = controller.get_digital(E_CONTROLLER_DIGITAL_L2);

    int claw_open = controller.get_digital(E_CONTROLLER_DIGITAL_R1);
    int claw_close = controller.get_digital(E_CONTROLLER_DIGITAL_R2);
    int button = controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X);
    if (button==1){
        if (highSens==true) highSens=false;
        else highSens = true;
    }

    // move the chassis with curvature drive

    bool fourBar_moving(false);
    if (highSens) chassis.arcade(throttle, turn);
    else chassis.arcade(throttle/2, turn/2);

    if (controller.get_digital(E_CONTROLLER_DIGITAL_L1)== 0 and controller.get_digital(E_CONTROLLER_DIGITAL_L2)==0) fourBar_moving = false;
    else fourBar_moving = true;

    if (fourBar_moving) {
        if (highSens){
            if (fourBar_up==1) fourBar.move(100);
            else if (fourBar_down==1) fourBar.move(-20);
        }
        else {
            if (fourBar_up==1) fourBar.move(75);
            else if (fourBar_down==1) fourBar.move(-10);
        }
    }
    else fourBar.brake();
    if (claw_open == 1 or claw_close == 1) {
        claw.move((claw_open - claw_close) * 100);
    }
    else claw.brake();
    // delay to save resources
    delay(10);
  }
}