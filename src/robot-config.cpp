#include "main.h"


std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<AsyncMotionProfileController> profileController;

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);

pros::Motor lift_motor(15, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Rotation lift_sensor(3);

pros::ADIDigitalOut shifter     ('a', true);
pros::ADIDigitalOut back_claw   ('b');
pros::ADIDigitalOut back_tilter ('c');
pros::ADIDigitalOut left_claw   ('d', true);
pros::ADIDigitalOut right_claw  ('e', true);

pros::Motor ring_motor(16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Gps gps(12, 0.12, 0.05);
pros::IMU imu(13);
pros::Distance back_distance(14);
pros::ADIDigitalIn goal_sensor('g');

void build_chassis() {
  chassis = ChassisControllerBuilder()
    .withMotors(
      {7, 9, -19},
      {8, -18, -20})
    .withSensors(
      RotationSensor(11),
      RotationSensor(12))
    // wheel size is 4.1797_in times 3/7 gear ratio
    .withDimensions({AbstractMotor::gearset::blue}, {{1662, 6.19911541924}, imev5BlueTPR})
    .withOdometry()
    // .withGains(
    //   {0.0005, 0, 0.0}, 
    //   {0.0005, 0, 0.0},  
    //   {0.001, 0, 0.0})
    //   // {0.0005, 0, 0.00001}, 
    //   // {0.0005, 0, 0.00001},  
    //   // {0.001, 0, 0.000001})
    .buildOdometry();

  profileController = 
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.4, // Maximum linear velocity of the Chassis in m/s
      3.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(chassis)
    .buildMotionProfileController();

  // chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}