#include "main.h"


std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<SkidSteerModel> skid_model;

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);

pros::Motor lift_motor(4, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Rotation lift_sensor(3);

pros::ADIDigitalOut lift_gripper('a', true);
pros::ADIDigitalOut back_tilter('b', true);

pros::Motor ring_motor(11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Gps gps(12, 0.07, 0.07);
pros::IMU imu(13);
pros::Distance back_distance(14);
pros::ADIDigitalIn goal_sensor('g');

void build_chassis() {
  chassis = ChassisControllerBuilder()
    .withMotors(
      {5, -6, 7},
      {-8, 9, -10}
    )
    // wheel size is 4.1797_in times 3/7 gear ratio
    .withDimensions({AbstractMotor::gearset::blue}, {{0.142939386978, 14.2739608613}, imev5BlueTPR})
    .withOdometry()
    .buildOdometry();

  // chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  skid_model = std::dynamic_pointer_cast<SkidSteerModel>(chassis->getModel()); // From last season
}