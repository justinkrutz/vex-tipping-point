#include "main.h"


std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<SkidSteerModel> skid_model;

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);

pros::Motor left_front(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_middle(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_back(7, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_front(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_middle(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_back(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor lift_motor(4, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Rotation lift_sensor(3);

pros::ADIDigitalOut lift_gripper('a', false);
pros::ADIDigitalOut back_tilter('b', false);
pros::ADIDigitalOut tilter_drop('d', true);
pros::ADIDigitalOut ring_shooter('h', false);

pros::Motor ring_motor(11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Gps gps(12, 0.07, 0.07);
pros::IMU imu(13);
pros::Distance back_distance(14);

pros::ADIDigitalIn goal_sensor_left('e');
pros::ADIDigitalIn goal_sensor_right('f');
pros::ADIDigitalIn goal_sensor_center('g');

pros::ADIDigitalIn reset_button('c');

void build_chassis() {
  // tracker_left.reset_position();
  // tracker_right.reset_position();
  // tracker_back.reset_position();

  chassis = ChassisControllerBuilder()
    .withMotors(
      {5, -6, 7},
      {-8, 9, -10}
    )
    // wheel size is 4.1797_in times 3/7 gear ratio
    .withDimensions({AbstractMotor::gearset::blue, (3.0 / 7.0)}, {{4.1797_in, 10.6016_in}, imev5BlueTPR})
    // .withDimensions(AbstractMotor::gearset::blue, {{1.7913_in, 10.6016_in}, imev5BlueTPR})
    .withOdometry()
    .buildOdometry();

  // chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  skid_model = std::dynamic_pointer_cast<SkidSteerModel>(chassis->getModel()); // From last season
}