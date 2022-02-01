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

pros::ADIDigitalOut lift_gripper('a', true);
pros::ADIDigitalOut back_tilter('b', true);

pros::Motor ring_motor(11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Gps gps(12, 0.07, 0.07);
pros::IMU imu(13);
pros::Distance back_distance(14);

// okapi::Motor drive_fl (9, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// okapi::Motor drive_fr (10, true,  AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// okapi::Motor drive_bl (1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// okapi::Motor drive_br (2, true,  AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

// pros::Motor intake_left (3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor intake_right (8, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor top_roller (12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
// pros::Motor bottom_roller (20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

// pros::Imu imu (4);
// pros::Rotation tracker_left (5);
// pros::Rotation tracker_back (6);
// pros::Rotation tracker_right (7);

// pros::Distance distance_sensor_right(15);
// pros::Distance distance_sensor_left(16);
// pros::Optical optical_sensor(17);

// pros::ADILineSensor ball_sensor_intake ('A');
// pros::ADILineSensor ball_sensor_bottom ('B');
// pros::ADILineSensor ball_sensor_middle ('C');
// pros::ADILineSensor ball_sensor_top    ('D');
// pros::ADILineSensor ball_sensor_score  ('E');
// //                                     ('F');
// pros::ADILineSensor goal_sensor_one    ('G');
// pros::ADILineSensor goal_sensor_two    ('H');

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