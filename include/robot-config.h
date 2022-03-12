#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"

extern std::shared_ptr<OdomChassisController> chassis;
extern std::shared_ptr<SkidSteerModel> skid_model;

extern pros::Controller master;
extern pros::Controller partner;

extern pros::Motor left_front;
extern pros::Motor left_middle;
extern pros::Motor left_back;
extern pros::Motor right_front;
extern pros::Motor right_middle;
extern pros::Motor right_back;

extern pros::Motor lift_motor;
extern pros::Rotation lift_sensor;

extern pros::ADIDigitalOut lift_gripper;
extern pros::ADIDigitalOut back_tilter;

extern pros::Motor ring_motor;

extern pros::Gps gps;
extern pros::IMU imu;
extern pros::Distance back_distance;
extern pros::ADIDigitalIn goal_sensor;

extern pros::ADIDigitalIn reset_button;

// extern okapi::Motor drive_fl (9, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// extern okapi::Motor drive_fr (10, true,  AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// extern okapi::Motor drive_bl (1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// extern okapi::Motor drive_br (2, true,  AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// extern pros::Motor intake_left;
// extern pros::Motor intake_right;
// extern pros::Motor top_roller;
// extern pros::Motor bottom_roller;

// extern pros::Imu imu;
// extern pros::Rotation tracker_left;
// extern pros::Rotation tracker_back;
// extern pros::Rotation tracker_right;

// extern pros::Distance distance_sensor_right;
// extern pros::Distance distance_sensor_left;
// extern pros::Optical optical_sensor;

// extern pros::ADILineSensor ball_sensor_intake;
// extern pros::ADILineSensor ball_sensor_bottom;
// extern pros::ADILineSensor ball_sensor_middle;
// extern pros::ADILineSensor ball_sensor_top;
// extern pros::ADILineSensor ball_sensor_score;

// extern pros::ADILineSensor goal_sensor_one;
// extern pros::ADILineSensor goal_sensor_two;

void build_chassis();

#endif // ROBOT_CONFIG_H