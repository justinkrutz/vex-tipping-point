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
extern pros::ADIDigitalOut tilter_drop;
extern pros::ADIDigitalOut ring_shooter;

extern pros::Motor ring_motor;

extern pros::Gps gps;
extern pros::IMU imu;
extern pros::Distance back_distance;
extern pros::ADIDigitalIn goal_sensor_left;
extern pros::ADIDigitalIn goal_sensor_right;
extern pros::ADIDigitalIn goal_sensor_center;

extern pros::ADIDigitalIn reset_button;

void build_chassis();

#endif // ROBOT_CONFIG_H