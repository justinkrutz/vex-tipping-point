#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "main.h"

extern std::shared_ptr<OdomChassisController> chassis;
extern std::shared_ptr<SkidSteerModel> skid_model;

extern pros::Controller master;
extern pros::Controller partner;

extern pros::Motor lift_motor;
extern pros::Rotation lift_sensor;

extern pros::ADIDigitalOut lift_gripper;
extern pros::ADIDigitalOut back_tilter;

extern pros::Motor ring_motor;

extern pros::Gps gps;
extern pros::IMU imu;
extern pros::Distance back_distance;
extern pros::ADIDigitalIn goal_sensor;

void build_chassis();

#endif // ROBOT_CONFIG_H