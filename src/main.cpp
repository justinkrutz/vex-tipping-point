#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-drive.h"
#include "odom-utilities.h"
#include "ball-system.h"
#include "odometry.h"
#include "lift.h"



bool menu_enabled = true;
bool open_claw_on_start = false;
bool auton_has_run = false;

void set_drive_callbacks() {
  menu_enabled = false;
  controllerbuttons::button_handler.clear_group("menu");
  // robotfunctions::set_callbacks();
  lift::set_callbacks();
  // autondrive::set_callbacks();
  autondrive::stick_control_enabled = true;
  autondrive::auton_group.terminate();
  controllermenu::master_print_array[0] = "";
  controllermenu::master_print_array[1] = "";
  controllermenu::master_print_array[2] = "";
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
  autondrive::stick_control_enabled = false;
  // lift_gripper.set_value(1);
  // build_chassis();
  // odom_init();
  // chassis->setState({13.491_in, 34.9911_in, 0_deg});
  // chassis->setState({15.7416_in, 31.4911_in, -90_deg});
  // imu_odom->setState({15.7416_in, 31.4911_in, -90_deg});
  // optical_sensor.set_led_pwm(100);
  pros::Task(autondrive::reset_button_task);
  pros::Task(autondrive::motor_task);
  lift::set_callbacks();
  // robotfunctions::set_callbacks();
  // ballsystem::set_callbacks();
  // ballsystem::init();
  autondrive::set_callbacks();
  // odomutilities::load_autons_from_SD();
  lift::init();
  controllermenu::init();
  // pros::Task roller_task (robotfunctions::rollers::main_task);
  controllerbuttons::button_handler.master.r2.pressed.set(set_drive_callbacks);
  // odomutilities::errorcorrection::start();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  open_claw_on_start = true;
  autondrive::auton_group.terminate();
  // odomutilities::errorcorrection::auto_goal_center = true;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  auton_has_run = true;
  controllermenu::run_auton();

  // while (true) {
  //   pros::delay(150);
  // }
}
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
  controllermenu::abort_auton();
  if (open_claw_on_start && !auton_has_run) {
    // lift::claw.retract();
  }
  if (pros::competition::is_connected()) {
    set_drive_callbacks();
  } else {
    // odomutilities::errorcorrection::auto_goal_center = false;
  }
  controllermenu::partner_print_array[0] = "test";

  while (true) {
    controllerbuttons::run_buttons();
    // ballsystem::debug();
    if (!menu_enabled) {
      // QLength imu_x = imu_odom->getState().x;
      // QLength imu_y = imu_odom->getState().y;
      // QAngle imu_theta = imu_odom->getState().theta;
      // controllermenu::master_print_array[0] = std::to_string(tracker_left.get_position())  + " " + std::to_string(imu_x.convert(inch));
      // controllermenu::master_print_array[1] = std::to_string(tracker_right.get_position()) + " " + std::to_string(imu_y.convert(inch));
      // controllermenu::master_print_array[2] = std::to_string(tracker_back.get_position())  + " " + std::to_string(imu_theta.convert(degree));
      
      // QLength x = chassis->getState().x;
      // QLength y = chassis->getState().y;
      // QAngle theta = chassis->getState().theta;
      // controllermenu::partner_print_array[0] = std::to_string(tracker_left.get_position())  + " " + std::to_string(x.convert(inch));
      // controllermenu::partner_print_array[1] = std::to_string(tracker_right.get_position()) + " " + std::to_string(y.convert(inch));
      // controllermenu::partner_print_array[2] = std::to_string(tracker_back.get_position())  + " " + std::to_string(theta.convert(degree));
      
      // controllermenu::master_print_array[0] = std::to_string(optical_sensor.get_raw().red)   + " " + std::to_string(optical_sensor.get_rgb().red);
      // controllermenu::master_print_array[1] = std::to_string(optical_sensor.get_hue());
      // controllermenu::master_print_array[2] = std::to_string(optical_sensor.get_saturation())  + " " + std::to_string(optical_sensor.get_proximity());
      // controllermenu::master_print_array[1] = std::to_string(distance_sensor_left.get())  + " " + std::to_string(distance_sensor_left.get_confidence());
      // controllermenu::master_print_array[2] = std::to_string(distance_sensor_right.get())  + " " + std::to_string(distance_sensor_right.get_confidence());
      
      // controllermenu::partner_print_array[0] = std::to_string(optical_sensor.get_raw().clear) + " " + std::to_string(optical_sensor.get_proximity());
      // controllermenu::partner_print_array[1] = std::to_string(optical_sensor.get_rgb().brightness);
      // controllermenu::partner_print_array[2] = std::to_string(optical_sensor.get_hue()) + " " + std::to_string(optical_sensor.get_saturation());
      // controllermenu::master_print_array[1] = std::to_string(imu.get_rotation());
    }
    pros::delay(10);
  }
}
