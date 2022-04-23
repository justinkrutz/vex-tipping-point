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


bool debug = true;
bool menu_enabled = true;
bool open_claw_on_start = false;
bool auton_has_run = false;

void set_drive_callbacks() {
  menu_enabled = false;
  controllerbuttons::button_handler.clear_group("menu");
  // robotfunctions::set_callbacks();
  lift::set_callbacks();
  // autondrive::set_callbacks();
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
  Logger::setDefaultLogger(
    std::make_shared<Logger>(
        TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
        "/ser/sout", // Output to the PROS terminal
        Logger::LogLevel::warn // Show errors and warnings
    )
  );
  // left_claw.set_value(1);
  build_chassis();
  odom_init();
  // chassis->setState({13.491_in, 34.9911_in, 0_deg});
  // chassis->setState({15.7416_in, 31.4911_in, -90_deg});
  // imu_odom->setState({15.7416_in, 31.4911_in, -90_deg});
  // optical_sensor.set_led_pwm(100);
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
  // odomutilities::errorcorrection::gps_allign = true;
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
    // lift::claw_l.retract();
  }
  if (pros::competition::is_connected()) {
    set_drive_callbacks();
  } else {
    // odomutilities::errorcorrection::gps_allign = false;
  }

  int i = 0;
  while (true) {
    controllerbuttons::run_buttons();
    if (!menu_enabled && debug) {
      auto imu_state = imu_odom->getState();
      // controllermenu::partner_print_array[0] = std::to_string(imu_x.convert(inch)) + " " + std::to_string(gps.get_error());
      // controllermenu::partner_print_array[1] = std::to_string(imu_y.convert(inch)) + " " + std::to_string(gps.get_heading());
      // controllermenu::partner_print_array[2] = std::to_string(imu_theta.convert(degree)) + " " + std::to_string(imu.get_heading());
      // controllermenu::partner_print_array[2] = std::to_string(gps.get_error()) + " " + std::to_string(imu.get_heading());
      
      // auto odom = chassis->getState();
      auto gps_pos = gps.get_status();
      auto gps_theta = gps_pos.yaw;
      auto gps_x = gps_pos.x;
      auto gps_y = gps_pos.y;
      printf(("\nx " + std::to_string(imu_state.x.convert(inch))
              + " y " + std::to_string(imu_state.y.convert(inch))
              + " theta " + std::to_string(imu_state.theta.convert(degree))).c_str());
      auto state = chassis->getState();
      controllermenu::master_print_array[0] = std::to_string(state.x.convert(inch));
      controllermenu::master_print_array[1] = std::to_string(state.y.convert(inch));
      controllermenu::master_print_array[2] = std::to_string(state.theta.convert(degree));
      // controllermenu::master_print_array[0] = std::to_string(imu_state.x.convert(inch));
      // controllermenu::master_print_array[1] = std::to_string(imu_state.y.convert(inch));
      // controllermenu::master_print_array[2] = std::to_string(imu_state.theta.convert(degree));
    }
    pros::delay(10);
  }
}
