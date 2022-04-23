#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-drive.h"
#include "odom-utilities.h"
#include "lift.h"
#include <stdio.h>
#include <complex.h>
#include "odometry.h"


namespace autondrive {
DriveState drive_state = DriveState::kDisabled;

controllerbuttons::MacroGroup auton_group;
controllerbuttons::MacroGroup drive_group;

bool is_blue = true;

void set_odom_state(OdomState state) {
  chassis->setState(state);
  imu_odom->setState(state);
}

OdomState get_odom_state() {
  // auto pos = gps.get_status();
  // auto theta = imu.get_rotation();
  // int flip = 1;
  // double theta_offset = 180;
  // if (!is_blue){
  //   flip = -1;
  //   theta_offset = 0;
  // }
  // OdomState state({pos.x*1_m*flip, pos.y*1_m*flip, theta*1_deg});
  // // OdomState state({pos.x*1_m*flip, pos.y*1_m*flip, fmod(pos.yaw - theta_offset, 360)*1_deg});
  // return state;
  // return chassis->getState();
  return imu_odom->getState();
}

double button_strafe = 0;
double button_turn = 0;
double button_forward = 0;

namespace drivetoposition {


std::queue<Target> targets;
std::queue<Target> target_queue;

// bool auton_drive_enabled = false;
bool targets_should_clear = true;
bool final_target_reached = true;
bool target_heading_reached = false;
bool target_distance_reached = false;

OdomState starting_position;
RampMathSettings move_settings = {20, 100, 15, 0.1, 0.2};
RampMathSettings strafe_settings = {10, 50, 10, 0.1, 0.1}; // This is not used
RampMathSettings turn_settings = {10, 50, 10, 0.1, 0.1}; // This is not used
double turn_max_speed = 100;
double turn_p = 30;

double forward = 0;
double strafe  = 0;
double turn    = 0;

Target::Target(QLength x, QLength y, QAngle theta, bool hold, bool is_turn) : x(x), y(y), theta(theta), hold(hold), is_turn(is_turn) {}

void Target::init_if_new() {
  if (is_new) {
    is_new = false;
    starting_state = get_odom_state();
    Point starting_point = {starting_state.x, starting_state.y};
    QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, {x, y, theta});

    millis_at_start = pros::millis();
    int drive_timeout = total_distance.convert(inch) * 0.0334 * 100000/move_settings.mid_output + 700;
    int turn_timeout = abs((starting_state.theta - theta).convert(degree))*5 + 200;
    int timeout = drive_timeout + turn_timeout;
  }
}

void update_legacy() {
  if (targets_should_clear) {
    targets = {};
  }

  while (!target_queue.empty()) { 
    targets.push(target_queue.front());
    target_queue.pop();
    targets_should_clear = false;
  }

  if (targets.empty() || drive_state != DriveState::kLegacy) {
    forward = 0;
    strafe = 0;
    turn = 0;
    final_target_reached = true;
    return;
  }

  Target &target = targets.front();
  target.init_if_new();

  double move_speed;
  double strafe_speed = 0;
  double turn_speed;

  Point target_point{target.x, target.y};

  OdomState target_state{target.x, target.y, target.theta};
  Point starting_point{target.starting_state.x, target.starting_state.y};

  auto [distance_to_target, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, get_odom_state());
  QLength distance_traveled = OdomMath::computeDistanceToPoint(starting_point, get_odom_state());
  QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);

  QAngle total_angle = target.theta - target.starting_state.theta;


  // controllermenu::master_print_array[1] = std::to_string(total_distance.convert(inch)) + " " + std::to_string(distance_traveled.convert(inch));
  if (distance_traveled >= total_distance || abs(cos(direction.convert(radian))) < 0.1) {
    target_distance_reached = true;
  }

  // controllermenu::master_print_array[2] = "turn_diff: " + std::to_string(abs((get_odom_state().theta - target_state.theta).convert(degree)));
  if (abs(get_odom_state().theta - target_state.theta) < 10_deg) {
    target_heading_reached = true;
  }

  if (pros::millis() - target.millis_at_start > target.timeout) {
    controllermenu::master_print_array[1] = "TO " + std::to_string(target.timeout);

  }

  // if ((target_heading_reached && target_distance_reached)
  //     || pros::millis() - target.millis_at_start > target.timeout) {
  if (target_heading_reached && target_distance_reached) {
    // if (!targets.empty() && !(targets.size() == 1 && target.hold)) {
    if (!targets.empty()) {
      target_heading_reached = false;
      target_distance_reached = false;
      targets.pop();
      return;
    } else {
      final_target_reached = true;
    }
  } 
  
  if (target_distance_reached && target.hold) {
    move_speed = 0;
    // move_speed = std::min(100.0, 2 * distance_to_target.convert(inch));
  } else {
    move_speed = rampMath(distance_traveled.convert(inch), total_distance.convert(inch), move_settings);
    if (abs(distance_traveled - total_distance) > 5_in) {
      strafe_speed  = 2 * sin(direction.convert(radian));
    }
  }

  if (target_heading_reached && target.hold) {
    turn_speed = std::min(turn_max_speed, turn_p * (target.theta - get_odom_state().theta).convert(radian));
    // turn_speed = std::min(turn_max_speed, turn_p * direction.convert(radian));
  } else {
    turn_speed = std::min(turn_max_speed, turn_p * (target.theta - get_odom_state().theta).convert(radian));
    // turn_speed = std::min(turn_max_speed, turn_p * direction.convert(radian));
  }

  forward = move_speed * cos(direction.convert(radian));
  strafe  = strafe_speed;
  // strafe  = 0;
  turn    = turn_speed;
}

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance, QAngle offset_angle, bool hold = true) {
  drive_state == DriveState::kLegacy;
  QLength x_offset = cos(offset_angle) * offset_distance;
  QLength y_offset = sin(offset_angle) * offset_distance;
  final_target_reached = false;
  target_queue.push({x - x_offset, y - y_offset, theta, hold});
}

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance, bool hold = true) {
  add_target(x, y, theta, offset_distance, theta, hold);
}

void add_target(QLength x, QLength y, QAngle theta, bool hold = true) {
  add_target(x, y, theta, 0_in, hold);
}

void add_target(Point point, QAngle theta, QLength offset_distance, QAngle offset_angle, bool hold = true) {
  add_target(point.x, point.y, theta, offset_distance, offset_angle, hold);
}

void add_target(Point point, QAngle theta, QLength offset_distance, bool hold = true) {
  add_target(point, theta, offset_distance, theta, hold);
}

void add_target(Point point, QAngle theta, bool hold = true) {
  add_target(point, theta, 0_in, hold);
}

void clear_all_targets() {
  targets_should_clear = true;
}

using namespace controllerbuttons;

// blocking functions

void wait_until_final_target_reached() {
  while (!final_target_reached) {
    wait(10);
  }
}

void wait_until_final_target_reached(int timeout) {
  int start_time = pros::millis();
  while (!final_target_reached && pros::millis() - start_time < timeout) {
    wait(10);
  }
  clear_all_targets();
}

void wait_until_goal(int timeout) {
  int start_time = pros::millis();
  while (!final_target_reached && pros::millis() - start_time < timeout
         && (back_distance.get() > 110 || back_distance.get() == PROS_ERR)) {
    wait(10);
  }
  clear_all_targets();
}

} // namespace drivetoposition


void motor_task()
{
  Slew drive_left_slew(DRIVER_SLEW);
  Slew drive_right_slew(DRIVER_SLEW);
  Controller okapi_master;
  using namespace drivetoposition;
  while(1)
  {
    
    int stick_forward = master.get_analog(ANALOG_RIGHT_Y);
    int stick_turn = master.get_analog(ANALOG_RIGHT_X);
    std::string x_str = std::to_string(int(get_odom_state().x.convert(inch)*100));
    std::string y_str = std::to_string(int(get_odom_state().y.convert(inch)*100));
    std::string theta_str = std::to_string(get_odom_state().theta.convert(degree));

    double forward = button_forward + drivetoposition::forward + stick_forward * 0.787401574803;
    double temp_turn  = stick_turn * 0.787401574803;
    double turn    = button_turn + drivetoposition::turn + temp_turn + strafe;

    if (drive_state == DriveState::kLegacy) {
      update_legacy();
      double left_drive  = (forward + turn)/100.0;
      double right_drive = (forward - turn)/100.0;
      // double left_drive  = 1.27 * (forward + turn);
      // double right_drive = 1.27 * (forward - turn);
      chassis->getModel()->left(left_drive);
      chassis->getModel()->right(right_drive);
    } else if (drive_state == DriveState::kDisabled) {
      double left_drive  = drive_left_slew.new_value(stick_forward + stick_turn);
      double right_drive = drive_right_slew. new_value(stick_forward - stick_turn);
      chassis->getModel()->tank(left_drive/127.0, right_drive/127.0);
    }

    pros::delay(5);
  }
}



using namespace controllerbuttons;

// Macro goal_turn_right(
//     [](){
//       button_strafe = 100;
//       button_turn = -70;
//       while (true) {
//         button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2300));
//         wait(10);
//       }
//     },
//     [](){
//         button_strafe = 0;
//         button_turn = 0;
//         button_forward = 0;
//     },
//     {&drive_group});

// Macro goal_turn_left(
//     [](){
//       button_strafe = -100;
//       button_turn = 80;
//       while (true) {
//         button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2300));
//         wait(10);
//       }
//     },
//     [](){
//         button_strafe = 0;
//         button_turn = 0;
//         button_forward = 0;
//     },
//     {&drive_group});

Macro pull_platform(
    [](){
      //auton_init({0_in, 0_in, 0_deg});
      button_forward = 20;
      lift_motor.move_absolute(0, 127);
      while (lift_motor.get_position() < 10) {
        pros::delay(10);
      }
    },
    [](){
      button_forward = 0;
    },
    {&auton_group});

void drive_group_terminate() {
  drive_group.terminate();
  button_strafe = 0;
  button_turn = 0;
  button_forward = 0;
}

void set_callbacks() {
//   using namespace controllerbuttons;
//   button_handler.master.left.pressed.set_macro(goal_turn_left);
//   button_handler.master.right.pressed.set_macro(goal_turn_right);
//   button_handler.master.left.released.set(drive_group_terminate);
//   button_handler.master.right.released.set(drive_group_terminate);
//   // button_handler.master.right.pressed.set_macro(drivetoposition::ball_allign);
  // button_handler.master.left.pressed.set_macro(pull_platform);
}

} // namespace autondrive



namespace autonroutines {
using namespace autondrive;
using namespace drivetoposition;
using namespace odomutilities;
using namespace controllerbuttons;
using namespace robotfunctions;
using namespace rollers;

int start_time = 0;


void auton_log() {
  std::string odom_log = "x,y,theta,balls_in_robot,intake_queue,score_queue,eject_queue,target_distance_reached,target_heading_reached,final_target_reached,targets,goal_sensor_one.get_value(),goal_sensor_two.get_value()";
  std::string odom_log_number_old = "";
  std::string odom_log_number_new = "";

  for (int i = 0; i < 1500; i++) {
    OdomState odom = get_odom_state();
    odom_log.append("\n" +
        std::to_string(odom.x.convert(inch)) + "," +
        std::to_string(odom.y.convert(inch)) + "," +
        std::to_string(odom.theta.convert(degree)) + "," +
        std::to_string(target_distance_reached) + "," +
        std::to_string(target_heading_reached) + "," +
        std::to_string(final_target_reached) + "," +
        std::to_string(targets.size()));
    pros::delay(10); 
  }
  
  std::ifstream odl_i("/usd/odom_log_number.txt");
  odl_i >> odom_log_number_old;
  odl_i.close();

  odom_log_number_new = std::to_string(std::stoi(odom_log_number_old) + 1);

  std::ofstream odl_o("/usd/odom_log_number.txt");
  odl_o << odom_log_number_new << std::endl;
  odl_o.close();

  std::ofstream logfile(("/usd/auton_log_" + odom_log_number_new + ".csv").c_str());
  logfile << odom_log << std::endl;
  logfile.close();
}

void auton_init(OdomState odom_state, std::string name = "unnamed", bool is_skills = false) {
  set_odom_state(odom_state);
  // int flip = 1;
  // QAngle theta_offset = 180_deg;
  // if (!is_blue){
  //   flip = -1;
  //   theta_offset = 0_deg;
  // }
  // auto x     = odom_state.x.convert(meter) * flip;
  // auto y     = odom_state.y.convert(meter) * flip;
  // auto theta = odom_state.theta.convert(degree);
  // gps.set_position(x, y, theta);
  // imu.set_rotation(theta);
  start_time = pros::millis();
  // auton_drive_enabled = true;
  drive_state = DriveState::kLegacy;
  (pros::Task(auton_log));
}

void auton_clean_up() {
  clear_all_targets();
  drive_state = DriveState::kDisabled;
  // auton_drive_enabled = false;
  button_turn = 0;
  button_forward = 0;
  controllermenu::master_print_array[0] = "Completed";
  controllermenu::master_print_array[1] = "Time: " + std::to_string(pros::millis() - start_time);
  controllermenu::master_print_array[2] = "";
}


int ring_speed = 300;

auto front_goal_offset = 12_in;
auto back_goal_offset = -15.5_in;
auto back_goal_wall_offset = back_goal_offset + 8_in;

Macro none([](){},[](){});

// Macro legacy_test(
//     [](){
//       auton_init({0_in, 0_in, 0_deg});
//       drive_state = DriveState::kOkapi;
//       chassis->driveToPoint({10_in, 10_in});
//       chassis->turnToAngle(0_deg);
//       chassis->moveDistance(10_in);
//       chassis->turnToPoint({10_in, 10_in});
//       chassis->driveToPoint({0_in, 0_in});
//       chassis->turnToAngle(0_deg);
//       wait(1000);

//     },
//     [](){
//       auton_clean_up();
//     },
//     {&auton_group});

Macro legacy_test(
    [](){
      auton_init({26_in, goal_1.y, 0_deg});
      using namespace odomutilities;

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      turn_settings.mid_output = 30;

      // turn_settings.mid_output = 30;
      lift::claw_l.retract();
      lift_motor.move_absolute(-10, 100);
      add_target(goal_1, 0_deg, front_goal_offset);
      wait_until_final_target_reached();
      lift::claw_l.extend();
      // lift_motor.move_absolute(90, 100);
      lift_motor.move_absolute(360, 100);
      wait(1000);
      //pick up blue goal 1

      add_target(goal_3.x, goal_1.y, 0_deg);
      add_target(goal_3.x, goal_1.y, 90_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      add_target(goal_3.x, goal_7.y, 90_deg, back_goal_offset);
      // wait_until_final_target_reached(3500);

      // add_target(10_in, 0_in, 0_deg);
      // add_target(10_in, 0_in, 90_deg);
      // add_target(10_in, 10_in, 90_deg);
      // add_target(10_in, 10_in, 45_deg);
      // add_target(0_in, 0_in, 45_deg);
      // add_target(0_in, 0_in, 0_deg);
      wait_until_final_target_reached();
      wait(3000);
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});
    Macro right_sweep(
    [](){
      auton_init({0_in, 0_in, 0_deg});
      using namespace odomutilities;


      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;
      move_settings.ramp_up_p = 0.2;
      move_settings.ramp_down_p = 0.2;
      turn_p = 50;



      lift::claw_l.retract();
      lift::claw_r.retract();
      lift::shift.extend();
      
      lift_motor.move_absolute(-10, 100);
      add_target(37_in, 0_in, 0_deg);
      wait_until_final_target_reached();
      lift::claw_r.extend();
      add_target(43_in, 0_in, 0_deg);
      wait_until_final_target_reached();
      lift::claw_l.extend();
      lift::shift.retract();
      add_target(0_in, 0_in, 0_deg);
      lift_motor.move_absolute(0, 100);
      wait_until_final_target_reached();
    },
    [](){
      lift_motor.move_absolute(0, 100);
      auton_clean_up();
    },
    {&auton_group});

    Macro okapi_test(
    [](){
      auton_init({0_in, 0_in, 0_deg});
      drive_state = DriveState::kOkapi;
      chassis->driveToPoint({20_in, 20_in});
      chassis->turnToAngle(0_deg);
      chassis->moveDistance(20_in);
      chassis->turnToPoint({20_in, 20_in});
      chassis->driveToPoint({0_in, 0_in});
      chassis->turnToAngle(0_deg);
      wait(1000);

    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

    Macro profiling_test(
    [](){
      auton_init({0_in, 0_in, 0_deg});
      drive_state = DriveState::kOkapi;
      profileController->generatePath(
      {{0_ft, 0_ft, 0_deg}, {3_ft, 0_ft, 0_deg}}, "A");
      profileController->setTarget("A");
      profileController->waitUntilSettled();
      profileController->generatePath(
      {{3_ft, 0_ft, 0_deg}, {6_ft, 0_ft, 0_deg}}, "B");
      profileController->setTarget("B", true, true);

    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

    Macro skills_3(
    [](){
      auton_init({26_in, goal_1.y, 0_deg});
      using namespace odomutilities;


      move_settings.start_output = 20;
      move_settings.mid_output = 50;
      move_settings.end_output = 20;
      move_settings.ramp_up_p = 0.4;
      move_settings.ramp_down_p = 0.4;

    

      lift::claw_l.retract();
      lift::claw_l.retract();
      lift_motor.move_absolute(-10, 100);
      add_target(goal_1, 0_deg, front_goal_offset);
      wait_until_final_target_reached();
      // move_settings.mid_output = 20;
      // move_settings.end_output = 10;
      lift::claw_l.extend();
      lift_motor.move_absolute(360, 100);
      wait(1000);
      //pick up blue goal 1

      // turn_settings.mid_output = 10;
      add_target(goal_3.x-5_in, goal_1.y, 0_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 50;
      move_settings.end_output = 20;
      add_target(goal_3.x-5_in, goal_1.y, 90_deg);
      add_target(goal_3.x-5_in, 8_ft, 90_deg);
      lift::tilter.retract();
      add_target(goal_3.x-5_in, 8_ft, 102_deg);
      wait_until_final_target_reached();
      turn_p = 50;
      add_target(goal_3, 102_deg, 20_in, 90_deg);
      wait_until_final_target_reached();
      turn_p = 30;

      // move_settings.mid_output = 100;
      // move_settings.end_output = 20;

      add_target(goal_3, 90_deg, 20_in);
      add_target(goal_3.x, goal_7.y, 90_deg, back_goal_offset);
      wait_until_final_target_reached(3500);
      add_target(goal_3.x, goal_6.y-2_in, 90_deg);
      wait_until_final_target_reached();
      // push yellow goal 3


      add_target(goal_3.x, goal_6.y-2_in, 0_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 20;
      wait(20);
      add_target(24_in, goal_6.y-2_in, 0_deg);
      wait(20);
      wait_until_final_target_reached(2000);
      add_target(24_in, goal_6.y-2_in, -5_deg);
      lift::tilter.extend();
      wait(300);
      wait_until_final_target_reached();
      move_settings.mid_output = 100;
      wait(100);
      // pick up red goal 6

      add_target(goal_4.x, goal_6.y-2_in, -5_deg);
      wait_until_final_target_reached();
      add_target(goal_4.x, goal_6.y-2_in, -100_deg);
      wait_until_final_target_reached();
      add_target(goal_4.x, goal_6.y-2_in, -90_deg);
      wait_until_final_target_reached();
      // move_settings.mid_output = 20;
      add_target(goal_4.x, goal_6.y-4_in, -90_deg);
      ring_motor.move_velocity(ring_speed);
      wait_until_final_target_reached(700);
      // move_settings.mid_output = 100;
      // wait(500);
      lift::claw_l.retract();
      // drop blue goal 1 on blue platform

      // auto state = get_odom_state();
      // imu_odom->setState({29_in, state.x, state.theta});
      // //drive into platform

      add_target(goal_4.x, 32_in, -90_deg);
      add_target(goal_4.x, 32_in, 0_deg);
      wait_until_final_target_reached();
      ring_motor.move_velocity(0);
      add_target(goal_4.x, 32_in, -15_deg);
      add_target(8_ft, 30_in, -15_deg);
      add_target(8_ft, 30_in, 0_deg);
      add_target(10_ft+6_in, 30_in, 0_deg);

      wait_until_final_target_reached();
      // move_settings.mid_output = 20;
      add_target(14_ft, 30_in, 0_deg);
      wait_until_final_target_reached(1300);
      wait(100);
      auto state = get_odom_state();
      imu_odom->setState({133.5_in, state.y, state.theta});
      //drive into wall

      move_settings.mid_output = 100;
      move_settings.ramp_up_p = 0.6;
      add_target(10_ft, goal_6.y, 0_deg);
      add_target(10_ft, goal_6.y, -90_deg);
      wait_until_final_target_reached();
      add_target(10_ft, goal_7.y, -90_deg);
      wait_until_final_target_reached(2000);
      move_settings.mid_output = 20;
      add_target(10_ft, -12_in, -90_deg);
      wait_until_final_target_reached(2000);
      wait(100);
      state = get_odom_state(); // declared before
      imu_odom->setState({state.x, 8_in, state.theta});
      move_settings.mid_output = 100;
      //drive into other wall

      add_target(10_ft, goal_6.y, -90_deg);
      add_target(10_ft, goal_7.y, -90_deg);
      add_target(10_ft, goal_7.y, -180_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(0, 100);
      wait(1000);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      add_target(goal_7, -180_deg, front_goal_offset);
      wait_until_final_target_reached(1200);
      lift::claw_l.extend();
      move_settings.ramp_up_p = 0.4;
      wait(500);
      // pick up red goal 7

      add_target(10_ft+6_in, goal_7.y, -180_deg);
      wait_until_final_target_reached(1200);
      lift_motor.move_absolute(360, 100);
      wait(1000);
      add_target(goal_5.x, goal_7.y, -180_deg);
      add_target(goal_5.x, goal_7.y, -90_deg);
      add_target(goal_5.x, goal_2.y-16_in, -90_deg);
      // push yellow goal 5

      add_target(goal_5.x, goal_2.y, -200_deg);
      wait_until_final_target_reached();
      add_target(goal_5.x, goal_2.y, -200_deg, -32_in);
      add_target(goal_5.x, goal_2.y, -270_deg, -32_in, -200_deg);
      wait_until_final_target_reached(4500);
      add_target(goal_4.x-12_in, goal_2.y+4_in, -270_deg);
      wait_until_final_target_reached(700);
      move_settings.mid_output = 100;
      // wait(500);
      lift::claw_l.retract();
      // drop red goal 7 on red platform

      add_target(goal_4.x-12_in, goal_2.y, -270_deg);
      add_target(goal_4.x-12_in, goal_2.y, -360_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      wait(1000);
      add_target(goal_4.x+12_in, goal_2.y, -360_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      // drop red goal 6 on the mat

      wait(1000);
      add_target(goal_5.x, goal_2.y, -360_deg, front_goal_offset);
      wait_until_final_target_reached();
      lift::tilter.extend();
      lift::claw_l.extend();
      wait(200);
      // pick up yellow goal 5

      add_target(goal_4.x+6_in, goal_2.y, -360_deg);
      lift_motor.move_absolute(360, 100);
      wait(1000);
      add_target(goal_4.x+6_in, goal_2.y, -270_deg);
      wait_until_final_target_reached();
      add_target(goal_4.x+6_in, goal_2.y+4_in, -270_deg);
      // add_target(goal_4.x+12_in, goal_2.y+4_in, -270_deg);
      wait_until_final_target_reached(1000);
      lift::claw_l.retract();
      // drop yellow goal 5 on red platform

      add_target(goal_4.x+6_in, goal_2.y, -270_deg);
      add_target(goal_4.x+6_in, goal_2.y, -180_deg);
      wait_until_final_target_reached();
      add_target(goal_4.x+12_in, goal_2.y, -180_deg);
      lift_motor.move_absolute(-10, 100);
      wait(1000);
      add_target(goal_4.x, goal_2.y, -180_deg);
      wait_until_final_target_reached();
      lift::claw_l.extend();
      wait(500);
      // pick up red goal 6

      lift_motor.move_absolute(360, 100);
      wait(1000);
      turn_max_speed = 20;
      add_target(goal_4.x, goal_2.y, -270_deg);
      add_target(goal_4.x, goal_2.y+4_in, -270_deg);
      wait_until_final_target_reached(1500);
      turn_max_speed = 100;
      lift::claw_l.retract();
      // drop red goal 6 on red platform

      add_target(goal_4.x, goal_2.y, -270_deg);
      add_target(goal_4.x, goal_2.y, -360_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      wait(1000);
      wait_until_final_target_reached();
      add_target(goal_2, -360_deg, front_goal_offset);
      wait_until_final_target_reached(2000);
      lift::claw_l.extend();
      wait(100);
      // pick up blue goal two

      add_target(goal_4, -360_deg, 49.5_in, -495_deg);
      add_target(goal_4, -495_deg, front_goal_offset);
      wait_until_final_target_reached();
      add_target(goal_6.x, goal_7.y, -495_deg, front_goal_offset);
      wait_until_final_target_reached();
      // push yellow goal 4
    
      
      // lift_motor.move_absolute(-10, 100);
      // add_target(goal_3.x, goal_6.y, -495_deg);
      // add_target(goal_3.x, goal_6.y, -90_deg);
      // wait(1000);
      // add_target(goal_3.x, goal_7.y, -90_deg, front_goal_offset);
      // wait_until_final_target_reached();
      // lift::claw_l.extend();
      // wait(200);
      // // pick up yellow goal 3

      // lift_motor.move_absolute(360, 100);
      // add_target(goal_3.x, goal_6.y+6_in, -90_deg);
      // add_target(goal_3.x, goal_6.y+6_in, 0_deg);
      // add_target(goal_4.x, goal_6.y+6_in, 0_deg);
      // add_target(goal_4.x, goal_6.y+6_in, -90_deg);
      // wait_until_final_target_reached();
      // move_settings.mid_output = 20;
      // add_target(goal_4.x, goal_6.y-4_in, -90_deg);
      // wait_until_final_target_reached(1000);
      // move_settings.mid_output = 100;
      // // wait(500);
      // lift::claw_l.retract();
      // add_target(goal_4, -90_deg);

      wait_until_final_target_reached();
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

} // namespace autonroutines