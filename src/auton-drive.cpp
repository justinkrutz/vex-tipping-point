#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-drive.h"
#include "odom-utilities.h"
#include <stdio.h>
#include <complex.h>
#include "odometry.h"


namespace autondrive {

controllerbuttons::MacroGroup auton_group;
controllerbuttons::MacroGroup drive_group;

bool is_blue = true;

OdomState get_odom_state() {
  auto pos = gps.get_status();
  int flip = 1;
  QAngle theta_offset = 180_deg;
  if (!is_blue){
    flip = -1;
    theta_offset = 0_deg;
  }
  OdomState state({pos.x*1_m*flip, pos.y*1_m*flip, pos.yaw*1_deg - theta_offset});
  return state;
  // return imu_odom->getState();
}

double button_strafe = 0;
double button_turn = 0;
double button_forward = 0;

namespace drivetoposition {

Target::Target(QLength x, QLength y, QAngle theta, bool hold) : x(x), y(y), theta(theta), hold(hold) {}

void Target::init_if_new() {
  if (is_new) {
    is_new = false;
    starting_state = get_odom_state();
  }
}

std::queue<Target> targets;
std::queue<Target> target_queue;

bool auton_drive_enabled = false;
bool targets_should_clear = true;
bool final_target_reached = true;
bool target_heading_reached = false;
bool target_distance_reached = false;

OdomState starting_position;
// RampMathSettings move_settings = {20, 100, 20, 0.5, 0.5};
// RampMathSettings turn_settings = {20, 100, 20, 0.1, 0.1};
RampMathSettings move_settings = {20, 100, 15, 0.1, 0.2};
RampMathSettings turn_settings = {10, 50, 10, 0.1, 0.1};

double forward = 0;
double strafe  = 0;
double turn    = 0;


void update() {
  if (targets_should_clear) {
    targets = {};
  }

  while (!target_queue.empty()) { 
    targets.push(target_queue.front());
    target_queue.pop();
    targets_should_clear = false;
  }

  if (targets.empty() || !auton_drive_enabled) {
    forward = 0;
    strafe = 0;
    turn = 0;
    final_target_reached = true;
    return;
  }

  Target &target = targets.front();
  target.init_if_new();

  double move_speed;
  double turn_speed;

  Point target_point{target.x, target.y};

  OdomState target_state{target.x, target.y, target.theta};
  Point starting_point{target.starting_state.x, target.starting_state.y};

  auto [distance_to_target, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, get_odom_state());
  QLength distance_traveled = OdomMath::computeDistanceToPoint(starting_point, get_odom_state());
  QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);

  QAngle total_angle = target.theta - target.starting_state.theta;


  if (distance_traveled >= total_distance) {
    target_distance_reached = true;
  }

  if (abs(get_odom_state().theta - target_state.theta) < 2.5_deg) {
    target_heading_reached = true;
  }

  if (target_heading_reached && target_distance_reached) {
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
    move_speed = std::min(100.0, 5 * distance_to_target.convert(inch));
  } else {
    move_speed = rampMath(distance_traveled.convert(inch), total_distance.convert(inch), move_settings);
  }

  if (target_heading_reached && target.hold) {
    turn_speed = std::min(100.0, 100 * (target.theta - get_odom_state().theta).convert(radian));
  } else {
    turn_speed = std::min(100.0, 100 * (target.theta - get_odom_state().theta).convert(radian));
  }

  forward = move_speed * cos(direction.convert(radian));
  strafe  = move_speed * sin(direction.convert(radian));
  turn    = turn_speed;
  controllermenu::master_print_array[1] = "d " + std::to_string(int(direction.convert(degree))) + " tdr " + std::to_string(int(target_distance_reached)) + " thr " + std::to_string(int(target_heading_reached));
  controllermenu::master_print_array[2] = "f " + std::to_string(int(forward)) + " t " + std::to_string(int(turn)) + " s " + std::to_string(int(strafe));
}

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance, QAngle offset_angle, bool hold = true) {
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

void add_target(odomutilities::Goal goal, QAngle theta, QLength offset_distance, QAngle offset_angle, bool hold = true) {
  add_target(goal.point.x, goal.point.y, theta, offset_distance, offset_angle, hold);
}

void add_target(odomutilities::Goal goal, QAngle theta, QLength offset_distance, bool hold = true) {
  add_target(goal, theta, offset_distance, theta, hold);
}

void add_target(odomutilities::Goal goal, QAngle theta, bool hold = true) {
  add_target(goal, theta, 0_in, hold);
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

} // namespace drivetoposition


void motor_task()
{
//  std::shared_ptr<ChassisModel> model = chassis->getModel();
  // std::shared_ptr<AbstractMotor> drive_left = chassis->getTopLeftMotor();
  // std::shared_ptr<AbstractMotor> drive_left = chassis->getTopLeftMotor();
  // std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  // std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  // std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();

  // int time_last = 0;
  Slew drive_fl_slew(DRIVER_SLEW);
  Slew drive_fr_slew(DRIVER_SLEW);
  Slew drive_bl_slew(DRIVER_SLEW);
  Slew drive_br_slew(DRIVER_SLEW);

  while(1)
  {
    drivetoposition::update();
    
    int stick_forward = master.get_analog(ANALOG_RIGHT_Y);
    int stick_turn = master.get_analog(ANALOG_RIGHT_X);
    // controllermenu::master_print_array[1] = std::to_string(stick_forward);
    // controllermenu::master_print_array[2] = std::to_string(stick_turn);
    std::string x_str = std::to_string(int(get_odom_state().x.convert(inch)*100));
    std::string y_str = std::to_string(int(get_odom_state().y.convert(inch)*100));
    std::string theta_str = std::to_string(int(get_odom_state().theta.convert(degree)*100));
    // controllermenu::master_print_array[1] = "x " + x_str + " y " + y_str + " t " + theta_str;

    double forward = button_forward + drivetoposition::forward + stick_forward * 0.787401574803;
    // double strafe  = button_strafe + drivetoposition::strafe;
    // double strafe  = button_strafe + drivetoposition::strafe  + master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    // double strafe  = button_strafe + drivetoposition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    double temp_turn  = stick_turn * 0.787401574803;
    double turn    = button_turn + drivetoposition::turn + temp_turn;
    // double turn    = button_turn + drivetoposition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
    double sync = std::min(1.0, 100 / (fabs(forward) + fabs(turn)));

    double drive_fl_pct = drive_fl_slew.new_value((forward + turn) * sync);
    double drive_fr_pct = drive_fr_slew.new_value((forward - turn) * sync);
    double drive_bl_pct = drive_bl_slew.new_value((forward + turn) * sync);
    double drive_br_pct = drive_br_slew.new_value((forward - turn) * sync);
    // model->left(forward + turn);
    // model->right(forward - turn);

    // drive_fl->moveVelocity(drive_fl_pct * 2);
    // drive_fr->moveVelocity(drive_fr_pct * 2);
    // drive_bl->moveVelocity(drive_bl_pct * 2);
    // drive_br->moveVelocity(drive_br_pct * 2);

    // model->arcade(master.get_analog(ANALOG_RIGHT_Y), master.get_analog(ANALOG_RIGHT_X));
    // skid_model->getLeftSideMotor()->moveVoltage(stick_forward + stick_turn);
    // skid_model->getRightSideMotor()->moveVoltage(stick_forward - stick_turn);
    // skid_model->arcade(stick_forward, stick_turn);

    if (drivetoposition::auton_drive_enabled) {
      // controllermenu::master_print_array[2] = "auton_drive_enabled";
      double left_drive  = 6 * (forward + turn);
      double right_drive = 6 * (forward - turn);
      left_front.  move_velocity(left_drive);
      left_middle. move_velocity(left_drive);
      left_back.   move_velocity(left_drive);
      right_front. move_velocity(right_drive);
      right_middle.move_velocity(right_drive);
      right_back.  move_velocity(right_drive);
    } else {
      double left_drive  = stick_forward + stick_turn;
      double right_drive = stick_forward - stick_turn;
      left_front   = left_drive;
      left_middle  = left_drive;
      left_back    = left_drive;
      right_front  = right_drive;
      right_middle = right_drive;
      right_back   = right_drive;
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
        // std::to_string(balls_in_robot.size()) + "," +
        // std::to_string(intake_queue) + "," +
        // std::to_string(score_queue) + "," +
        // std::to_string(eject_queue) + "," +
        std::to_string(target_distance_reached) + "," +
        std::to_string(target_heading_reached) + "," +
        std::to_string(final_target_reached) + "," +
        std::to_string(targets.size()));
        // std::to_string(targets.size()) + "," +
        // std::to_string(goal_sensor_one.get_value()) + "," +
        // std::to_string(goal_sensor_two.get_value()));
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
  // imu_odom->setState(odom_state);
  int flip = 1;
  QAngle theta_offset = 180_deg;
  if (!is_blue){
    flip = -1;
    theta_offset = 0_deg;
  }
  auto x     = odom_state.x.convert(meter) * flip;
  auto y     = odom_state.y.convert(meter) * flip;
  auto theta = (odom_state.theta - theta_offset).convert(degree);
  gps.set_position(x, y, theta);
  start_time = pros::millis();
  auton_drive_enabled = true;
  (pros::Task(auton_log));
}

void auton_clean_up() {
  clear_all_targets();
  auton_drive_enabled = false;
  // stow_after_eject = false;
  // button_strafe = 0;
  button_turn = 0;
  button_forward = 0;
  controllermenu::master_print_array[0] = "Completed";
  controllermenu::master_print_array[1] = "Time: " + std::to_string(pros::millis() - start_time);
  controllermenu::master_print_array[2] = "";
}

Macro none([](){},[](){});


Macro test(
    [](){
      auton_init({13.491_in, 34.9911_in, 270_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      add_target(18_in, 34.9911_in, 270_deg);
      // add_target(goal_1, -135_deg, 25_in);
      // add_target(goal_1, -135_deg, 30_in);
      // add_target(goal_1, 0_deg, 30_in, -135_deg);
      add_target(13.491_in, 34.9911_in, 0_deg);

      wait_until_final_target_reached();
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

Macro blue_wp(
    [](){
      auton_init({40_in, 60_in, 270_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      add_target(40_in, 60_in, 315_deg);
      add_target(40_in, 60_in, 315_deg, -16_in);
      // add_target(45_in, 55_in, 0_deg);
      // add_target(36_in, 60_in, 0_deg);
      // add_target(goal_1, -135_deg, 25_in);
      // add_target(goal_1, -135_deg, 30_in);
      // add_target(goal_1, 0_deg, 30_in, -135_deg);
      // add_target(13.491_in, 34.9911_in, 0_deg);

      wait_until_final_target_reached();
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

Macro red_wp(
    [](){
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

} // namespace autonroutines