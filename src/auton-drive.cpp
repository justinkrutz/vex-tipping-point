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

controllerbuttons::MacroGroup auton_group;
controllerbuttons::MacroGroup drive_group;

bool is_blue = true;

OdomState get_odom_state() {
  auto pos = gps.get_status();
  auto theta = imu.get_rotation();
  int flip = 1;
  double theta_offset = 180;
  if (!is_blue){
    flip = -1;
    theta_offset = 0;
  }
  OdomState state({pos.x*1_m*flip, pos.y*1_m*flip, theta*1_deg});
  // OdomState state({pos.x*1_m*flip, pos.y*1_m*flip, fmod(pos.yaw - theta_offset, 360)*1_deg});
  return state;
  // return imu_odom->getState();
}

double button_strafe = 0;
double button_turn = 0;
double button_forward = 0;
bool stick_control_enabled = true;

namespace drivetoposition {

Target::Target(QLength x, QLength y, QAngle theta, bool hold, bool is_turn) : x(x), y(y), theta(theta), hold(hold), is_turn(is_turn) {}

void Target::init_if_new() {
  if (is_new) {
    is_new = false;
    starting_state = get_odom_state();
    left_front.tare_position();
    right_front.tare_position();
    millis_at_start = pros::millis();
  }
}

std::queue<Target> targets;
std::queue<Target> target_queue;

bool auton_drive_enabled = false;
bool targets_should_clear = true;
bool final_target_reached = true;
bool target_heading_reached = false;
bool target_distance_reached = false;
QLength proximity_to_target = 0_in;

OdomState starting_position;
// RampMathSettings move_settings = {20, 100, 20, 0.5, 0.5};
// RampMathSettings turn_settings = {20, 100, 20, 0.1, 0.1};
RampMathSettings move_settings = {20, 100, 15, 0.1, 0.2};
RampMathSettings turn_settings = {10, 50, 10, 0.1, 0.1};

double turn_max_speed = 100;
double turn_p = 0.5;

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

  double x_d = target.x.convert(inch);

  double distance_moved = fabs(((right_front.get_position() + left_front.get_position())/2)*5.62753492038/360);

  Point target_point{target.x, target.y};

  OdomState target_state{target.x, target.y, target.theta};
  Point starting_point{target.starting_state.x, target.starting_state.y};

  auto [distance_to_target, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, get_odom_state());
  proximity_to_target = distance_to_target;
  QLength distance_traveled = OdomMath::computeDistanceToPoint(starting_point, get_odom_state());
  QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);

  QAngle total_angle = target.theta - target.starting_state.theta;


  if (fabs(x_d) <= distance_moved) {
    target_distance_reached = true;
  }

  double theta_error = (target.theta - get_odom_state().theta).convert(degree);
  if (fabs(theta_error) < 2.5) {
    target_heading_reached = true;
  }

  if ((target_heading_reached && target.is_turn) || (target_distance_reached && !target.is_turn)) {
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
    // move_speed = std::min(100.0, 5 * distance_to_target.convert(inch));
    move_speed = 0;
  } else {
    move_speed = rampMath(distance_moved, fabs(x_d), move_settings);
  }


  if (target_heading_reached && target.hold) {
    turn_speed = turn_p * theta_error;
  } else {
    turn_speed = turn_p * theta_error;
  }

  turn = std::max(-turn_max_speed, std::min(turn_max_speed, turn_speed));
  if (target.is_turn) {
    forward = 0;
    strafe = 0;
  } else {
    // turn = 0;
    // controllermenu::master_print_array[2] = "d " + std::to_string(direction.convert(degree));
    forward = move_speed * (fabs(x_d)/x_d);
    // forward = 20 * (target.is_fwd*2-1);
    // forward = move_speed * cos(direction.convert(radian));
    // forward = move_speed;
    // strafe  = move_speed * sin(direction.convert(radian));
  }
  // controllermenu::master_print_array[1] = "d " + std::to_string(int(direction.convert(degree))) + " tdr " + std::to_string(int(target_distance_reached)) + " thr " + std::to_string(int(target_heading_reached));
  // controllermenu::master_print_array[2] = "f " + std::to_string(int(forward)) + " t " + std::to_string(int(turn)) + " s " + std::to_string(int(strafe));
}

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance, QAngle offset_angle, bool hold = true, bool is_turn = false) {
  QLength x_offset = cos(offset_angle) * offset_distance;
  QLength y_offset = sin(offset_angle) * offset_distance;
  final_target_reached = false;
  target_queue.push({x - x_offset, y - y_offset, theta, hold, is_turn});
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

void add_target(QAngle theta, bool hold = true) {
  add_target(0_in, 0_in, theta, 0_in, 0_deg, hold, true);
}

void add_target(QLength distance, QAngle theta, bool hold = true) {
  add_target(distance, 0_in, theta, 0_in, hold);
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

void wait_until_clamp_or_target() {
  while (!final_target_reached && !lift::claw.piston_out) {
    wait(10);
  }
  clear_all_targets();
}

void wait_until_proximity(QLength proximity) {
  while (!final_target_reached && proximity_to_target < proximity) {
    wait(10);
  }
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

void reset_button_task(){
  lv_obj_t * calibration_status =  lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(calibration_status, "Calibrated on boot");
  lv_obj_align(calibration_status, NULL, LV_ALIGN_CENTER, 0, 0);
  while(true) {
    if (reset_button.get_new_press()) {
      pros::delay(500);
      lv_label_set_text(calibration_status, "Calibrating... please wait");
      // lv_obj_align(calibration_status, NULL, LV_ALIGN_CENTER, 0, 0);
      lift_motor.tare_position();
      imu.reset();
      while (imu.is_calibrating()) {
        pros::delay(10);
      }
      lv_label_set_text(calibration_status, "Finished IMU Calibration!");
      // lv_obj_align(calibration_status, NULL, LV_ALIGN_CENTER, 0, 0);
    }
    pros::delay(50);
  }
}

void motor_task()
{
//  std::shared_ptr<ChassisModel> model = chassis->getModel();
  // std::shared_ptr<AbstractMotor> drive_left = chassis->getTopLeftMotor();
  // std::shared_ptr<AbstractMotor> drive_left = chassis->getTopLeftMotor();
  // std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  // std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  // std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();

  // int time_last = 0;
  Slew drive_left_slew(DRIVER_SLEW);
  Slew drive_right_slew(DRIVER_SLEW);

  while(1)
  {
    drivetoposition::update();

    int stick_forward = 0;
    int stick_turn = 0;
    if (stick_control_enabled) {
      stick_forward = input_curve(master.get_analog(ANALOG_RIGHT_Y));
      stick_turn = input_curve(master.get_analog(ANALOG_RIGHT_X));
    }
    // controllermenu::master_print_array[1] = std::to_string(stick_forward);
    // controllermenu::master_print_array[2] = std::to_string(stick_turn);
    std::string x_str = std::to_string(int(get_odom_state().x.convert(inch)*100));
    std::string y_str = std::to_string(int(get_odom_state().y.convert(inch)*100));
    std::string theta_str = std::to_string(get_odom_state().theta.convert(degree));
    // controllermenu::master_print_array[1] = "x " + x_str + " y " + y_str;
    // controllermenu::master_print_array[2] = "t " + theta_str;

    double forward = button_forward + drivetoposition::forward + stick_forward * 0.787401574803;
    double temp_turn  = stick_turn * 0.787401574803;
    double turn    = button_turn + drivetoposition::turn + temp_turn;
    double sync = std::min(1.0, 100 / (fabs(forward) + fabs(turn)));
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
      double left_drive  = drive_left_slew.new_value(stick_forward + stick_turn);
      double right_drive = drive_right_slew. new_value(stick_forward - stick_turn);
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

Macro pull_platform(
    [](){
      //auton_init({0_in, 0_in, 0_deg});
      button_forward = 20;
      lift_motor.move_absolute(0, 127);
      while (lift_motor.get_position() < 10) {
        wait(10);
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
  button_handler.master.left.pressed.set_macro(pull_platform);
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
    wait(10); 
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
  auto theta = odom_state.theta.convert(degree);
  gps.set_position(x, y, theta);
  imu.set_rotation(theta);
  start_time = pros::millis();
  auton_drive_enabled = true;
  (pros::Task(auton_log));
}

void auton_clean_up() {
  clear_all_targets();
  auton_drive_enabled = false;
  lift::auto_grip_enabled = true;
  lift::shooter.retract();
  // stow_after_eject = false;
  // button_strafe = 0;
  button_turn = 0;
  button_forward = 0;
  controllermenu::master_print_array[0] = "Completed";
  controllermenu::master_print_array[1] = "Time: " + std::to_string(pros::millis() - start_time);
  controllermenu::master_print_array[2] = "";
}


int ring_speed = 300;
auto kickstand_offset_distance = 4.5_in;
auto kickstand_offset_angle = 0_deg;

auto defaut_kickstand_distance = 39.5_in;
auto defaut_no_kickstand_distance = 43_in;

void goal_rush(QLength first_distance, QAngle first_angle, QLength second_distance, QAngle second_angle, bool tall_goal = false, int end_speed = 100) {
  move_settings.ramp_up_p = 0.3;
  move_settings.ramp_down_p = 0.3;
  move_settings.start_output = 100;
  move_settings.mid_output = 100;
  move_settings.end_output = end_speed;

  lift::auto_grip_enabled = true;
  lift::auto_grip_ready = true;
  lift::goal_detect_center_only = false;

  lift::claw.retract();
  add_target(first_distance, first_angle);
  lift_motor.move(-127);
  wait(250);
  lift_motor.tare_position();
  lift_motor.move(0);
  if (tall_goal) {
    lift_motor.move_absolute(20, 100);
  } else {
    lift_motor.move_absolute(30, 100);
  }

  wait_until_final_target_reached();
  lift::claw.extend();
  lift::auto_grip_ready = false;
  if (tall_goal) wait(200);
  move_settings.end_output = 20;
  add_target(second_distance, second_angle);

  if (tall_goal) {
    lift_motor.move_absolute(100, 100);
  } else {
    lift_motor.move_absolute(360, 100);
  }
  
  wait_until_final_target_reached();
}

Macro none([](){},[](){});

Macro test(
    [](){
      auton_init({0_in, 0_in, -32_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 20;
      move_settings.end_output = 15;
      lift::tilter.retract();
      wait(500);
      add_target(-14_in, -32_deg);
      // add_target(-20_in, -122_deg);

      // wait_until_final_target_reached(1000);
      wait_until_final_target_reached();
      wait(500);
      lift::tilter.extend();
      wait(1000);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(-ring_speed);
      wait(300);
      ring_motor.move_velocity(ring_speed);
      wait(500);
      ring_motor.move_velocity(0);
      // add_target(6_in, -32_deg);
      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

void option_right_yellow_ml (bool kickstand) {
  auton_init({0_in, 0_in, 0_deg});


  if (kickstand) {
    goal_rush(defaut_kickstand_distance, 0_deg, -26_in, 0_deg);
  } else {
    goal_rush(defaut_no_kickstand_distance, 0_deg, -26_in, 0_deg);
  }

  move_settings.start_output = 20;
  move_settings.mid_output = 100;
  move_settings.end_output = 20;

  lift::tilter.retract();
  add_target(-90_deg);
  wait_until_final_target_reached();
  move_settings.mid_output = 40;
  add_target(-18_in, -90_deg);
  wait_until_final_target_reached(1200);
  move_settings.mid_output = 100;
  lift::tilter.extend();
  wait(500);
  ring_motor.move_velocity(400);
  move_settings.mid_output = 20;
  move_settings.ramp_down_p = 0.4;
  move_settings.ramp_up_p = 0.4;
  add_target(-180_deg);
  wait_until_final_target_reached();
  add_target(24_in, -180_deg);
  ring_motor.move_velocity(600);
  wait_until_final_target_reached();
  
  move_settings.mid_output = 100;
  wait(11);
  add_target(-24_in, -180_deg);
  wait(700);
  ring_motor.move_velocity(-600);
  wait_until_final_target_reached(3000);

  move_settings.mid_output = 20;
  wait(11);
  ring_motor.move_velocity(600);
  add_target(24_in, -180_deg);
  wait_until_final_target_reached(3000);

  add_target(-6_in, -180_deg);
  wait_until_final_target_reached();
  add_target(-90_deg);
  turn_p = 0.3;
  wait_until_final_target_reached();
  lift::tilter.retract();

  wait_until_final_target_reached();
}

Macro right_yellow_ml(
    [](){ option_right_yellow_ml(false); },
    auton_clean_up,
    {&auton_group});

Macro kickstand_right_yellow_ml(
    [](){ option_right_yellow_ml(true); },
    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

void option_left_yellow_ml(bool kickstand) {
  auton_init({57_in, 32_in, 10_deg});

  if (kickstand) {
    lift::shooter.extend();
    goal_rush(defaut_kickstand_distance, 10_deg, -48_in, 10_deg);
  } else {
    goal_rush(defaut_no_kickstand_distance, 10_deg, -48_in, 10_deg);
  }
  lift::shooter.retract();

  move_settings.start_output = 20;
  move_settings.mid_output = 20;
  move_settings.end_output = 20;
  move_settings.ramp_down_p = 0.3;
  move_settings.ramp_up_p = 0.3;


  add_target(-12_in, 0_deg);
  wait_until_final_target_reached(700);
  add_target(2_in, 0_deg);

  add_target(-90_deg);
  wait_until_final_target_reached();
  add_target(-2_in, -90_deg);
  lift_motor.move_absolute(360, 100);
  wait(1000);
  add_target(7_in, -90_deg);
  wait_until_final_target_reached(1000);
  lift::tilter.retract();
  // wait(500);
  add_target(-80_deg);
  add_target(-14_in, -80_deg);
  wait_until_final_target_reached(3000);
  wait(200);
  lift::tilter.extend();
  wait(200);

  move_settings.start_output = 20;
  move_settings.mid_output = 20;
  move_settings.end_output = 20;
  // add_target(-90_deg);
  add_target(48_in, -90_deg);
  ring_motor.move_velocity(400);
  wait(1000);
  ring_motor.move_velocity(600);
  wait_until_final_target_reached(2000);
  add_target(-16_in, -90_deg);
  wait_until_final_target_reached();
  add_target(-240_deg);

  // move_settings.start_output = 100;
  // move_settings.mid_output = 100;
  // move_settings.end_output = 100;
  // add_target(-6_in, 270_deg);
  // add_target(4_in, 270_deg);
  wait_until_final_target_reached(700);
  lift::tilter.retract();
}

Macro left_yellow_ml(
    [](){ option_left_yellow_ml(false); },
    auton_clean_up,
    {&auton_group});

Macro kickstand_left_yellow_ml(
    [](){ option_left_yellow_ml(true); },
    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

void option_left_center(bool kickstand) {
  auton_init({57_in, 32_in, 10_deg});

  if (kickstand) {
    lift::shooter.extend();
    goal_rush(defaut_kickstand_distance, 10_deg, -29_in, 0_deg);
  } else {
    goal_rush(defaut_no_kickstand_distance, 10_deg, -29_in, 0_deg);
  }
  lift::shooter.retract();

  move_settings.start_output = 20;
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  turn_p = 0.5;
  turn_max_speed = 50;

  add_target(-90_deg);
  add_target(-36_in, -90_deg);
  add_target(-180_deg);
  wait_until_final_target_reached();
  lift::tilter.retract();
  add_target(-17_in, -180_deg);
  wait_until_final_target_reached();

  move_settings.mid_output = 20;
  move_settings.end_output = 15;
  add_target(-12_in, -180_deg);
  wait_until_final_target_reached(3000);
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  lift::tilter.extend();
  wait(1300);
  turn_max_speed = 10;
  add_target(57_in, -135_deg);
  wait_until_final_target_reached();
}

Macro left_center(
    [](){ option_left_center(false); },
    auton_clean_up,
    {&auton_group});

Macro kickstand_left_center(
    [](){ option_left_center(true); },
    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

void option_right_center_pull(bool kickstand) {
  auton_init({57_in, 32_in, 0_deg});

  if (kickstand) {
    goal_rush(defaut_kickstand_distance, 0_deg, -24_in, 0_deg);
  } else {
    goal_rush(defaut_no_kickstand_distance, 0_deg, -24_in, 0_deg);
  }

  move_settings.start_output = 20;
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  turn_p = 0.3;
  turn_max_speed = 40;

  add_target(-45_deg);
  add_target(19_in, -45_deg);
  add_target(135_deg);
  wait_until_final_target_reached();

  lift::tilter.retract();
  wait(500);
  move_settings.mid_output = 20;
  move_settings.end_output = 15;
  add_target(-24_in, 135_deg);
  wait_until_final_target_reached(3000);
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  lift::tilter.extend();
  wait(1300);
  add_target(57_in, 135_deg);
  wait_until_final_target_reached();
}

Macro right_center_pull(
    [](){ option_right_center_pull(false); },
    auton_clean_up,
    {&auton_group});

Macro kickstand_right_center_pull(
    [](){ option_right_center_pull(true); },

    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

void option_right_center_fast(bool kickstand) {
  auton_init({57_in, 32_in, 0_deg});

  if (kickstand) {
    goal_rush(defaut_kickstand_distance, 0_deg, -6_in, 0_deg, false);
  } else {
    goal_rush(defaut_no_kickstand_distance, 0_deg, -6_in, 0_deg, false);
  }

  move_settings.start_output = 20;
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  turn_p = 0.3;
  turn_max_speed = 40;

  add_target(90_deg);
  wait_until_final_target_reached();
  add_target(-12_in, 90_deg);
  wait(200);
  lift::tilter.retract();
  add_target(115_deg);
  wait_until_final_target_reached();

  move_settings.mid_output = 20;
  move_settings.end_output = 15;
  add_target(-14_in, 115_deg);
  wait_until_final_target_reached(3000);
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  lift::tilter.extend();
  wait(1300);
  add_target(57_in, 135_deg);
  wait_until_final_target_reached();
}

Macro right_center_fast(
    [](){ option_right_center_fast(false); },
    auton_clean_up,
    {&auton_group});

Macro kickstand_right_center_fast(
    [](){ option_right_center_fast(true); },

    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

void option_right_center_shoot(bool kickstand) {
  auton_init({57_in, 32_in, 0_deg});

  if (kickstand) {
    goal_rush(defaut_kickstand_distance, 0_deg, -6_in, 0_deg, false);
  } else {
    goal_rush(defaut_no_kickstand_distance, 0_deg, -6_in, 0_deg, false);
  }

  move_settings.start_output = 20;
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  turn_p = 0.3;
  turn_max_speed = 40;

  add_target(90_deg);
  wait_until_final_target_reached();
  add_target(-12_in, 90_deg);
  wait(200);
  lift::tilter.retract();
  add_target(115_deg);
  wait_until_final_target_reached();

  move_settings.mid_output = 20;
  move_settings.end_output = 15;
  add_target(-14_in, 115_deg);
  wait_until_final_target_reached(3000);
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  lift::tilter.extend();
  wait(1300);
  add_target(57_in, 135_deg);
  wait_until_final_target_reached();
}

Macro right_center_shoot(
    [](){ option_right_center_shoot(false); },
    auton_clean_up,
    {&auton_group});

Macro kickstand_right_center_shoot(
    [](){ option_right_center_shoot(true); },

    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

void option_center_right_fast(bool kickstand) {
  auto starting_angle = -35_deg;
  auton_init({0_in, 0_in, starting_angle});

  if (kickstand) {
    goal_rush(47_in, starting_angle, -6_in, starting_angle, true, 100);
  } else {
    goal_rush(6_in+defaut_no_kickstand_distance, starting_angle, -6_in, starting_angle, true, 100);
  }

  move_settings.start_output = 20;
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  turn_p = 0.4;
  turn_max_speed = 40;
  move_settings.ramp_up_p = 0.2;
  move_settings.ramp_down_p = 0.2;

  wait(100);
  add_target(-90_deg);
  wait_until_final_target_reached();
  add_target(-8_in, -90_deg);
  wait(200);
  lift::tilter.retract();
  add_target(-120_deg);
  wait_until_final_target_reached();

  move_settings.mid_output = 40;
  move_settings.end_output = 20;
  add_target(-10_in, -120_deg);
  wait_until_final_target_reached(3000);
  move_settings.mid_output = 100;
  move_settings.end_output = 20;
  lift::tilter.extend();
  wait(500);
  add_target(45_in, -135_deg);
  add_target(-90_deg);
  wait_until_final_target_reached();
}

Macro center_right_fast(
    [](){ option_center_right_fast(false); },
    auton_clean_up,
    {&auton_group});

Macro kickstand_center_right_fast(
    [](){ option_center_right_fast(true); },

    auton_clean_up,
    {&auton_group});

/* ------------------------------------------------------------- */

Macro extra_far(
    [](){
      auton_init({0_in, 0_in, 0_deg});

      move_settings.ramp_up_p = 0.3;
      move_settings.ramp_down_p = 0.3;
      move_settings.start_output = 100;
      move_settings.mid_output = 100;
      move_settings.end_output = 100;

      lift::auto_grip_enabled = true;
      lift::auto_grip_ready = true;
      lift::goal_detect_center_only = false;

      lift::claw.retract();
      add_target(60_in, 0_deg);
      lift_motor.move(-64);
      wait(150);
      lift_motor.tare_position();
      lift_motor.move_absolute(30, 100);
      wait_until_clamp_or_target();
      lift::claw.extend();
      lift::auto_grip_ready = false;
      move_settings.end_output = 20;
      add_target(-60_in, 0_deg);

      lift_motor.move_absolute(360, 100);
      
      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro win_point_3(
    [](){
      auton_init({0_in, 0_in, 0_deg});
      lift::auto_grip_enabled = false;


      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      // add_target(315_deg);
      lift_motor.move_absolute(230, 100);
      wait(400);
      add_target(4_in, 0_deg);
      wait_until_final_target_reached();
      wait(100);
      lift::claw.retract();
      wait(100);
      add_target(-6_in, 0_deg);
      wait(200);
      lift_motor.move_absolute(0, 100);
      wait(300);
      add_target(6_in, 0_deg);
      wait_until_final_target_reached();
      // wait(200);
      lift::claw.extend();
      lift_motor.move_absolute(360, 100);
      wait_until_final_target_reached();
      wait(200);
      add_target(-90_deg);
      add_target(24_in, -90_deg);
      add_target(-180_deg);
      wait_until_final_target_reached();
      move_settings.end_output = 100;
      add_target(-100_in, -180_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      move_settings.start_output = 100;
      move_settings.end_output = 40;
      move_settings.mid_output = 40;
      add_target(-15_in, -180_deg);
      wait_until_final_target_reached(1000);

      lift::tilter.extend();
      wait(500);
      ring_motor.move_velocity(400);
      move_settings.start_output = 20;
      move_settings.mid_output = 20;
      move_settings.end_output = 20;
      move_settings.ramp_down_p = 0.4;
      move_settings.ramp_up_p = 0.4;
      add_target(-270_deg);
      wait_until_final_target_reached();
      add_target(24_in, -270_deg);
      ring_motor.move_velocity(600);
      wait_until_final_target_reached();
      
      move_settings.mid_output = 100;
      wait(11);
      add_target(-24_in, -270_deg);
      wait(700);
      ring_motor.move_velocity(-600);
      wait_until_final_target_reached(3000);

      move_settings.mid_output = 20;
      wait(11);
      ring_motor.move_velocity(600);
      add_target(24_in, -270_deg);
      wait_until_final_target_reached(3000);

      add_target(-6_in, -270_deg);
      wait_until_final_target_reached();
      add_target(-90_deg);
      turn_p = 0.3;
      wait_until_final_target_reached();
      lift::tilter.retract();

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro win_point_2(
    [](){
      auton_init({0_in, 0_in, 0_deg});
      lift::auto_grip_enabled = false;


      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      // add_target(315_deg);
      lift_motor.move_absolute(230, 100);
      wait(400);
      add_target(6_in, 0_deg);
      wait_until_final_target_reached();
      wait(100);
      lift::claw.retract();
      wait(100);
      add_target(-6_in, 0_deg);
      wait(200);
      lift_motor.move_absolute(20, 100);
      wait(300);
      add_target(6_in, 0_deg);
      wait_until_final_target_reached();
      // wait(200);
      lift::claw.extend();
      add_target(7_in, 0_deg);
      lift_motor.move_absolute(360, 100);
      wait_until_final_target_reached();
      wait(200);
      add_target(90_deg);
      add_target(-115_in, 90_deg);
      add_target(180_deg);
      wait_until_final_target_reached();
      add_target(20_in, 180_deg);
      wait_until_final_target_reached(2000);
      move_settings.mid_output = 20;
      add_target(10_in, 180_deg);
      wait_until_final_target_reached(1200);
      add_target(-4_in, 180_deg);
      add_target(270_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      wait(500);
      move_settings.mid_output = 40;
      add_target(-14_in, 270_deg);
      wait_until_final_target_reached(3000);
      wait(100);
      lift::tilter.extend();
      wait_until_final_target_reached(3000);
      move_settings.start_output = 20;
      move_settings.end_output = 20;
      move_settings.mid_output = 20;
      wait(500);
      add_target(14_in, 270_deg);
      ring_motor.move_velocity(400);
      wait(500);
      ring_motor.move_velocity(600);
      wait_until_final_target_reached(2000);
      add_target(-4_in, 270_deg);
      wait_until_final_target_reached();

      lift::tilter.retract();

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

    Macro skills_2(
    [](){
      auton_init({0_in, 0_in, 0_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      turn_settings.mid_output = 30;

      // turn_settings.mid_output = 30;
      lift::claw.retract();
      lift_motor.move_absolute(-10, 100);
      add_target(4_in, 0_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      // lift_motor.move_absolute(90, 100);
      lift_motor.move_absolute(360, 100);
      wait(1000);
      //pick up blue goal one

      add_target(4_in, 0_deg);
      add_target(90_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      add_target(-106_in, 90_deg);
      wait_until_final_target_reached(3500);
      add_target(6_in, 90_deg);
      wait_until_final_target_reached();

      add_target(0_deg);
      // add_target(10_in, 0_deg);
      // wait_until_final_target_reached();
      // lift::tilter.retract();
      // add_target(-30_in, 0_deg);
      add_target(-16_in, 0_deg);
      move_settings.mid_output = 20;
      wait_until_final_target_reached(3000);
      turn_settings.mid_output = 30;
      lift::tilter.extend();
      wait(1000);
      move_settings.mid_output = 100;
      // pick up red goal one

      add_target(44_in, 0_deg);
      add_target(-70_deg);
      // ring_motor.move_velocity(ring_speed);
      // wait(2000);
      // ring_motor.move_velocity(-ring_speed);
      // wait(300);
      // ring_motor.move_velocity(ring_speed);
      // wait(1000);
      // ring_motor.move_velocity(0);
      wait_until_final_target_reached();
      move_settings.mid_output = 20;
      add_target(12_in, -70_deg);
      wait_until_final_target_reached(1000);
      move_settings.mid_output = 100;
      // wait(500);
      lift::claw.retract();
      // drop blue goal one on blue platform

      add_target(-4_in, -90_deg);
      add_target(-5_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      lift_motor.move_absolute(0, 100);
      add_target(47_in, -5_deg);
      wait_until_final_target_reached();
      lift::claw.retract();
      add_target(-132_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      add_target(11_in, -132_deg);
      wait_until_final_target_reached(1200);
      lift::claw.extend();
      wait(500);
      add_target(-10_in, -132_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(360, 100);
      wait(1000);
      // add_target(4_in, -132_deg);
      // pick up red goal two

      add_target(-40_deg);
      add_target(-16_in, -40_deg);
      add_target(-95_deg);
      add_target(-66_in, -95_deg);
      // push yellow goal two


      add_target(-180_deg);
      add_target(18_in, -180_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      wait(500);
      add_target(8_in, -180_deg);
      wait_until_final_target_reached();
      wait(500);
      add_target(-270_deg);
      lift::tilter.extend();
      wait_until_final_target_reached();
      add_target(6_in, -270_deg);
      wait_until_final_target_reached(1000);
      lift::claw.retract();
      add_target(-4_in, -270_deg);
      // drop red goal one on red platform

      add_target(-360_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      wait(800);
      move_settings.mid_output = 50;
      add_target(32_in, -360_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      lift_motor.move_absolute(360, 100);
      add_target(-22_in, -360_deg);
      wait_until_final_target_reached();
      turn_settings.mid_output = 20;
      add_target(-270_deg);
      wait_until_final_target_reached();
      turn_settings.mid_output = 50;
      add_target(6_in, -270_deg);
      wait_until_final_target_reached(2000);
      lift::claw.retract();
      // drop red goal two on the red platform

      add_target(-4_in, -270_deg);
      add_target(-350_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      add_target(46_in, -350_deg);
      wait_until_final_target_reached(4000);
      // move_settings.mid_output = 100;
      lift::claw.extend();
      wait(500);
      add_target(-2_in, -360_deg);
      lift_motor.move_absolute(360, 100);
      // add_target(-345_deg);
      // wait_until_final_target_reached();
      // lift::tilter.retract();
      // add_target(-24_in, -345_deg);
      wait_until_final_target_reached();
      add_target(-320_deg);
      // wait(500);
      add_target(-42_in, -320_deg);
      wait_until_final_target_reached();
      wait(500);
      move_settings.mid_output = 40;
      // lift::tilter.extend();
      wait(1500);
      add_target(-60_in, -325_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 50;
      add_target(48_in, -325_deg);
      add_target(-450_deg);
      wait_until_final_target_reached();
      add_target(53_in, -450_deg);
      wait_until_final_target_reached(3000);
      lift::claw.retract();
      add_target(-6_in, -450_deg);
      add_target(-360_deg);
      // pick up blue goal two

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});


    //=============================================================================================//
    //=============================================================================================//

Macro left_side_rings(
    [](){
      auton_init({0_in, 0_in, 0_deg});
      move_settings.ramp_down_p = 0.4;
      move_settings.ramp_up_p = 0.4;

      move_settings.start_output = 20;
      move_settings.mid_output = 20;
      move_settings.end_output = 15;

      lift_motor.move_absolute(300, 100);
      lift::tilter.retract();
      wait(500);
      add_target(-12_in, 0_deg);
      wait_until_final_target_reached(3000);
      lift::tilter.extend();
      wait(500);
      ring_motor.move_velocity(ring_speed);
      add_target(6_in, 0_deg);
      add_target(-90_deg);
      wait_until_final_target_reached();
      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;
  
      for (size_t i = 0; i < 3; i++) {
        move_settings.mid_output = 100;
        wait(11);
        add_target(-24_in, -90_deg);
        // wait(500);
        // ring_motor.move_velocity(-400);
        wait_until_final_target_reached(3000);

        move_settings.mid_output = 30;
        wait(11);
        ring_motor.move_velocity(400);
        add_target(24_in, -90_deg);
        wait_until_final_target_reached(3000);
      }
      move_settings.start_output = 100;
      move_settings.mid_output = 100;
      move_settings.end_output = 100;
      add_target(-4_in, -90_deg);
      add_target(4_in, -90_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro right_yellow_rings(
    [](){
      auton_init({57_in, 32_in, 0_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      lift::claw.retract();
      lift_motor.move_absolute(-10, 100);

      add_target(43_in, 0_deg);
      wait_until_final_target_reached(1300);
      lift::claw.extend();
      add_target(-24_in, 0_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      add_target(-90_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(360, 100);
      move_settings.mid_output = 40;
      add_target(-14_in, -90_deg);
      wait_until_final_target_reached(1200);
      move_settings.mid_output = 100;
      lift::tilter.extend();
      wait(500);
      // add_target(3_in, -90_deg);
      ring_motor.move_velocity(400);
      move_settings.ramp_down_p = 0.4;
      move_settings.ramp_up_p = 0.4;
      add_target(0_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 30;
      ring_motor.move_velocity(600);
      add_target(28_in, 0_deg);
    
      wait_until_final_target_reached();
      add_target(-50_in, 0_deg);
      move_settings.mid_output = 50;
      wait_until_final_target_reached(3000);
      add_target(4_in, 0_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro kickstand_right_yellow_rings(
    [](){
      auton_init({57_in, 32_in, 0_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      lift::claw.retract();
      lift_motor.move_absolute(-10, 100);

      add_target(43_in-kickstand_offset_distance, 0_deg-kickstand_offset_angle);
      wait_until_final_target_reached(1300);
      lift::claw.extend();
      add_target(-24_in, 0_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      add_target(-90_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(360, 100);
      move_settings.mid_output = 40;
      add_target(-14_in, -90_deg);
      wait_until_final_target_reached(1200);
      move_settings.mid_output = 100;
      lift::tilter.extend();
      wait(500);
      // add_target(3_in, -90_deg);
      ring_motor.move_velocity(400);
      move_settings.ramp_down_p = 0.4;
      move_settings.ramp_up_p = 0.4;
      add_target(0_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 30;
      ring_motor.move_velocity(600);
      add_target(28_in, 0_deg);
    
      wait_until_final_target_reached();
      add_target(-50_in, 0_deg);
      move_settings.mid_output = 50;
      wait_until_final_target_reached(3000);
      add_target(4_in, 0_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro win_point_1(
    [](){
      auton_init({57_in, 32_in, -45_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      // add_target(315_deg);
      lift_motor.move_absolute(230, 100);
      wait(700);
      add_target(17_in, -45_deg);
      wait_until_final_target_reached();
      wait(500);
      lift::claw.retract();
      wait(200);
      add_target(-6_in, -45_deg);
      wait(500);
      lift_motor.move_absolute(10, 100);
      wait(300);
      add_target(6_in, -45_deg);
      wait_until_final_target_reached();
      wait(500);
      lift::claw.extend();
      wait(200);
      add_target(-3_in, -45_deg);
      lift_motor.move_absolute(50, 100);
      add_target(30_deg);
      add_target(-18_in, 30_deg);
      add_target(0_deg);
      add_target(-76_in, 0_deg);
      add_target(-100_deg);
      wait_until_final_target_reached();
      // move_settings.mid_output = 50;
      move_settings.mid_output = 40;
      add_target(-122_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      // wait(500);
      add_target(-20_in, -122_deg);

      wait_until_final_target_reached(1000);
      wait(500);
      lift::tilter.extend();
      wait(500);
      ring_motor.move_velocity(ring_speed);
      wait(1500);
      // add_target(6_in, -122_deg);
      add_target(-90_deg);
      // ring_motor.move_velocity(-ring_speed);
      // wait(300);
      // ring_motor.move_velocity(ring_speed);
      // wait(500);
      // ring_motor.move_velocity(0);
      // move_settings.start_output = 100;
      // move_settings.mid_output = 100;
      // move_settings.end_output = 100;
      // add_target(-4_in, -90_deg);
      // add_target(4_in, -90_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro point_and_plus_fast(
    [](){
      auton_init({57_in, 32_in, 10_deg});

      turn_settings.start_output = 20;
      turn_settings.mid_output = 50;
      turn_settings.end_output = 20;

      move_settings.start_output = 100;
      move_settings.mid_output = 100;
      move_settings.end_output = 100;
      move_settings.ramp_down_p = 0.09;

      lift::claw.retract();
      lift_motor.move_absolute(-10, 100);

      add_target(46_in, 10_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      move_settings.ramp_down_p = 0.2;
      move_settings.end_output = 20;
      add_target(-26.5_in, 10_deg);
      wait(500);
      lift_motor.move_absolute(90, 100);
      wait_until_final_target_reached();
      move_settings.start_output = 20;
      add_target(30_deg);
      add_target(-11_in, 30_deg);
      turn_settings.mid_output = 30;
      add_target(-53_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 20;
      move_settings.end_output = 15;
      lift::tilter.retract();
      wait(500);
      add_target(-12_in, -53_deg);
      wait_until_final_target_reached();
      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;
      wait(500);
      lift::tilter.extend();
      wait(1000);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(-ring_speed);
      wait(300);
      ring_motor.move_velocity(ring_speed);
      wait(1000);
      ring_motor.move_velocity(0);
      add_target(6_in, -45_deg);
      add_target(45_deg);
      wait_until_final_target_reached();
      add_target(-20_in, 45_deg);
      wait_until_final_target_reached(1000);
      add_target(3_in, 45_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
    },
    auton_clean_up,
    {&auton_group});

Macro one_side(
    [](){
      auton_init({57_in, 32_in, -45_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      lift_motor.move_absolute(230, 100);
      wait(700);
      add_target(17_in, -45_deg);
      wait_until_final_target_reached();
      wait(1000);
      lift::claw.retract();
      wait(200);
      add_target(-6_in, -45_deg);
      wait(500);
      lift_motor.move_absolute(10, 100);
      wait(1000);
      add_target(8_in, -45_deg);
      wait_until_final_target_reached();
      wait(500);
      lift::claw.extend();
      wait(500);
      lift_motor.move_absolute(50, 100);
      add_target(-8_in, -45_deg);
      ring_motor.move_velocity(0);

      wait_until_final_target_reached();

    },
    auton_clean_up,
    {&auton_group});

Macro point_and_shoot(
    [](){
      auton_init({57_in, 32_in, 0_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      lift::claw.retract();
      lift_motor.move_absolute(10, 100);
      add_target(44_in, 0_deg);
      wait_until_final_target_reached();
      wait(200);
      lift::claw.extend();
      add_target(-44_in, 0_deg);
      wait_until_final_target_reached();

    },
    auton_clean_up,
    {&auton_group});

Macro point_and_plus(
    [](){
      auton_init({57_in, 32_in, 10_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      lift::claw.retract();
      lift_motor.move_absolute(10, 100);

      add_target(43_in, 10_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      lift_motor.move_absolute(10, 100);
      add_target(-23.5_in, 10_deg);
      add_target(45_deg);
      add_target(-12_in, 45_deg);
      add_target(-45_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 20;
      move_settings.end_output = 15;
      lift::tilter.retract();
      wait(500);
      add_target(-12_in, -45_deg);
      // add_target(-20_in, -122_deg);

      // wait_until_final_target_reached(1000);
      wait_until_goal(3000);
      wait(500);
      lift::tilter.extend();
      wait(1000);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(-ring_speed);
      wait(300);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(0);
      add_target(6_in, -45_deg);
      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro point_and_plus_old(
    [](){
      auton_init({57_in, 32_in, 10_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      lift::claw.retract();
      lift_motor.move_absolute(10, 100);

      add_target(43_in, 10_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      lift_motor.move_absolute(10, 100);
      add_target(-23.5_in, 10_deg);
      add_target(45_deg);
      add_target(-12_in, 45_deg);
      add_target(-45_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 20;
      move_settings.end_output = 15;
      lift::tilter.retract();
      wait(500);
      add_target(-14_in, -45_deg);
      wait_until_final_target_reached(3000);
      wait(500);
      lift::tilter.extend();
      wait(1000);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(-ring_speed);
      wait(300);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(0);
      add_target(6_in, -45_deg);
      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro point_and_plus_4(
    [](){
      auton_init({57_in, 32_in, 10_deg});

      move_settings.start_output = 100;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      lift::claw.retract();
      lift_motor.move_absolute(10, 100);

      add_target(43_in, 10_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      lift_motor.move_absolute(10, 100);
      move_settings.start_output = 20;
      move_settings.ramp_down_p = 0.2;
      add_target(-23.5_in, 10_deg);
      add_target(45_deg);
      add_target(-12_in, 45_deg);
      add_target(-45_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 20;
      move_settings.end_output = 15;
      lift::tilter.retract();
      wait(500);
      add_target(-14_in, -45_deg);
      wait_until_final_target_reached(3000);
      wait(500);
      lift::tilter.extend();
      wait(1000);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(-ring_speed);
      wait(300);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(0);
      add_target(6_in, -45_deg);
      add_target(0_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

Macro skills_1(
    [](){
      auton_init({0_in, 0_in, 0_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      // turn_settings.mid_output = 30;
      lift::claw.retract();
      lift_motor.move_absolute(-10, 100);
      add_target(4_in, 0_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      // lift_motor.move_absolute(90, 100);
      lift_motor.move_absolute(360, 100);
      wait(1000);
      //pick up blue goal one

      // add_target(-7_in, 0_deg);
      add_target(-90_deg);
      add_target(24_in, -90_deg);
      add_target(90_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      add_target(-78_in, 90_deg);
      add_target(10_in, 90_deg);
      wait_until_final_target_reached();
      // lift::tilter.extend();
      // push yellow goal one

      add_target(0_deg);
      // add_target(10_in, 0_deg);
      // wait_until_final_target_reached();
      // lift::tilter.retract();
      // add_target(-30_in, 0_deg);
      add_target(-16_in, 0_deg);
      move_settings.mid_output = 20;
      wait_until_final_target_reached(3000);
      lift::tilter.extend();
      wait(1000);
      move_settings.mid_output = 100;
      // pick up red goal one

      add_target(44_in, 0_deg);
      add_target(-70_deg);
      ring_motor.move_velocity(ring_speed);
      wait(2000);
      ring_motor.move_velocity(-ring_speed);
      wait(300);
      ring_motor.move_velocity(ring_speed);
      wait(1000);
      ring_motor.move_velocity(0);
      wait_until_final_target_reached();
      move_settings.mid_output = 20;
      add_target(12_in, -70_deg);
      wait_until_final_target_reached(1000);
      move_settings.mid_output = 100;
      // wait(500);
      lift::claw.retract();
      // drop blue goal one on blue platform

      add_target(-4_in, -90_deg);
      add_target(-5_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(0, 100);
      add_target(47_in, -5_deg);
      add_target(-132_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      add_target(9_in, -132_deg);
      wait_until_final_target_reached(1000);
      lift::claw.extend();
      add_target(-8_in, -132_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(360, 100);
      wait(1000);
      // add_target(4_in, -132_deg);
      // pick up red goal two

      add_target(-40_deg);
      add_target(-16_in, -40_deg);
      add_target(-95_deg);
      add_target(-66_in, -95_deg);
      // push yellow goal two


      add_target(-180_deg);
      add_target(18_in, -180_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      wait(500);
      add_target(8_in, -180_deg);
      wait_until_final_target_reached();
      wait(500);
      add_target(-270_deg);
      lift::tilter.extend();
      wait_until_final_target_reached();
      add_target(6_in, -270_deg);
      wait_until_final_target_reached(1000);
      lift::claw.retract();
      add_target(-4_in, -270_deg);
      // drop red goal one on red platform

      add_target(-360_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      wait(800);
      move_settings.mid_output = 50;
      add_target(32_in, -360_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      lift_motor.move_absolute(360, 100);
      add_target(-22_in, -360_deg);
      wait_until_final_target_reached();
      turn_settings.mid_output = 20;
      add_target(-270_deg);
      wait_until_final_target_reached();
      turn_settings.mid_output = 50;
      add_target(6_in, -270_deg);
      wait_until_final_target_reached(2000);
      lift::claw.retract();
      // drop red goal two on the red platform

      add_target(-4_in, -270_deg);
      add_target(-360_deg);
      wait_until_final_target_reached();
      lift_motor.move_absolute(-10, 100);
      add_target(42_in, -360_deg);
      wait_until_final_target_reached(4000);
      // move_settings.mid_output = 100;
      lift::claw.extend();
      lift_motor.move_absolute(360, 100);
      add_target(-345_deg);
      wait_until_final_target_reached();
      lift::tilter.retract();
      add_target(-24_in, -345_deg);
      wait_until_final_target_reached();
      add_target(-320_deg);
      // wait(500);
      add_target(-18_in, -320_deg);
      wait_until_final_target_reached();
      wait(500);
      move_settings.mid_output = 50;
      lift::tilter.extend();
      wait(1500);
      add_target(-12_in, -320_deg);
      add_target(-450_deg);
      wait_until_final_target_reached();
      add_target(36_in, -450_deg);
      wait_until_final_target_reached(5000);
      lift::claw.retract();
      add_target(-6_in, -450_deg);
      add_target(-360_deg);
      // pick up blue goal two

      wait_until_final_target_reached();
    },
    auton_clean_up,
    {&auton_group});

    Macro keyan_skills(
    [](){
      auton_init({0_in, 0_in, 0_deg});

      move_settings.start_output = 20;
      move_settings.mid_output = 100;
      move_settings.end_output = 20;

      turn_settings.mid_output = 30;

      // turn_settings.mid_output = 30;
      lift::claw.retract();
      lift::tilter.retract();
      lift_motor.move_absolute(-10, 100);
      wait(300);

      // Pick up First goal
      add_target(-6_in, 0_deg);
      wait_until_final_target_reached();
      lift::tilter.extend();
      wait(300);
      //end

      // Pick up 2nd goal
      add_target(100_deg);
      wait_until_final_target_reached();
      move_settings.mid_output = 80;
      add_target(42_in, 100_deg);
      wait_until_final_target_reached();
      lift::claw.extend();
      wait(300);
      //end

      move_settings.mid_output = 100;

      //put yellow goal 1 on platform
      add_target(145_deg);
      wait_until_final_target_reached();
      add_target(36_in, 145_deg);
      wait_until_final_target_reached();
      add_target(90_deg);
      wait_until_final_target_reached();
      add_target(24_in, 90_deg);
      wait_until_final_target_reached();
      //end

    },
    auton_clean_up,
    {&auton_group});


} // namespace autonroutines