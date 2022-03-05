#include "main.h"
#include "robot-config.h"
#include "robot-functions.h"
#include "odom-utilities.h"
#include "auton-drive.h"
#include "controller-menu.h"
#include "odometry.h"


// #include <bits/stdc++.h>
// // #include "json.hpp"
// // using json = nlohmann::ordered_json;

// #define DEFAULT_GOAL_OFFSET 13_in
// #define DEFAULT_BALL_OFFSET 10_in
#define GPS_X_RATE 0.1
#define GPS_Y_RATE 0.1
#define GPS_THETA_RATE 1
#define GPS_ERROR_THRESHOLD 0.05

namespace odomutilities {


// Goal::Goal(Point point, std::vector<QAngle> angles, GoalType goal_type) : point(point), angles(angles), offset(DEFAULT_GOAL_OFFSET), goal_type(goal_type) {
//   goals.push_back(this);
// }

// Goal::Goal(Point point, std::vector<QAngle> angles, QLength offset, GoalType goal_type) : point(point), angles(angles), offset(offset), goal_type(goal_type) {
//   goals.push_back(this);
// }

// std::vector<Goal*> Goal::goals = {};

// Goal *Goal::closest(Point current_point) {
//   Goal *closest_goal = goals[0];
//   for (auto &&goal : goals) {
//     if (OdomMath::computeDistanceToPoint(current_point, {goal->point.x, goal->point.y}) <
//         OdomMath::computeDistanceToPoint(current_point, {closest_goal->point.x, closest_goal->point.y})) {
//       closest_goal = goal;
//     }
//   }
//   return closest_goal;
// }

// Point goal_1 = {40.9_in,  11.4_in};
// Point goal_2 = {129.2_in, 35_in};
// Point goal_3 = {35_in,    70.3_in};
// Point goal_4 = {70.3_in,  70.3_in};
// Point goal_5 = {105.7_in, 70.3_in};
// Point goal_6 = {11.4_in,  105.7_in};
// Point goal_7 = {99.8_in,  129.2_in};
Point goal_1 = {40.9_in,  129.2_in};
Point goal_2 = {129.2_in, 105.7_in};
Point goal_3 = {35_in,    70.3_in};
Point goal_4 = {70.3_in,  70.3_in};
Point goal_5 = {105.7_in, 70.3_in};
Point goal_6 = {11.4_in,  35_in};
Point goal_7 = {99.8_in,  11.4_in};


// /*        MATCH SETUP
//    │                       │
//    └───────────────────────┘
// ┌─────────────────────────────┐
// │                             │
// │──────────────╩──────────────│
// │                             │
// │                             │
// │                             │
// │4═════d═════e═5═f═════g═════6│
// │              c              │
// │                             │
// │                             │
// │──a───────────╦───────────b──│
// │1             2             3│
// └─────────────────────────────┘
//    ┌───────────────────────┐
//    │                       │
// */

// namespace matchballs {
//   Point ball_a { 12.0304_in,  12.0304_in};
//   Point ball_b { 12.0304_in, 128.6418_in};
//   Point ball_c { 61.5432_in,  70.3361_in};
//   Point ball_d { 70.3361_in,  34.9916_in};
//   Point ball_e { 70.3361_in,  61.5432_in};
//   Point ball_f { 70.3361_in,  79.1290_in};
//   Point ball_g { 70.3361_in, 105.6806_in};
// }

// /*       SKILLS SETUP
//    │                       │
//    └───────────────────────┘
// ┌──────────────╦──────────────┐
// │7             8             9│
// │──────m───────╩───────n──────│
// │k                           l│
// │              j              │
// │                             │
// │4═══f════g════5════h════i═══6│
// │                             │
// │              e              │
// │c                           d│
// │──────a───────╦───────b──────│
// │1             2             3│
// └──────────────╩──────────────┘
//    ┌───────────────────────┐
//    │                       │
// */

// namespace skillsballs {
//   Point ball_a { 22.8361_in,  34.9911_in};
//   Point ball_b { 22.8361_in, 105.8361_in};
//   Point ball_c { 34.9911_in,   3.3361_in};
//   Point ball_d { 34.9911_in, 137.3361_in};
//   Point ball_e { 46.8361_in,  70.3361_in};
//   Point ball_f { 70.3361_in,  23.3361_in};
//   Point ball_g { 70.3361_in,  46.8361_in};
//   Point ball_h { 70.3361_in,  93.8361_in};
//   Point ball_i { 70.3361_in, 117.3361_in};
//   Point ball_j { 93.8361_in,  70.3361_in};
//   Point ball_k {105.8361_in,   3.3361_in};
//   Point ball_l {105.8361_in, 137.3361_in};
//   Point ball_m {117.6361_in,  34.9911_in};
//   Point ball_n {117.6361_in, 105.6361_in};
// }

namespace errorcorrection {

using namespace odomutilities;

// Goal *last_goal = &goal_9;
// Point last_odom_point;

// bool first_goal_reached = false;

// ObjectSensor goal_os ({&goal_sensor_one, &goal_sensor_two}, 2600, 2750);

// bool waiting = false;
// int time_triggered = 0;
// const int kWaitTime = 100;
// const QLength kGoalOffset = 12.2274_in;
// const QLength kDetectionDistance = 15_in;

bool gps_allign = true;

void loop() {
  gps.set_data_rate(5);
  Slew gps_x_slew(GPS_X_RATE);
  Slew gps_y_slew(GPS_Y_RATE);
  Slew gps_theta_slew(GPS_THETA_RATE);

  while (true) {
    pros::delay(5);
    if (!gps_allign) continue;

    double headings [2] = {0, 180};
    for (auto && offset_deg : headings) {
      auto odom = chassis->getState();
      auto gps_pos = gps.get_status();
      double gps_offset_yaw = (int((gps_pos.yaw + offset_deg)*1000) % 360000)/1000.0;
      controllermenu::partner_print_array[int(offset_deg)/180] = std::to_string(gps_offset_yaw);
      QAngle gps_theta = gps_theta_slew.new_value(gps_offset_yaw) * degree;
      QLength gps_x = gps_x_slew.new_value(gps_pos.x) * inch;
      QLength gps_y = gps_y_slew.new_value(gps_pos.y) * inch;
      if (gps.get_error() < GPS_ERROR_THRESHOLD && (gps_x - odom.x).abs() < 30_in
          && (gps_y - odom.y).abs() < 30_in
          && (gps_theta - odom.theta).abs() < 20_deg) {
        chassis->setState({gps_x, gps_y, gps_theta});
        imu_odom->setState({gps_x, gps_y, gps_theta});
      }
    }
  }
}

void start() {
  pros::Task task(loop);
}

} // namespace errorcorrection

} // namespace odomutilities