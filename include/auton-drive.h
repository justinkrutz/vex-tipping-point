#ifndef AUTON_DRIVE_H
#define AUTON_DRIVE_H

#include "controller-buttons.h"

#define WAIT_UNTIL(condition) \
while (!(condition)) {        \
  controllerbuttons::wait(5);             \
}

#define WAIT_UNTIL_T(condition, timeout) \
int wait_until_timeout_start = pros::millis(); \
while (!(condition) && pros::millis() - wait_until_timeout_start < timeout) { \
  controllerbuttons::wait(5); \
}

namespace autondrive {
extern controllerbuttons::MacroGroup auton_group;
extern controllerbuttons::MacroGroup drive_group;
extern bool stick_control_enabled;

namespace drivetoposition {

class Target {
 public:
  Target(QLength x, QLength y, QAngle theta, bool hold = true, bool is_turn = true);

  QLength x = 0_in;
  QLength y = 0_in;
  QAngle theta = 0_deg;
  OdomState starting_state;
  bool hold;
  bool is_turn;
  int millis_at_start;

  bool is_new = true;
  void init_if_new();
};

// void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance, QAngle offset_angle);
// void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance);
// void add_target(QLength x, QLength y, QAngle theta);
// extern bool targets_should_clear;
// extern bool final_target_reached;
extern controllerbuttons::Macro goal_center;
} // namespace drivetoposition


void reset_button_task();
void motor_task();
void set_callbacks();
} // namespace autondrive


namespace autonroutines {

extern controllerbuttons::Macro win_point_2;
extern controllerbuttons::Macro win_point_3;
extern controllerbuttons::Macro left_yellow_ml;
extern controllerbuttons::Macro right_yellow_ml;
extern controllerbuttons::Macro right_center_pull;
extern controllerbuttons::Macro right_center_fast;
extern controllerbuttons::Macro right_center_shoot;
extern controllerbuttons::Macro center_right_fast;
extern controllerbuttons::Macro left_center;

extern controllerbuttons::Macro kickstand_left_yellow_ml;
extern controllerbuttons::Macro kickstand_right_yellow_ml;
extern controllerbuttons::Macro kickstand_right_yellow_rings;
extern controllerbuttons::Macro kickstand_right_center_pull;
extern controllerbuttons::Macro kickstand_right_center_fast;
extern controllerbuttons::Macro kickstand_right_center_shoot;
extern controllerbuttons::Macro kickstand_center_right_fast;
extern controllerbuttons::Macro kickstand_left_center;

extern controllerbuttons::Macro skills_2;
extern controllerbuttons::Macro extra_far;

extern controllerbuttons::Macro left_side_rings;
extern controllerbuttons::Macro right_yellow_rings;
extern controllerbuttons::Macro none;
extern controllerbuttons::Macro test;
extern controllerbuttons::Macro win_point_1;
extern controllerbuttons::Macro one_side;
extern controllerbuttons::Macro point_and_shoot;
extern controllerbuttons::Macro point_and_plus;
extern controllerbuttons::Macro point_and_plus_old;
extern controllerbuttons::Macro point_and_plus_fast;
extern controllerbuttons::Macro skills_1;
extern controllerbuttons::Macro point_and_plus_4;
extern controllerbuttons::Macro keyan_skills;

} // namespace autonroutines

#endif // AUTON_DRIVE_H