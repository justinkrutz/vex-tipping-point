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


void motor_task();
void set_callbacks();
} // namespace autondrive


namespace autonroutines {

extern controllerbuttons::Macro none;
extern controllerbuttons::Macro test;
extern controllerbuttons::Macro left_side_rings;
extern controllerbuttons::Macro blue_wp;
extern controllerbuttons::Macro one_side;
extern controllerbuttons::Macro point_and_shoot;
extern controllerbuttons::Macro point_and_plus;
extern controllerbuttons::Macro point_and_plus_old;
extern controllerbuttons::Macro right_side_two;
extern controllerbuttons::Macro point_and_plus_fast;
extern controllerbuttons::Macro skills;
extern controllerbuttons::Macro skills2;
extern controllerbuttons::Macro point_and_plus_4;
extern controllerbuttons::Macro shawnton_4_0;
extern controllerbuttons::Macro right_yellow_win_point;
extern controllerbuttons::Macro keyan_skills;

} // namespace autonroutines

#endif // AUTON_DRIVE_H