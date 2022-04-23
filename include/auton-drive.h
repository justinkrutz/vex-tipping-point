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

enum DriveState {
  kDisabled,
  kOkapi,
  kLegacy
};

extern DriveState drive_state;

extern controllerbuttons::MacroGroup auton_group;
extern controllerbuttons::MacroGroup drive_group;

namespace drivetoposition {

class Target {
 public:
  Target(QLength x, QLength y, QAngle theta, bool hold = true, bool is_move = true, bool is_turn = true);

  QLength x = 0_in;
  QLength y = 0_in;
  QAngle theta = 0_deg;
  OdomState starting_state;
  bool hold;

  // bool is_legacy;
  bool is_move;
  bool is_turn;
  bool is_forward;
  // int dir_multiplier;

  int millis_at_start;
  int timeout;

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
extern controllerbuttons::Macro right_sweep;
extern controllerbuttons::Macro skills_3;
extern controllerbuttons::Macro legacy_test;
extern controllerbuttons::Macro okapi_test;
extern controllerbuttons::Macro profiling_test;

} // namespace autonroutines

#endif // AUTON_DRIVE_H