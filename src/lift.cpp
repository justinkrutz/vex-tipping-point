#include "lift.h"
#include "main.h"
#include "robot-config.h"
#include "controller-menu.h"
#include "controller-buttons.h"

namespace lift {

bool me = false, joy_lift = false;
int lift_position[] = {0, 90, 360};
int lift_position_index = 0, piston_cycles = 0;
bool auto_lift = false, piston_out, piston2_out;

void init() {

  lift_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  lift_sensor.set_reversed(true);
}

Piston::Piston(pros::ADIDigitalOut& piston, bool is_double) : piston(piston), is_double(is_double) {}
  
void Piston::extend() {
  piston.set_value(true);
  piston_cycles++;
  piston_out = true;
  print();
}

void Piston::retract() {
  piston.set_value(false);
  piston_out = false;
  if (is_double) {
    piston_cycles++;
  }
  print();
}

void Piston::toggle() {
  if (is_double) {
    piston_cycles++;
  } else if (!piston_out) {
    piston_cycles++;
  }
  piston_out = !piston_out;
  piston.set_value(piston_out);
  print();
}

void Piston::print() {
  controllermenu::master_print_array[0] = std::to_string(piston_cycles);
}

int Piston::piston_cycles = 0;

class MotorToggle {
 public:
  MotorToggle(pros::Motor& motor) : motor(motor) {}

  void toggle(bool is_forward = false) {
    if (is_moving && was_forward == is_forward) {
      motor = 0;
      is_moving = false;
    } else {
      if (is_forward) {
        motor.move_velocity(-400);
        is_moving = true;
        was_forward = true;
      } else {
        motor.move_velocity(400);
        is_moving = true;
        was_forward = false;
      }
    }
  }

  pros::Motor& motor;
  bool is_moving = false, was_forward = false;
};


Piston claw(lift_gripper);
Piston tilter(back_tilter);

MotorToggle intake(ring_motor);

bool auto_grip_enabled = true;
bool auto_grip_ready = true;
bool goal_auto_gripped = false;

void task_function() {
  while (true) {
    if (auto_grip_enabled && auto_grip_ready && goal_sensor.get_new_press() && !claw.piston_out) {
      claw.extend();
      goal_auto_gripped = true;
    }
    pros::delay(1);
  }
}

void set_callbacks() {
  using namespace controllerbuttons;
  // button_handler.master.r1.pressed.set(toggle_grabber);
  // button_handler.master. x.pressed .set([&](){ auto_grip_enabled = true; });
  // button_handler.master. b.pressed .set([&](){ auto_grip_enabled = false; });
  // button_handler.master.r1.pressed .set([&](){ claw.toggle(); });
  // button_handler.master.r2.pressed .set([&](){ tilter.toggle(); });
  button_handler.master. x.pressed .set([&](){ auto_grip_enabled = true; });
  button_handler.master. b.pressed .set([&](){ auto_grip_enabled = false; });
  button_handler.master.r1.released.set([&](){ if(goal_auto_gripped){claw.extend(); goal_auto_gripped = false;} else { claw.toggle();} auto_grip_ready = false; });
  button_handler.master.r1.pressed .set([&](){ auto_grip_ready = true; });
  button_handler.master.r2.pressed .set([&](){ tilter.toggle(); });
  // button_handler.master.r2.pressed.set_macro();

  button_handler.master.up  .pressed .set([&](){ intake.toggle(); });//ring_motor = 127;
//  button_handler.master.up  .released.set([&](){ lift.toggle(true); });//ring_motor = 0;
 button_handler.master.down.pressed .set([&](){ intake.toggle(true); });//ring_motor = -127;
//  button_handler.master.down.released.set([&](){ lift.toggle(); });//ring_motor = 0;
  pros::Task task(task_function);

  button_handler.master.l1.pressed .set([&](){ lift_motor = 127; });
  button_handler.master.l1.released.set([&](){ lift_motor = 0; });
  button_handler.master.l2.pressed .set([&](){ lift_motor = -127; });
  button_handler.master.l2.released.set([&](){ lift_motor = 0; });
  button_handler.master.left.pressed.set([&](){ lift_motor.move_absolute(200, 40); }); // set down tall goal on end of platform
}

} // namespace lift