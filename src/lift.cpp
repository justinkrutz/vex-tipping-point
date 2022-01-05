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
  print();
}

void Piston::retract() {
  piston.set_value(false);
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
        motor = -127;
        is_moving = true;
        was_forward = true;
      } else {
        motor = 127;
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

void set_callbacks() {
  using namespace controllerbuttons;
  // button_handler.master.r1.pressed.set(toggle_grabber);
  button_handler.master.r1.pressed .set([&](){ claw.toggle(); });
  button_handler.master.r2.pressed .set([&](){ tilter.toggle(); });
  // button_handler.master.r2.pressed.set_macro();

  button_handler.master.up  .pressed .set([&](){ intake.toggle(); });//ring_motor = 127;
//  button_handler.master.up  .released.set([&](){ lift.toggle(true); });//ring_motor = 0;
  button_handler.master.down.pressed .set([&](){ intake.toggle(true); });//ring_motor = -127;
//  button_handler.master.down.released.set([&](){ lift.toggle(); });//ring_motor = 0;

  button_handler.master.l1.pressed .set([&](){ lift_motor = 127; });
  button_handler.master.l1.released.set([&](){ lift_motor = 0; });
  button_handler.master.l2.pressed .set([&](){ lift_motor = -127; });
  button_handler.master.l2.released.set([&](){ lift_motor = 0; });
}

} // namespace lift