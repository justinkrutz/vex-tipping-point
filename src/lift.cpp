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

  // while (true) {
    // if (!master.get_digital(DIGITAL_X) && !master.get_digital(DIGITAL_B)) {
    //   ring_motor = master.get_analog(ANALOG_LEFT_Y);
    // } else if (master.get_digital(DIGITAL_X)) {
    //   ring_motor = 127;
    // } else if (master.get_digital(DIGITAL_B)) {
    //   ring_motor = -127;
    // }
    // if (!auto_lift) {
    //   if (master.get_digital(DIGITAL_L1)) {
    //     lift_motor = 127;
    //   } else if (master.get_digital(DIGITAL_L2)) {
    //     lift_motor = -127;
    //   } else {
    //     lift_motor = 0;
    //     lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //   }
    // }
    // if (auto_lift) {
    //   if (master.get_digital_new_press(DIGITAL_L1)) {
    //     // if (lift_position_index < sizeof(lift_position) - 1) {
    //     if (lift_position_index < 2) {
    //       lift_position_index++;
    //       lift_motor.move_absolute(lift_position[lift_position_index], 127);
    //     }
    //   }
    //   if (master.get_digital_new_press(DIGITAL_L2)) {
    //     if (lift_position_index > 0) {
    //       lift_position_index--;
    //       lift_motor.move_absolute(lift_position[lift_position_index], 127);
    //     }
    //   }
    // }

    // if (joy_lift && !master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L1)) {
    //   lift_motor = master.get_analog(ANALOG_LEFT_Y);
    // }

    // if (master.get_digital_new_press(DIGITAL_DOWN)) {
    //   auto_lift = !auto_lift;
    // }

    // if (master.get_digital_new_press(DIGITAL_R1)) {
    //   if (!piston_out) {
    //     lift_gripper.set_value(1);
    //     piston_out = true;
    //     piston_cycles++;
    //     // master.clear_line(0);
    //     // pros::delay(50);
    //     // master.print(0, 0, "%d", piston_cycles);
    //     controllermenu::master_print_array[0] = std::to_string(piston_cycles);

    //   } else {
    //     lift_gripper.set_value(0);
    //     piston_out = false;
    //   }
    // }
  // }

}

class Piston {
 public:
  Piston(pros::ADIDigitalOut& piston, bool is_double = false) : piston(piston), is_double(is_double) {}
  static int piston_cycles;
  void toggle() {
    if (is_double) {
      piston_cycles++;
    } else if (!piston_out) {
      piston_cycles++;
    }
    piston_out = !piston_out;
    piston.set_value(piston_out);
    print();
  }
  void print() {
    controllermenu::master_print_array[0] = std::to_string(piston_cycles);
  }
  bool piston_out = false;
  bool is_double;
  pros::ADIDigitalOut& piston;
};
int Piston::piston_cycles = 0;

void toggle_grabber() {
  if (!piston_out) {
    lift_gripper.set_value(1);
    piston_out = true;
    piston_cycles++;
    // master.clear_line(0);
    // pros::delay(50);
    // master.print(0, 0, "%d", piston_cycles);
    controllermenu::master_print_array[0] = std::to_string(piston_cycles);

  } else {
    lift_gripper.set_value(0);
    piston_out = false;
  }
}

class Motor_toggle {
 public:
  Motor_toggle(pros::Motor& motor) : motor(motor) {}

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

Motor_toggle intake(ring_motor);

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