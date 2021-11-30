#include "main.h"

    bool me = false, joy_lift = false;
    int lift_position[] = {0, 90, 360};
    int lift_position_index = 0, piston_cycles = 0;
    bool auto_lift = false, piston_out, piston2_out;
    lift.set_brake_mode(MOTOR_BRAKE_HOLD);
    lift_sensor.set_reversed(true);

    while (true) {
        if (!master.get_digital(DIGITAL_X) && !master.get_digital(DIGITAL_B)) {
            ring_motor = master.get_analog(ANALOG_LEFT_Y);
        } else if (master.get_digital(DIGITAL_X)) {
            ring_motor = 127;
        } else if (master.get_digital(DIGITAL_B)) {
            ring_motor = -127;
        }
        if (!auto_lift) {
            if (master.get_digital(DIGITAL_L1)) {
                lift = 127;
            } else if (master.get_digital(DIGITAL_L2)) {
                lift = -127;
            } else {
                lift = 0;
                lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            }
        }
        if (auto_lift) {
            if (master.get_digital_new_press(DIGITAL_L1)) {
                // if (lift_position_index < sizeof(lift_position) - 1) {
                if (lift_position_index < 2) {
                    lift_position_index++;
                    lift.move_absolute(lift_position[lift_position_index], 127);
                }
            }
            if (master.get_digital_new_press(DIGITAL_L2)) {
                if (lift_position_index > 0) {
                    lift_position_index--;
                    lift.move_absolute(lift_position[lift_position_index], 127);
                }
            }
        }

        if (joy_lift && !master.get_digital(DIGITAL_L1) && !master.get_digital(DIGITAL_L1)) {
            lift = master.get_analog(ANALOG_LEFT_Y);
        }

        if (master.get_digital_new_press(DIGITAL_DOWN)) {
            auto_lift = !auto_lift;
        }

        if (master.get_digital_new_press(DIGITAL_R1)) {
            if (!piston_out) {
                lift_gripper.set_value(1);
                piston_out = true;
                piston_cycles++;
                master.clear_line(0);
                delay(50);
                master.print(0, 0, "%d", piston_cycles);
            } else {
                lift_gripper.set_value(0);
                piston_out = false;
            }
        }

// keyan config

// Motor left_front(5, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
// Motor left_middle(6, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
// Motor left_back(7, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
// Motor right_front(8, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
// Motor right_middle(9, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
// Motor right_back(10, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
// Motor lift(4, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
// Motor ring_motor(11, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);