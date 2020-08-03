#include "api.h"

#include <bits/stdc++.h>

#include "controller-buttons.h"
#include "robot-config.h"


namespace controllerbuttons {

class MacroHandler {
  private:
  static pros::Mutex mutex;

  static void task_start_wrapper(void * void_ptr) {
    Macro * macro_ptr = (Macro*) void_ptr;
    for(auto &group : macro_ptr->macro_groups) {
      group->is_running_ptr = &macro_ptr->is_running;
      group->group_task_t = &macro_ptr->button_task_t;
    }

    macro_ptr->is_running = true;
    macro_ptr->function();
    macro_ptr->is_running = false;
    mutex.take(TIMEOUT_MAX);
    mutex.give();
    printf("ended\r\n");
  }

  public:

  struct MacroGroup {
    bool * is_running_ptr = &unnasigned_group;
    pros::task_t * group_task_t;
  };

  class Macro {
    public:

    pros::task_t button_task_t;
    std::function<void()> function;
    std::vector<MacroGroup *> macro_groups;

    void task_start_wrapper() {
      for(auto &group : macro_groups) {
        group->is_running_ptr = &is_running;
        group->group_task_t = &button_task_t;
      }

      is_running = true;
      function();
      is_running = false;
      mutex.take(TIMEOUT_MAX);
      mutex.give();
      printf("ended\r\n");
    }

    void start(Macro macro) {
      // std::function<void()> func(std::bind(&Macro::task_start_wrapper, this));
      // button_task_t = pros::c::task_create(),
      button_task_t = pros::c::task_create(std::bind(&Macro::task_start_wrapper, this),
      // button_task_t = pros::c::task_create([this] { task_start_wrapper(); },
                                           nullptr,
                                           TASK_PRIORITY_DEFAULT,
                                           TASK_STACK_DEPTH_DEFAULT,
                                           "macro");
  }

    bool is_running;
  };

  void start(Macro macro) {
    macro.button_task_t = pros::c::task_create(task_start_wrapper,
                                         (void *)&macro,
                                         TASK_PRIORITY_DEFAULT,
                                         TASK_STACK_DEPTH_DEFAULT,
                                         "macro_handler");
  }

  void interrupt_macro_group(MacroGroup * group) {
    mutex.take(TIMEOUT_MAX);
    printf("is_running_ptr = %d\r\n", *group->is_running_ptr);
    if (*group->is_running_ptr) {
      pros::c::task_delete(*group->group_task_t);
      printf("DELETED!\r\n");
      *group->is_running_ptr = false;
    }
    mutex.give();
  }
};


class ButtonHandler {
  public:
  class Controller {
    public:
    class Button {
      public:
      class Trigger {
        private:
        
        pros::Controller * controller_;
        pros::controller_digital_e_t button_;

        std::function<void()> function_;
        std::vector<std::string> button_groups_;
        bool run_as_task_;
        
        bool was_triggered_ = false;
        // bool is_running;
        bool is_set_ = false;
        bool trigger_on_release_;
        // std::vector<MacroGroup *> macro_groups;

        public:
        Trigger(pros::Controller * controller,
                pros::controller_digital_e_t button,
                bool trigger_on_release) {
          controller_ = controller;
          button_ = button;
          trigger_on_release_ = trigger_on_release;
        }

        void set(std::function<void()> function,
                 std::vector<std::string> button_groups = {"default"},
                 bool run_as_task = false) {
          function_ = function;
          button_groups_ = button_groups;
          run_as_task_ = run_as_task;
          is_set_ = true;
        }

        void clear() {
          is_set_ = false;
        }

        void clear_if_in_group(std::string button_group) {
          for (auto &group_to_clear : button_groups_) {
            if (button_group == group_to_clear){
              clear();
              return;
            }
          }
        }

        void run_if_triggered() {
          if (is_set_) {
            bool is_pressing = controller_->get_digital(button_);
            bool was_pressed  = (is_pressing && !was_triggered_);
            bool was_released = (!is_pressing && was_triggered_);
            bool is_running = false;
            // for (auto &macro_group : macro_groups) {
            //   is_running = is_running || *macro_group->is_running_ptr;
            // }

            // If the button has been pressed and the task isn't running
            if (was_pressed && !is_running) {
              // Set the function to not run when the button is held
              was_triggered_ = true;
              if (trigger_on_release_) return;
            } else if (was_released) {
              was_triggered_ = false;
              if (!trigger_on_release_) return;
            } else {
              return;
            }
            // if (run_as_task) {
            // // Run the function in a separate task
            // button_task_t =
            //     pros::c::task_create(task_start_wrapper, (void *)&button_callback,
            //         TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "run_buttons");
            // } else {
            function_();
            // }
          }
        }
      };

      Button(pros::Controller * controller,
             pros::controller_digital_e_t button) :
             pressed(controller, button, false),
             released(controller, button, true) {}

      Trigger pressed;
      Trigger released;
      std::vector<Controller::Button::Trigger *> triggers{&pressed, &released};
    };
    
    Controller(pros::Controller * controller) :
               l1    (controller, pros::E_CONTROLLER_DIGITAL_L1),
               l2    (controller, pros::E_CONTROLLER_DIGITAL_L2),
               r1    (controller, pros::E_CONTROLLER_DIGITAL_R1),
               r2    (controller, pros::E_CONTROLLER_DIGITAL_R2),
               up    (controller, pros::E_CONTROLLER_DIGITAL_UP),
               down  (controller, pros::E_CONTROLLER_DIGITAL_DOWN),
               left  (controller, pros::E_CONTROLLER_DIGITAL_LEFT),
               right (controller, pros::E_CONTROLLER_DIGITAL_RIGHT),
               x     (controller, pros::E_CONTROLLER_DIGITAL_X),
               b     (controller, pros::E_CONTROLLER_DIGITAL_B),
               y     (controller, pros::E_CONTROLLER_DIGITAL_Y),
               a     (controller, pros::E_CONTROLLER_DIGITAL_A) {}

    Button l1;
    Button l2;
    Button r1;
    Button r2;
    Button up;
    Button down;
    Button left;
    Button right;
    Button x;
    Button b;
    Button y;
    Button a;
    std::vector<Button *> buttons{&l1, &l2, &r1, &r2, &up, &down, &left, &right, &x, &b, &y, &a};
  };


  ButtonHandler(pros::Controller * master_controller,
                pros::Controller * partner_controller) :
                master(master_controller), 
                partner(partner_controller) {
    for (auto &controller : controllers) {
      for (auto &button : controller->buttons) {
        for (auto &trigger : button->triggers) { 
          all_triggers.push_back(trigger);
        }
      }
    }
  }

  // Controller *master;
  Controller master;
  Controller partner;
  std::vector<Controller *> controllers{&master, &partner};

  std::vector<Controller::Button::Trigger *> all_triggers;

  void run() {
    for (auto &trigger : all_triggers) {
      trigger->run_if_triggered();
    }
  }

  void clear_all() {
    for (auto &trigger : all_triggers) {
      trigger->clear();
    }
  }

  void clear_group(std::string button_group) {
    for (auto &trigger : all_triggers) {
      trigger->clear_if_in_group(button_group);
    }
  }
};

ButtonHandler button_handler(&master, &partner);

void foo_one() {
  printf("foo_one\n");
}

void foo_two() {
  printf("foo_two\n");
  set_callbacks();
}

void foo_three() {
  printf("foo_three\n");
  button_handler.clear_group("test");
}

void foo_four() {
  printf("foo_four\n");
  button_handler.master.a.pressed.clear();
  button_handler.master.a.released.set(foo_one, {"test"});
}

void foo_five_start() {
  printf("foo_five_start\n");
}

void foo_five_end() {
  printf("foo_five_end\n");
}

void set_callbacks() {
  button_handler.master.a.pressed.set(foo_one, {"test"});
  button_handler.master.x.pressed.set(foo_two);
  button_handler.master.b.pressed.set(foo_three, {"test"});
  button_handler.master.y.pressed.set(foo_four, {"test"});
  button_handler.master.left.pressed.set(foo_five_start, {"test"});
  button_handler.master.left.released.set(foo_five_end, {"test"});
}

void run_buttons() {
  button_handler.run();
}





















bool unnasigned_group;

pros::Mutex mutex;

// Stores what buttons should run which functions
// Is writen to in ::setCalback functions
std::vector<ButtonStruct> button_callbacks;

// void task_start_wrapper(void * void_ptr) {
//   ButtonStruct * struct_ptr = (ButtonStruct*) void_ptr;
//   for(auto group : struct_ptr->macro_groups) {
//     group->is_running_ptr = &struct_ptr->is_running;
//     group->group_task_t = &struct_ptr->button_task_t;
//   }

//   struct_ptr->is_running = true;
//   struct_ptr->function();
//   struct_ptr->is_running = false;
//   mutex.take(TIMEOUT_MAX);
//   mutex.give();
//   printf("ended\r\n");
// }

// void interrupt_macro_group(MacroGroup * group) {
//   mutex.take(TIMEOUT_MAX);
//   printf("is_running_ptr = %d\r\n", *group->is_running_ptr);
//   if (*group->is_running_ptr) {
//     pros::c::task_delete(*group->group_task_t);
//     printf("DELETED!\r\n");
//     *group->is_running_ptr = false;
//   }
//   mutex.give();
// }

// void run_buttons() {
//   // Cycle through all button callbacks
//   for (auto &button_callback : button_callbacks) {
//     bool is_pressing = button_callback.controller->get_digital(button_callback.button);
//     bool was_pressed  = (is_pressing &&
//                          !button_callback.was_triggered);
//     bool was_released = (!is_pressing &&
//                          button_callback.was_triggered);
//     bool is_running = false;
//     for (auto &macro_group : button_callback.macro_groups) {
//       is_running = is_running || *macro_group->is_running_ptr;
//     }

//     // If the button has been pressed and the task isn't running
//     if (was_pressed && !is_running) {
//       // Set the function to not run when the button is held
//       button_callback.was_triggered = true;
//       if (button_callback.trigger_on_release) continue;
//     } else if (was_released) {
//       button_callback.was_triggered = false;
//       if (!button_callback.trigger_on_release) continue;
//     } else {
//       continue;
//     }
//     if (button_callback.run_as_task) {
//     // Run the function in a separate task
//     button_callback.button_task_t =
//         pros::c::task_create(task_start_wrapper, (void *)&button_callback,
//             TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "run_buttons");
//     } else {
//       button_callback.function();
//     }
//   }
// }
} // namespace controllerbuttons
