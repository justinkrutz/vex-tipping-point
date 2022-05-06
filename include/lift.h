#include "main.h"

namespace lift {

void init();
void set_callbacks();

class Piston {
 public:
  Piston(pros::ADIDigitalOut& piston, bool invert = false, bool is_double = false);
  
  static int piston_cycles;

  void extend();
  void retract();
  void toggle();
  void print();

  bool invert = false;
  bool piston_out = false;
  bool is_double;
  pros::ADIDigitalOut& piston;
};

extern Piston claw;
extern Piston tilter;
extern Piston tilter_release;
extern Piston shooter;

extern bool auto_grip_ready;
extern bool auto_grip_enabled;
extern bool goal_detect_center_only;

} // namespace lift