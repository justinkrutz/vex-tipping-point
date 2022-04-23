#include "main.h"

namespace lift {

void init();
void set_callbacks();

class Piston {
 public:
  Piston(pros::ADIDigitalOut& piston, bool is_double = false);
  
  static int piston_cycles;

  void extend();
  void retract();
  void toggle();
  void print();

  bool piston_out = false;
  bool is_double;
  pros::ADIDigitalOut& piston;
};

extern Piston claw_r;
extern Piston claw_l;
extern Piston claw_b;
extern Piston tilter;
extern Piston shift;
} // namespace lift