#ifndef ODOM_UTILITIES_H
#define ODOM_UTILITIES_H
#include "main.h"


namespace odomutilities {

// enum GoalType {kSide, kCorner, kCenter};

// class Goal {
//  public:
//   Goal(Point point, std::vector<QAngle> , GoalType goal_type);
//   Goal(Point point, std::vector<QAngle> angles, QLength offset, GoalType goal_type);
//   const Point point;
//   const std::vector<QAngle> angles;
//   const QLength offset;
//   GoalType goal_type;

//   static Goal *closest(Point current_point);
//   static std::vector<Goal*> goals;
// };

extern Point goal_1;
extern Point goal_2;
extern Point goal_3;
extern Point goal_4;
extern Point goal_5;
extern Point goal_6;
extern Point goal_7;

/*        MATCH SETUP
   │                       │
   └───────────────────────┘
┌─────────────────────────────┐
│                             │
│──────────────╩──────────────│
│                             │
│                             │
│                             │
│4═════D═════E═5═F═════G═════6│
│              C              │
│                             │
│                             │
│──A───────────╦───────────B──│
│1             2             3│
└─────────────────────────────┘
   ┌───────────────────────┐
   │                       │
*/


namespace errorcorrection {

extern bool gps_allign;

void start();

}

} // namespace odomutilities

#endif // ODOM_UTILITIES_H
