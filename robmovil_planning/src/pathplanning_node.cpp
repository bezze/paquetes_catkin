#include <ros/ros.h>
#include "robmovil_planning/GridPlanner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle n("~");
  robmovil_planning::GridPlanner GridPlanner(n);
  ros::spin();
  return 0;
}
