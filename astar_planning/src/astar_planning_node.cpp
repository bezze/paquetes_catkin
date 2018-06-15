#include <ros/ros.h>
#include "AStarPlanner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle n("~");
  robmovil_planning::AStarPlanner AStarPlanner(n);
  ros::spin();
  return 0;
}
