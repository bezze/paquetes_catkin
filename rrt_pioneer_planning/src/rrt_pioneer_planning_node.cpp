#include <ros/ros.h>
#include <iostream>

#include "PioneerRRTPlanner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle n("~");
  
  robmovil_planning::PioneerRRTPlanner PioneerRRTPlanner(n);

  ros::spin();
  return 0;
}
