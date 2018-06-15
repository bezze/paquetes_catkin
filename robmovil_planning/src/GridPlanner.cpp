#include "robmovil_planning/GridPlanner.h"
#include "tf_utils.hpp"
#include <robmovil_msgs/Trajectory.h>
#include <nav_msgs/Path.h>

robmovil_planning::GridPlanner::GridPlanner(ros::NodeHandle& nh)
{
  while( ros::ok() and ros::Time::now() == ros::Time(0) ); // se espera por el primer mensaje de '/clock'

  grid_sub_ = nh.subscribe("/grid", 1, &GridPlanner::on_grid, this);
  goal_sub_ = nh.subscribe("/planning_goal", 1, &GridPlanner::on_goal, this);
  trajectory_pub_ = nh.advertise<robmovil_msgs::Trajectory>("/robot/trajectory", 1, true);
  path_pub_ = nh.advertise<nav_msgs::Path>("/robot/path_planned", 1, true);
}

void robmovil_planning::GridPlanner::publishPath(const robmovil_msgs::Trajectory& trajectory)
{
  nav_msgs::Path path_msg;

  path_msg.header.stamp = trajectory.header.stamp;
  path_msg.header.frame_id = "map";

  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    const robmovil_msgs::TrajectoryPoint& wpoint = trajectory.points[i];

    geometry_msgs::PoseStamped stamped_pose_msg;

    stamped_pose_msg.header.stamp = trajectory.header.stamp;
    stamped_pose_msg.header.frame_id = trajectory.header.frame_id;

    tf::Transform wp_map_ref;
    tf::transformMsgToTF(wpoint.transform, wp_map_ref);
    wp_map_ref = map_to_odom_ * wp_map_ref;

    tf::poseTFToMsg(wp_map_ref, stamped_pose_msg.pose);

    path_msg.poses.push_back(stamped_pose_msg);
  }

  path_pub_.publish( path_msg );
}

void robmovil_planning::GridPlanner::on_grid(const nav_msgs::OccupancyGridConstPtr& grid)
{
  if (not lookupTransformSafe(transform_listener_, "map", "odom", grid->header.stamp, map_to_odom_))
    return;

  ROS_INFO_STREAM("New occupancy grid received");

  grid_ = grid;
}

void robmovil_planning::GridPlanner::on_goal(const geometry_msgs::PoseStamped& goal)
{
  ROS_INFO_STREAM("New goal recived");

  if(!grid_){ ros::Duration(1).sleep(); if(!grid_){ ROS_INFO_STREAM("No grid message received!"); return;} }

  tf::poseStampedMsgToTF(goal, goal_pose_);

  tf::StampedTransform odom_to_robot;
  if (not lookupTransformSafe(transform_listener_, "map", "odom", goal.header.stamp, odom_to_robot))
    return;

  starting_pose_ = tf::Stamped< tf::Pose >(odom_to_robot, odom_to_robot.stamp_, odom_to_robot.frame_id_);

  robmovil_msgs::Trajectory trajectory_msg;

  if(do_planning(trajectory_msg)){
    ROS_INFO_STREAM("A plan has been found");

    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.frame_id = "odom";

    trajectory_pub_.publish( trajectory_msg );
    publishPath(trajectory_msg);
  }else
    ROS_INFO_STREAM("No plan has been found");
}

bool robmovil_planning::GridPlanner::getOriginOfCell(uint i, uint j, double& x, double& y)
{
  if(!grid_ or i < 0 or j < 0 or i >= grid_->info.width or j >= grid_->info.height)
    return false;

  x = grid_->info.origin.position.x + i * grid_->info.resolution;
  y = grid_->info.origin.position.y + j * grid_->info.resolution;

  return true;
}

bool robmovil_planning::GridPlanner::getCenterOfCell(uint i, uint j, double& x, double& y)
{
  if (!getOriginOfCell(i,j,x,y))
    return false;

  x += grid_->info.resolution / 2;
  y += grid_->info.resolution / 2;

  return true;
}

inline double rounddown_nearest(const double& num, const double& multiple)
{
  double r = std::fmod(num, multiple);
  return num < 0 ? num + r : num - r;
}

bool robmovil_planning::GridPlanner::getCellOfPosition(const double& x, const double& y, uint& i, uint& j)
{
  if(!grid_)
    return false;

  /*double floor_width = grid_->info.width * grid_->info.resolution;
  double floor_height = grid_->info.height * grid_->info.resolution;*/

  // ???? sumar la mitad del piso???
  /*double tmp_i = (rounddown_nearest(x, grid_->info.resolution) + floor_width/2) / grid_->info.resolution;
  double tmp_j = (rounddown_nearest(y, grid_->info.resolution) + floor_height/2) / grid_->info.resolution;*/

  double tmp_i = rounddown_nearest(x-grid_->info.origin.position.x, grid_->info.resolution) / grid_->info.resolution;
  double tmp_j = rounddown_nearest(y-grid_->info.origin.position.y, grid_->info.resolution) / grid_->info.resolution;

  if(tmp_i < 0 or tmp_j < 0 or tmp_i >= grid_->info.width or tmp_j >= grid_->info.height)
  {
      return false;
  }

  i = tmp_i;
  j = tmp_j;

  return true;
}

bool robmovil_planning::GridPlanner::isCellOccupy(int i, int j)
{
  if(!grid_ or i < 0 or j < 0 or i >= grid_->info.width or j >= grid_->info.height)
    return true;

  bool res = grid_->data[i + grid_->info.width * j];
  if (res)
      std::cout << "!!! Una celda ocupada POSTa!!" << std::endl;

  return res;
  //return grid_->data[i + grid_->info.width * j];
}

bool robmovil_planning::GridPlanner::isANeighborCellOccupy(int i, int j)
{
  if(!grid_ or i < 0 or j < 0 or i >= grid_->info.width or j >= grid_->info.height)
    return true;

  return isCellOccupy(i-1,j-1) or isCellOccupy(i-1,j) or isCellOccupy(i-1,j+1) or isCellOccupy(i,j-1) or
         isCellOccupy(i,j+1) or isCellOccupy(i+1,j-1) or isCellOccupy(i+1,j) or isCellOccupy(i+1,j+1);
}

bool robmovil_planning::GridPlanner::isCellOrNeighbourhoodOccupy(int i, int j)
{
    for (int k = i-1; k <= i+1; ++k)
      for (int l = j-1; l <= j+1; ++l)
      {
          // check if occupied or not defined
          if (isCellOccupy(k,l))
              return true;
      };

    return false;
}

bool robmovil_planning::GridPlanner::isPositionOccupy(const double& x, const double& y)
{
  uint i, j;

  if(!getCellOfPosition(x,y,i,j))
    return true; // ???

  return isCellOccupy(i,j);
}

bool robmovil_planning::GridPlanner::do_planning(robmovil_msgs::Trajectory& result_trajectory)
{
  ROS_INFO_STREAM("do_planning: " << " Test implementation for debugging!");

  ROS_INFO_STREAM("Grid: " << grid_->info.resolution << " " << grid_->info.width << " " << grid_->info.height << std::endl << " " << grid_->info.origin);

  uint i, j;

  double x = starting_pose_.getOrigin().getX();
  double y = starting_pose_.getOrigin().getY();

  getCellOfPosition(x, y, i, j);

  ROS_INFO_STREAM("starting_pose: " << x << "," << y << " Cell: " << i << "," << j );

  x = goal_pose_.getOrigin().getX();
  y = goal_pose_.getOrigin().getY();

  getCellOfPosition(x, y, i, j);

  ROS_INFO_STREAM("goal_pose: " << x << "," << y << " Cell: " << i << "," << j );

  x = -4.71;
  y = 4.17;

  getCellOfPosition(x, y, i, j);

  ROS_INFO_STREAM("Pos: " << x << "," << y << " Cell: " << i << "," << j);

  x = -0.5;
  y = -0.5;

  getCellOfPosition(x, y, i, j);

  ROS_INFO_STREAM("Pos: " << x << "," << y << " Cell: " << i << "," << j);

  x = -0.25;
  y = -0.25;

  getCellOfPosition(x, y, i, j);

  ROS_INFO_STREAM("Pos: " << x << "," << y << " Cell: " << i << "," << j);

  x = 0.25;
  y = 0.25;

  getCellOfPosition(x, y, i, j);

  ROS_INFO_STREAM("Pos: " << x << "," << y << " Cell: " << i << "," << j);

  x = -7.5;
  y = -7.5;

  ROS_INFO_STREAM("Pos: " << x << "," << y << " Cell: " << i << "," << j << " Occupy " << isPositionOccupy(x,y));

  return false;
}
