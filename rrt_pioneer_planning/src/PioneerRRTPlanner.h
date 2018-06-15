#ifndef __PIONEERRRTPLANNER_H__
#define __PIONEERRRTPLANNER_H__

#include <vector>
#include <map>
#include <stdexcept>
#include <algorithm>
#include <initializer_list>
#include <random>

#include <ros/ros.h>
#include <tf/tf.h>

#include "rrt_planning/RRTPlanner.h"

namespace robmovil_planning {

class PioneerRRTPlanner : public robmovil_planning::RRTPlanner<3>
{
  public:
    PioneerRRTPlanner(ros::NodeHandle& nh);

  protected:
  
    double time_step_ = 1.f;
    double Vx_step_;
    double Wz_step_;
  
    virtual double distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2);
  
    SpaceConfiguration defineStartConfig();
    SpaceConfiguration defineGoalConfig();
    
    SpaceConfiguration generateRandomConfig();
    SpaceConfiguration nearest();
    SpaceConfiguration steer();
    
    bool isGoalAchieve();
    bool isFree();
    bool isValid();
    
    void notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory,
                                  const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                  std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const;

  private:
    // radio del goal, define la distancia máxima para la cual asumimos que ya llegamos al goal
    static constexpr double cGoalRadius = 1;

    // porcentajes del tamaño de la zona de goal-bias para cada dimensión
    static constexpr double cGoalBiasedZonePercent[] { .1f /*X*/, .1f /*Y*/, .1f /*theta*/};

    // dado un punto (x,y) devuelve la ocupación de su celda y vecindad asociadas
    bool isPointOrNeighbourhoodOccupy(double x, double y);

    // calcula la intersección entre la grilla y la goal-biased-zone (zona-"cae cerca del goal")
    static void getGoalBiasedZoneLimits(
            const double &cGoalZoneMinMetres,
            const double &cGoalZoneMaxMetres,
            const double &cGridLowerBoundMetres,
            const double &cGridUpperBoundMetres,
            double &zoneMin, double &zoneMax);

    //
    double GetRandomOOZ(const double &upperBound, const double &bzLowerBound, const double &bzUpperBound);
};

}

#endif // __PIONEERRRTPLANNER_H__
