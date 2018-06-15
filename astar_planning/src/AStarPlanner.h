#ifndef __ASTARPLANNER_H__
#define __ASTARPLANNER_H__

#include <ros/ros.h>
#include "robmovil_planning/GridPlanner.h"
#include <queue>
#include <vector>

namespace robmovil_planning {

class AStarPlanner : public GridPlanner
{
  public:
    AStarPlanner(ros::NodeHandle& nh);
    
    struct Cell
    {
      uint i = 0;
      uint j =0;
      
      Cell(){};
      Cell(const Cell& c) : i(c.i), j(c.j){};
      Cell(uint _i, uint _j) : i(_i), j(_j){};
      
      bool operator <( const Cell& c2) const
      { return (j < c2.j) or (i < c2.i and j == c2.j); } // Primero por filas (y), luego por columnas (x)
      
      bool operator ==( const Cell& c2) const
      { return j == c2.j and i == c2.i; }
      
      bool operator !=( const Cell& c2) const
      { return j != c2.j or i != c2.i; }
    };

    struct CellWithPriority : Cell
    {
      double priority = 0;
      
      CellWithPriority(){};
      
      CellWithPriority(const Cell& c, const double& _priority)
      : Cell(c), priority(_priority){};
      
      CellWithPriority(uint _i, uint _j, const double& _priority)
      : Cell(_i, _j), priority(_priority){};
    };
    
    class PriorityCompare
    {
      public:
        bool operator() (const CellWithPriority& cwp1, const CellWithPriority& cwp2)
          { return cwp1.priority > cwp2.priority; }
    };

    using CellPriorityQueue = std::priority_queue<CellWithPriority, std::vector<CellWithPriority>, PriorityCompare>;

//    class CellIterablePriorityQueue : public CellPriorityQueue
//    {
//    public:
//        const CellPriorityQueue::container_type & GetInternalRepr()
//        {
//            return c;
//        }
//    };

  protected:    
    /** Callback override */
    bool do_planning(robmovil_msgs::Trajectory& result_trajectory);
    
    std::vector<Cell> neighbors(const Cell& c);
    
    double heuristic_cost(const Cell& start, const Cell& goal, const Cell& current);
    
    void notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory, const Cell& start, const Cell& goal, 
                          std::map<Cell, Cell>& came_from);
};

}

#endif // __ASTARPLANNER_H__
