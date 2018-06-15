#include "AStarPlanner.h"
#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <algorithm>

#define COST_BETWEEN_CELLS 1

robmovil_planning::AStarPlanner::AStarPlanner(ros::NodeHandle& nh)
: GridPlanner(nh)
{ }

std::vector<robmovil_planning::AStarPlanner::Cell> robmovil_planning::AStarPlanner::neighbors(const Cell& c)
{
  /* COMPLETAR: Calcular un vector de vecinos (distintos de c).
   * IMPORTANTE: Tener en cuenta los limites de la grilla (utilizar grid_->info.width y grid_->info.heigh)
   *             y aquellas celdas ocupadas */
  
  std::vector<Cell> neighbors;

  const int cCurrentI = c.i;
  const int cCurrentJ = c.j;

  for (auto i = cCurrentI-1; i <= cCurrentI+1; ++i)
      for (auto j = cCurrentJ-1; j <= cCurrentJ+1; ++j)
      {
          // discard myself
          if (i == j)
              continue;
          // check if occupied or not defined
          //if (!isCellOccupy(i,j))
          if (!isCellOrNeighbourhoodOccupy(i,j))
              neighbors.push_back(Cell(i,j));
      };

  return neighbors;
}

double robmovil_planning::AStarPlanner::heuristic_cost(const Cell& start, const Cell& goal, const Cell& current)
{  
  /* COMPLETAR: Funcion de heuristica de costo */
  unsigned int deltaI = abs(goal.i-current.i);
  unsigned int deltaJ = abs(goal.j-current.j);
//  return std::max(deltaI, deltaJ);
  return deltaI + deltaJ;
}

typedef robmovil_planning::AStarPlanner::Cell __Cell;
double costBetweenCells(const __Cell &cell1, const __Cell &cell2)
{
    return COST_BETWEEN_CELLS;
}

bool robmovil_planning::AStarPlanner::do_planning(robmovil_msgs::Trajectory& result_trajectory)
{
  uint start_i, start_j;
  uint goal_i, goal_j;
  
  getCellOfPosition(starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), start_i, start_j);
  getCellOfPosition(goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), goal_i, goal_j);
  
  /* Celdas de inicio y destino */
  const Cell start = Cell(start_i, start_j);
  const Cell goal = Cell(goal_i, goal_j);
  
  /* Contenedores auxiliares recomendados para completar el algoritmo */
  CellPriorityQueue frontier; // f(v)
  std::set<Cell> sClosed;
  std::map<Cell, Cell> came_from; // prev(v)
  std::map<Cell, double> cost_so_far; // g(v)
  
  bool path_found = false;


  /* COMPLETAR: Utilizar los contenedores auxiliares para la implementacion del algoritmo A*
   * NOTA: Pueden utilizar las funciones neighbors(const Cell& c) y heuristic_cost(const Cell& start, const __Cell& goal, const __Cell& current)
   *       para la resolucion */

  /* Inicializacion de los contenedores (start comienza con costo 0) */
  frontier.push(CellWithPriority(start, 0));

  cost_so_far[start] = 0;
  
  while ( ! frontier.empty() )
  {
      Cell current = frontier.top();

      // chequeamos si current es una copia vieja
      if (sClosed.find(current) != sClosed.end())
      {
          frontier.pop();
          continue;
      }

      // llegué al goal?
      if (current == goal)
      {
          path_found = true;
          break;
      }

      frontier.pop();
      sClosed.insert(current);

      for (const auto &neighbor : neighbors(current))
      {
          if (sClosed.find(neighbor) != sClosed.end())
              continue;

          double cost = cost_so_far[current] + costBetweenCells(current, neighbor);

          // si está en la frontera y el costo nuevo es mayor o igual lo salteamos
          if (cost_so_far.find(neighbor) != cost_so_far.end() && cost >= cost_so_far[neighbor])
              continue;

          frontier.push(CellWithPriority(neighbor, cost + heuristic_cost(start, goal, neighbor))); // lo agregamos
          came_from[neighbor] = current;
          cost_so_far[neighbor] = cost;
      }
  }

  if(not path_found)
    return false;
  
  /* Construccion y notificacion de la trajectoria.
   * NOTA: Se espera que came_from sea un diccionario representando un grafo de forma que:
   *       goal -> intermedio2 -> intermedio1 -> start */
  notifyTrajectory(result_trajectory, start, goal, came_from);

  return true;
}

void robmovil_planning::AStarPlanner::notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory, const Cell& start, const Cell& goal, 
                                                       std::map<Cell, Cell>& came_from)
{
  std::vector<Cell> path;
  Cell current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    ROS_INFO_STREAM("Path " << it->i << ", " << it->j);
    
    double cell_x, cell_y;
    
    getCenterOfCell(it->i, it->j, cell_x, cell_y);
    
    // Se crean los waypoints de la trajectoria
    robmovil_msgs::TrajectoryPoint point_msg;

    point_msg.transform.translation.x = cell_x;
    point_msg.transform.translation.y = cell_y;
    point_msg.transform.translation.z = 0;
    
    if(it != path.rend()-1){
      double delta_x, delta_y;
      getCenterOfCell((it+1)->i, (it+1)->j, delta_x, delta_y);
      delta_x = delta_x - cell_x;
      delta_y = delta_y - cell_y;
      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(delta_y, delta_x)) );
    } else
      point_msg.transform.rotation = tf::createQuaternionMsgFromYaw( angles::normalize_angle(atan2(cell_y, cell_x)) );
    
    result_trajectory.points.push_back( point_msg );
  }
}
