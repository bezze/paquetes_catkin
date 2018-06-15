#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <random>
#include <queue>

template<size_t CONF_LEN>
robmovil_planning::RRTPlanner<CONF_LEN>::RRTPlanner(ros::NodeHandle& nh, uint max_iterations, double goal_bias)
: GridPlanner(nh), goal_bias_(goal_bias), max_iterations_(max_iterations)
{ std::random_device rd; rand_gen_ = std::mt19937(rd());}

template<size_t CONF_LEN>
bool robmovil_planning::RRTPlanner<CONF_LEN>::do_planning(robmovil_msgs::Trajectory& result_trajectory)
{  
  start_config_ = defineStartConfig();
  goal_config_ = defineGoalConfig();
  
  graph_[start_config_] = std::list<SpaceConfiguration>();
  
  bool path_found = false;

  uint i = 0;
  uint32_t totalCellsOccupy = 0;
  while(i < max_iterations_)
  {
    if(isGoalAchieve())
    { path_found = true; break; }
    
    rand_config_ = generateRandomConfig();
    std::cout << "GeneratedRandomConfig: (" << rand_config_.config[0] << ","
    	<< rand_config_.config[1] << ","
    	<< rand_config_.config[2] << ")" << std::endl;
    near_config_ = nearest();
    new_config_ = steer();
    
    if(isFree() and isValid())
    {      
      graph_[near_config_].push_back(new_config_);
      
      if(graph_.count(new_config_) == 0) // ???
        graph_[new_config_] = std::list<SpaceConfiguration>();
    }
    else
    {
    	//ROS_INFO_STREAM("OccupiedCell: " << new_config_.config[0] << "," << new_config_.config[1] << " (" << new_config_.config[2] << " tetha)");
    	++totalCellsOccupy;
    }
    
    i++;
  }
  
  ROS_INFO_STREAM("totalCellsOccupy: " << totalCellsOccupy);
  ROS_INFO_STREAM("Plan " << path_found << " Graph size " << graph_.size() << " it " << i);
  print_rrt_graph();
  
  if(not path_found)
    return false;
    
  std::map<SpaceConfiguration, SpaceConfiguration> came_from;

  if(findPathOnGraph(graph_, start_config_, new_config_, came_from)){
    notifyTrajectory(result_trajectory, start_config_, new_config_, came_from);
    return true;
  }else
    return false;
}

template<size_t CONF_LEN>
bool robmovil_planning::RRTPlanner<CONF_LEN>::findPathOnGraph(std::map<SpaceConfiguration, std::list<SpaceConfiguration> >& graph, 
                                        const SpaceConfiguration& src, const SpaceConfiguration& dest,
                                        std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const
{
  if(graph.count(src) == 0 or graph.count(dest) == 0){
    ROS_INFO_STREAM("No se encuentran src y dest en el grafo");
    return false;
  }

  std::vector<SCWithCost> container;
  std::priority_queue<SCWithCost, std::vector<SCWithCost>, CostCompare> frontier(CostCompare(), container);
  std::map<SpaceConfiguration, double> cost_so_far;
  
  frontier.push(SCWithCost(src, 0));
  cost_so_far[src] = 0;
  
  bool path_found = false;
  
  while( not frontier.empty())
  {
    SCWithCost current = frontier.top();
    frontier.pop();

    if(current == dest)
      path_found = true;
      
    const std::list<SpaceConfiguration>& neighbors = graph[current];
     
    for(SpaceConfiguration next : neighbors){
      
      double new_cost = cost_so_far[current] + 1;
      
      if(cost_so_far.count(next) == 0 or new_cost < cost_so_far[next]){        
        cost_so_far[next] = new_cost;
        came_from[next] = current;
        
        auto it = std::find(container.begin(), container.end(), next);
        
        if(it == container.end()){
          frontier.push(SCWithCost(next, new_cost));
        }else{
          it->cost = new_cost;
          std::make_heap(container.begin(), container.end(), CostCompare());
        }
      }
    }
  }
  
  return path_found;
}

template<size_t CONF_LEN>
void robmovil_planning::RRTPlanner<CONF_LEN>::print_rrt_graph()
{
  std::ofstream rrt_graph_logfile("rrt_graph.log");
  
  start_config_.print(rrt_graph_logfile) << std::endl;
  
  goal_config_.print(rrt_graph_logfile) << std::endl;
  
  for(const auto& edges : graph_)
    for(const auto& edge : edges.second){
      edges.first.print(rrt_graph_logfile) << " ";
      edge.print(rrt_graph_logfile) << std::endl;
    }
      
  rrt_graph_logfile.close();
}
