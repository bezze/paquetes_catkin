#include "PioneerRRTPlanner.h"

#include <angles/angles.h>
#include <queue>
#include <map>
#include <algorithm>
#include <vector>
#include <random>
#include <queue>

constexpr double cMaxGoalAchieveXYDistanceMetres = 0.1f;
constexpr double cMaxGoalAchieveThetaDistanceRadians = M_PI / 2;

typedef robmovil_planning::PioneerRRTPlanner::SpaceConfiguration SpaceConfiguration;

robmovil_planning::PioneerRRTPlanner::PioneerRRTPlanner(ros::NodeHandle& nh)
: RRTPlanner(nh, 0, 0)
{ 
  nh.param<double>("goal_bias", goal_bias_, 0.6);
  int it_tmp;
  nh.param<int>("max_iterations", it_tmp, 20000);
  max_iterations_ = it_tmp >= 0 ? it_tmp : 20000;
  nh.param<double>("linear_velocity_stepping", Vx_step_, 0.05);
  nh.param<double>("angular_velocity_stepping", Wz_step_, 0.025);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineStartConfig()
{
  /* Se toma la variable global de la pose incial y se la traduce a SpaceConfiguration */
  return SpaceConfiguration( { starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), tf::getYaw(starting_pose_.getRotation()) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineGoalConfig()
{
  /* Se toma la variable global de la pose del goal y se la traduce a SpaceConfiguration */
  return SpaceConfiguration( { goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), tf::getYaw(goal_pose_.getRotation()) } );
}

void robmovil_planning::PioneerRRTPlanner::getGoalBiasedZoneLimits(
        const double &cGoalZoneMinMetres,
        const double &cGoalZoneMaxMetres,
        const double &cGridLowerBoundMetres,
        const double &cGridUpperBoundMetres,
        double &zoneMin, double &zoneMax)
{
    zoneMin = std::max(cGoalZoneMinMetres, cGridLowerBoundMetres);
    zoneMax = std::min(cGoalZoneMaxMetres, cGridUpperBoundMetres);
}

double robmovil_planning::PioneerRRTPlanner::GetRandomOOZ(const double &upperBound, const double &bzLowerBound, const double &bzUpperBound)
{
    double r = this->randBetween(0, upperBound);
    return r < bzLowerBound ? r : r + (bzUpperBound-bzLowerBound);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::generateRandomConfig()
{  
  
    /* COMPLETAR: Deben retornar una configuracion aleatoria dentro del espacio de busqueda.
     * 
     * ATENCION: - Tener en cuenta el valor de la variable global goal_bias_
     *           - Pueden utilizar la funcion randBetween(a,b) para la generacion de numeros aleatorios 
     *           - Utilizar las funciones getOriginOfCell() y la informacion de la grilla para establecer el espacio de busqueda:
     *                grid_->info.width, grid_->info.height, grid_->info.resolution */  

    // ancho y alto de la grilla en metros
    const double cGridWidthMetres = grid_->info.width * grid_->info.resolution;
    const double cGridHeightMetres = grid_->info.height * grid_->info.resolution;

    // bordes de la grilla en coordenadas del mapa (x,y) en metros
    const double cGridXPosLowerBoundMetres = grid_->info.origin.position.x;
    const double cGridYPosLowerBoundMetres = grid_->info.origin.position.y;
    const double cGridXPosUpperBoundMetres = grid_->info.origin.position.x + cGridWidthMetres;
    const double cGridYPosUpperBoundMetres = grid_->info.origin.position.y + cGridHeightMetres;

    /*std::cout << "cGridXPosLowerBoundMetres: " << cGridXPosLowerBoundMetres << std::endl;
    std::cout << "cGridYPosLowerBoundMetres: " << cGridYPosLowerBoundMetres << std::endl;
    std::cout << "cGridXPosUpperBoundMetres: " << cGridXPosUpperBoundMetres << std::endl;
    std::cout << "cGridYPosUpperBoundMetres: " << cGridYPosUpperBoundMetres << std::endl;*/

    // Zona del bias
    double goalZoneXRange_min, goalZoneXRange_max;
    double goalZoneYRange_min, goalZoneYRange_max;
    double goalZoneThetaRange_min, goalZoneThetaRange_max;

    const double cGoalBiasedZoneWidthMetres = cGridWidthMetres * cGoalBiasedZonePercent[0];
    const double cGoalBiasedZoneHeightMetres = cGridHeightMetres * cGoalBiasedZonePercent[1];
    const double cGoalBiasedZoneDeltaTheta = 2 * M_PI * cGoalBiasedZonePercent[2];

    // calculamos la intersección entre la "near goal zone" y la grilla
    getGoalBiasedZoneLimits(goal_config_.get(0)-cGoalBiasedZoneWidthMetres/2, goal_config_.get(0)+cGoalBiasedZoneWidthMetres/2, cGridXPosLowerBoundMetres, cGridXPosUpperBoundMetres, goalZoneXRange_min, goalZoneXRange_max);
    getGoalBiasedZoneLimits(goal_config_.get(1)-cGoalBiasedZoneHeightMetres/2, goal_config_.get(1)+cGoalBiasedZoneHeightMetres/2, cGridYPosLowerBoundMetres, cGridYPosUpperBoundMetres, goalZoneYRange_min, goalZoneYRange_max);
    getGoalBiasedZoneLimits(goal_config_.get(2)-cGoalBiasedZoneDeltaTheta/2, goal_config_.get(2)+cGoalBiasedZoneDeltaTheta/2, 0., 2 * M_PI, goalZoneThetaRange_min, goalZoneThetaRange_max);
    goalZoneThetaRange_min = goal_config_.get(2)-cGoalBiasedZoneDeltaTheta/2;
    goalZoneThetaRange_max = goal_config_.get(2)+cGoalBiasedZoneDeltaTheta/2;

    /*std::cout << "goalZoneXRange: " << goalZoneXRange_min << " : " << goalZoneXRange_max << std::endl;
    std::cout << "goalZoneYRange: " << goalZoneYRange_min << " : " << goalZoneYRange_max << std::endl;
    std::cout << "goalZoneThetaRange: " << goalZoneThetaRange_min << " : " << goalZoneThetaRange_max << std::endl;*/

    double r = randBetween(0,1);
    
    // chequeamos si hay que generar adentro de la zona goal-biased
    if (r < goal_bias_)
    {
        // ancho y alto de la grilla en metros

        double x = goalZoneXRange_min + randBetween(0,1) * (goalZoneXRange_max - goalZoneXRange_min);
        double y = goalZoneYRange_min + randBetween(0,1) * (goalZoneYRange_max - goalZoneYRange_min);
        double theta = angles::normalize_angle(goalZoneThetaRange_min + randBetween(0,1) * (goalZoneThetaRange_max - goalZoneThetaRange_min));
        return SpaceConfiguration( {x, y, theta} );
    }
    else // fuera de la zona goal-biased (out of zone, OOZ)
    {
        const double cOOZWidthMetres = cGridWidthMetres - (goalZoneXRange_max - goalZoneXRange_min);
        const double cOOZHeightMetres = cGridHeightMetres - (goalZoneYRange_max - goalZoneYRange_min);
        const double cOOZthetaRad = 2 * M_PI - cGoalBiasedZoneDeltaTheta;

        double x = grid_->info.origin.position.x + GetRandomOOZ(cOOZWidthMetres, goalZoneXRange_min, goalZoneXRange_max);
        double y = grid_->info.origin.position.y + GetRandomOOZ(cOOZHeightMetres, goalZoneYRange_min, goalZoneYRange_max);
        double theta = angles::normalize_angle(goalZoneThetaRange_max + randBetween(0,1) * cOOZthetaRad);
        return SpaceConfiguration( {x, y, theta} );
    }
}

double distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2)
{
  double dist_ori = abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) );

  const auto deltax = c1.get(0) - c2.get(0);
  const auto deltay = c1.get(1) - c2.get(1);
  return sqrt(deltax*deltax + deltay*deltay) + 0.5f * dist_ori;
}

double robmovil_planning::PioneerRRTPlanner::distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2)
{
  /* COMPLETAR: Funcion auxiliar recomendada para evaluar la distancia entre configuraciones
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */
  
  return distancesBetween(c1, c2);
}

// \param[in] container No puede ser vacío
template <typename Iter>
SpaceConfiguration GetNearest(const SpaceConfiguration &origin, const Iter &container)
{
    SpaceConfiguration nearest;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& config : container)
    {
        double d = distancesBetween(config, origin);

        if (d < min_distance)
        {
            min_distance = d;
            nearest = config;
        }
    }
    return nearest;
}

#include <boost/range/adaptors.hpp>

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::nearest()
{
    /* COMPLETAR: Retornar configuracion mas cercana a la aleatoria (rand_config_). DEBE TENER HIJOS LIBRES
     *
     * ATENCION: - Deberan recorrer la variable global graph_ la cual contiene los nodos del arbol
     *             generado hasta el momento como CLAVES DEL DICCIONARIO
     *           - Se recomienda establecer una relacion de distancia entre configuraciones en distancesBetween()
     *             y utilizar esa funcion como auxiliar */

    /*std::vector<RRTGraph::value_type> scNotFullPairs;
    std::copy_if(graph_.begin(), graph_.end(), std::back_inserter(scNotFullPairs), [] (const RRTGraph::value_type &s) { return s.second.size() < 3; });*/

    using boost::adaptors::filtered;
    auto notFullNodeSet = graph_ | filtered([](decltype(graph_)::value_type const& s) { return s.second.size() < 3; })
            | boost::adaptors::transformed([](decltype(graph_)::value_type const& s) { return s.first; });

    /*std::vector<SpaceConfiguration> scNotFull;
    for (auto &p : scNotFullPairs)
        scNotFull.push_back(p.first);*/

    return GetNearest(rand_config_, notFullNodeSet);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::steer()
{  
    /* COMPLETAR: Retornar una nueva configuracion a partir de la mas cercana near_config_.
     *            La nueva configuracion debe ser ademas la mas cercana a rand_config_ de entre las posibles.
     *
     * ATENCION: - Utilizar las variables globales Vx_step_ , Wz_step_ para la aplicaciones de las velocidades
     *           - Pensar en la conversion de coordenadas polares a cartesianas al establecer la nueva configuracion
     *           - Utilizar angles::normalize_angle() */

    /* Ejemplo de como construir una posible configuracion: */
//    double x_posible = near_config_.get(0) /* + algo */;
//    double y_posible = near_config_.get(0) /* + algo */;
//    double theta_posible = angles::normalize_angle(near_config_.get(2) /* + algo */ );
//
//    SpaceConfiguration s_posible({ x_posible, y_posible, theta_posible });

    // construimos el conjunto de configuraciones candidatas
    const double &nearConfigTheta = near_config_.get(2);
    std::set<SpaceConfiguration> scCandidates;
    for (int i = -1; i <= 1; ++i)
    {
        double theta = angles::normalize_angle(nearConfigTheta + i * Wz_step_ * time_step_);
        double x = near_config_.get(0) + Vx_step_ * time_step_ * cos(theta);
        double y = near_config_.get(1) + Vx_step_ * time_step_ * sin(theta);
        scCandidates.insert(SpaceConfiguration({ x, y, theta }));
    }

    SpaceConfiguration steer;

    /* Conjunto de steers ya ocupados en la configuracion near_config_ */
    auto &nearConfigChildren = graph_[near_config_];
    const std::set<SpaceConfiguration> occupiedSteerings(nearConfigChildren.begin(), nearConfigChildren.end());
    std::vector<SpaceConfiguration> freeSteerings;

    std::set_difference(scCandidates.begin(), scCandidates.end(), occupiedSteerings.begin(), occupiedSteerings.end(),
        std::inserter(freeSteerings, freeSteerings.end()));

    /* RECOMENDACION: Establecer configuraciones posibles en free_steerings y calcular la mas cercana a rand_config_ */

    if (freeSteerings.size() == 0)
        std::cout << "!!!!!!!!!! ERROR: no encontramos un free steering" << std::endl;

    return GetNearest(rand_config_, freeSteerings);;
}

bool robmovil_planning::PioneerRRTPlanner::isPointOrNeighbourhoodOccupy(double x, double y)
{
    uint i, j;
    bool res = getCellOfPosition(x, y, i, j);
    //std::cout << "getCellOfPosition: " << (res ? "true" : "false") << std::endl;
    if (res)
    {
        bool res2 = isCellOrNeighbourhoodOccupy(i, j);
        //std::cout << "-->> isCellOrNeighbourhoodOccupy: " << (res2 ? "true" : "false") << std::endl;
        return res2;
    }

    return true;
    //return res ? isCellOrNeighbourhoodOccupy(i, j) : true;
}

bool robmovil_planning::PioneerRRTPlanner::isFree()
{
  /* COMPLETAR: Utilizar la variable global new_config_ para establecer si existe un area segura alrededor de esta */
  return !isPointOrNeighbourhoodOccupy(new_config_.get(0), new_config_.get(1));
}

bool robmovil_planning::PioneerRRTPlanner::isGoalAchieve()
{
  
  /* COMPLETAR: Comprobar si new_config_ se encuentra lo suficientemente cerca del goal.
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */

    double deltaTheta = abs( angles::shortest_angular_distance(new_config_.get(2), goal_config_.get(2)));

    const auto deltax = new_config_.get(0) - goal_config_.get(0);
    const auto deltay = new_config_.get(1) - goal_config_.get(1);

    return sqrt(deltax*deltax + deltay*deltay) < cMaxGoalAchieveXYDistanceMetres
            && deltaTheta < cMaxGoalAchieveThetaDistanceRadians;
}



/* DESDE AQUI YA NO HACE FALTA COMPLETAR */



bool robmovil_planning::PioneerRRTPlanner::isValid()
{ return true; }

void robmovil_planning::PioneerRRTPlanner::notifyTrajectory(robmovil_msgs::Trajectory& result_trajectory,
                                                            const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                                            std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const
{
  std::vector<SpaceConfiguration> path;
  SpaceConfiguration current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }
  
  result_trajectory.header.stamp = ros::Time::now();
  result_trajectory.header.frame_id = "odom";
  
  ros::Duration t_from_start = ros::Duration(0);
  ros::Duration delta_t = ros::Duration(1);

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {    
    double config_x = it->get(0);
    double config_y = it->get(1);
    double config_theta = it->get(2);
    
    tf::Transform wp_odom_ref;
    wp_odom_ref.getOrigin().setX(config_x);
    wp_odom_ref.getOrigin().setY(config_y);
    wp_odom_ref.getOrigin().setZ(0);
    wp_odom_ref.setRotation(tf::createQuaternionFromYaw(config_theta));
    
    wp_odom_ref = map_to_odom_.inverse() * wp_odom_ref;
    
    // Se crean los waypoints de la trayectoria
    robmovil_msgs::TrajectoryPoint point_msg;
    
    transformTFToMsg(wp_odom_ref, point_msg.transform);
    
    if(it != path.rend()-1) {
      double config_dx = (it+1)->get(0) - config_x;
      double config_dy = (it+1)->get(1) - config_y;
      double config_dtheta = angles::shortest_angular_distance(config_theta, (it+1)->get(2));
      point_msg.velocity.linear.x = sqrt(pow(config_dx,2) + pow(config_dy,2));
      point_msg.velocity.angular.z = config_dtheta;
    }else{
      point_msg.velocity.linear.x = 0;
      point_msg.velocity.angular.z = 0;
    }
    
    point_msg.time_from_start = t_from_start;
    
    result_trajectory.points.push_back( point_msg );
    
    t_from_start += delta_t;
  }
}

