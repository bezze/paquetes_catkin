#include <angles/angles.h>
#include "KinematicPositionController.h"
#include "tf_utils.hpp"

KinematicPositionController::KinematicPositionController(ros::NodeHandle& nh) :
  TrajectoryFollower(nh), transform_listener_( tfBuffer_ )
{
    expected_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);
    
    ros::NodeHandle nhp("~");
    
    std::string goal_selection;
    nhp.param<std::string>("goal_selection", goal_selection, "TIME_BASED");
    nhp.param<double>("fixed_goal_x", fixed_goal_x_, 3);     
    nhp.param<double>("fixed_goal_y", fixed_goal_y_, 0);     
    nhp.param<double>("fixed_goal_a", fixed_goal_a_, -M_PI_2);     
    
    if(goal_selection == "TIME_BASED")
      goal_selection_ = TIME_BASED;
    else if(goal_selection == "PURSUIT_BASED")
      goal_selection_ = PURSUIT_BASED;
    else if(goal_selection == "FIXED_GOAL")
      goal_selection_ = FIXED_GOAL;
    else
      goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const ros::Time& t0, const ros::Time& t1, double y0, double y1, const ros::Time& t)
{
  return y0 + (t - t0).toSec() * (y1 - y0) / (t1 - t0).toSec();
}

bool KinematicPositionController::getCurrentPose(const ros::Time& t, double& x, double& y, double& a)
{
  tf2::Transform odom_to_robot;
  if (not lookupTransformSafe(tfBuffer_, "odom", "base_link", t, odom_to_robot))
    return false;

  x = odom_to_robot.getOrigin().getX();
  y = odom_to_robot.getOrigin().getY();

  a = tf2::getYaw(odom_to_robot.getRotation());

  return true;
}

/**
 * NOTA: Para un sistema estable mantener:
 * - 0 < K_RHO
 * - K_RHO < K_ALPHA
 * - K_BETA < 0
 */
#define K_RHO 0.5 // 1
#define K_ALPHA 1.5 // 1.5
#define K_BETA -1.5 // -1

bool KinematicPositionController::control(const ros::Time& t, double& v, double& w)
{
  // Se obtiene la pose actual publicada por la odometria
  double current_x, current_y, current_a;
  if( not getCurrentPose(t, current_x, current_y, current_a) )
    return true;

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz

  /** EJERCICIO 1: COMPLETAR: Aqui deberan realizar las cuentas necesarias para determinar:
   *             - la velocidad lineal: asignando la variable v
   *             - la velocidad angular: asignando la variable w 
   *  
   *  RECORDAR: cambiar el marco de referencia en que se encuentran dx, dy y theta */

  double dx_I = goal_x - current_x;
  double dy_I = goal_y - current_y;
  //double thetaGoal_I = current_a;

  double dx_G = cos(-goal_a) * dx_I - sin(-goal_a) * dy_I;
  double dy_G = sin(-goal_a) * dx_I + cos(-goal_a) * dy_I;
  double thetaRobot_G = current_a - goal_a;
  
  // Computar variables del sistema de control
  double rho = sqrt(dx_G * dx_G + dy_G * dy_G);
  double alpha = angles::normalize_angle(atan2(dy_G, dx_G) - thetaRobot_G); // Normalizes the angle to be -M_PI circle to +M_PI circle It takes and returns radians.
  double beta =  angles::normalize_angle(-thetaRobot_G - alpha); // Realizar el calculo dentro del metodo de normalizacion

  /* Calcular velocidad lineal y angular* 
   * Existen constantes definidas al comienzo del archivo para
   * K_RHO, K_ALPHA, K_BETA */
  v = K_RHO * rho;
  w = K_ALPHA * alpha + K_BETA * beta;

  ROS_INFO_STREAM("atan2: " << atan2(dy_I, dx_I) << " theta siegwart: " << thetaRobot_G << " expected_atheta: "  << current_a << " rho: " << rho << " alpha: " << alpha << " beta: " << beta << " v: " << v << " w: " << w);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{ return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));}

double Distance(double x1, double y1, double x2, double y2)
{
    const double distanceX = x1 - x2;
    const double distanceY = y1 - y2;
    return sqrt(distanceX*distanceX + distanceY*distanceY);
}

constexpr double cLookAheadRadius = .1;

bool KinematicPositionController::getPursuitBasedGoal(const ros::Time& t, double& x, double& y, double& a)
{
  // Los obtienen los valores de la posicion y orientacion actual.
  double current_x, current_y, current_a;
  if( not getCurrentPose(t, current_x, current_y, current_a) )
    return true;
    
  // Se obtiene la trayectoria requerida.
  const robmovil_msgs::Trajectory& trajectory = getTrajectory();
  
  /** EJERCICIO 3:
   * Se recomienda encontrar el waypoint de la trayectoria más cercano al robot en términos de x,y
   * y luego buscar el primer waypoint que se encuentre a una distancia predefinida de lookahead en x,y */
  
  // calculamos el waypoint de la trayectoria más cercano al robot
  double minDistanceWaypoint = std::numeric_limits<double>::max();
  unsigned int minDistanceWaypointIdx = 0;
  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
      const robmovil_msgs::TrajectoryPoint& wpoint = trajectory.points[i];
      double wpoint_x = wpoint.transform.translation.x;
      double wpoint_y = wpoint.transform.translation.y;
      double distance = Distance(wpoint_x, wpoint_y, current_x, current_y);
      if (distance < minDistanceWaypoint)
      {
          minDistanceWaypoint = distance;
          minDistanceWaypointIdx = i;
      }
  }

  /* NOTA: De esta manera les es posible recorrer la trayectoria requerida */  
  for(unsigned int i = minDistanceWaypointIdx+1; i < trajectory.points.size(); i++)
  {
    // Recorren cada waypoint definido
    const robmovil_msgs::TrajectoryPoint& wpoint = trajectory.points[i];
    
    // Y de esta manera puede acceder a la informacion de la posicion y orientacion requerida en el waypoint
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);
    
    //...
    // calculamos la distancia del robot al waypoint
    double distance = Distance(wpoint_x, wpoint_y, current_x, current_y);
    if (distance > cLookAheadRadius)
    {
        x = wpoint_x;
        y = wpoint_y;
        a = wpoint_a;
        return true;
    }
  }
  
  /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
  return false;
}

bool KinematicPositionController::getTimeBasedGoal(const ros::Time& t, double& x, double& y, double& a)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;
    
  ROS_INFO_STREAM("processing index: " << next_point_idx);

  const robmovil_msgs::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const ros::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const ros::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);

  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}
