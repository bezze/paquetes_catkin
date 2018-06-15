#ifndef __PATHPLANNER_H__
#define __PATHPLANNER_H__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <robmovil_msgs/Trajectory.h>

namespace robmovil_planning {

class GridPlanner
{
  public:
    GridPlanner(ros::NodeHandle& nh);
    
  protected:
  
    /* Transformacion entre el mapa y el marco inercial de movimiento (odom) */
    tf::StampedTransform map_to_odom_;
    
    /* Se almacena el puntero del mensaje /grid recibido
     * NOTA: Si es null entonces se invalidan las funciones auxiliares */
    nav_msgs::OccupancyGridConstPtr grid_;
    
    /* Pose donde inicia el robot (map_to_odom) y pose del goal */
    tf::Stamped< tf::Pose > starting_pose_;
    tf::Stamped< tf::Pose > goal_pose_;
    
    /* Funcionen auxiliares para operar con la grilla de ocupacion.
     * Las posiciones x,y obtenidas son referentes al marco del mapa (map) */
     
    /* Posicion de la esquina/centro de la celda i,j. 
     * Devuelve false en caso de que los parametros sean invalidos o grid_ = NULL */
    bool getOriginOfCell(uint i, uint j, double& x, double& y);
    bool getCenterOfCell(uint i, uint j, double& x, double& y);
    
    /* Celda correspondiente a la posicion x,y (referente al marco del mapa)
     * Devuelve false en caso de que los parametros sean invalidos o grid_ = NULL */
    bool getCellOfPosition(const double& x, const double& y, uint& i, uint& j);

    /* Funciones de comprobacion de ocupacion, se considera la celda ocupada si
     * posee un valor asignado distinto de 0 (es decir, no se utiliza threshold)
     * Devuelve false en caso de que los parametros sean invalidos o grid_ = NULL */    
    bool isCellOccupy(int i, int j);
    bool isCellOrNeighbourhoodOccupy(int i, int j);
    bool isANeighborCellOccupy(int i, int j);
    bool isPositionOccupy(const double& x, const double& y);
    
    /**
     * Callback que implementaran los planificadores derivados.
     * 
     * @result_trajectory
     *  Trayectoria encontrada entre start y goal.
     * 
     * @return
     *   True si pudo encontrarse un camino entre start y goal. False en caso contrario.
     */
    virtual bool do_planning(robmovil_msgs::Trajectory& result_trajectory);

  private:
    ros::Subscriber grid_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher path_pub_;
    
    void publishPath(const robmovil_msgs::Trajectory& trajectory);

    tf::TransformListener transform_listener_;

    void on_grid(const nav_msgs::OccupancyGridConstPtr& grid);
    void on_goal(const geometry_msgs::PoseStamped& goal);
};

}
#endif // __PATHPLANNER_H__
