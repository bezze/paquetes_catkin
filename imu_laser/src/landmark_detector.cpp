#include <vector>
#include <tf/tf.h>
#include <robmovil_msgs/LandmarkArray.h>
#include <sensor_msgs/PointCloud.h>
#include "landmark_detector.h"

#define LANDMARK_DIAMETER 0.1 // metros (0.1 = 10cm)
constexpr float cLandmarkRadius = LANDMARK_DIAMETER / 2;
constexpr float cLandmarkRayGroupTolerance = 0.03f;

robmovil_ekf::LandmarkDetector::LandmarkDetector(ros::NodeHandle& _n) :
    n(_n), transform_received(false)
{
  laser_sub = n.subscribe("/scan", 1, &LandmarkDetector::on_laser_scan, this);
  landmark_pub = n.advertise<robmovil_msgs::LandmarkArray>("/landmarks", 1);
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("/landmarks_pointcloud", 1);

  listener = boost::make_shared<tf::TransformListener>();

  n.param("robot_frame", robot_frame, std::string("base_link"));
  n.param("publish_robot_frame", publish_robot_frame, std::string("base_link"));
  n.param("laser_frame", laser_frame, std::string("laser"));

  ROS_INFO_STREAM("publishing to frame " << publish_robot_frame);
}

tf::Vector3 CalculateCentroid(const tf::Vector3 &point1, const tf::Vector3 &point2)
{
    const tf::Vector3 deltaV = (point2 - point1) * .5f;
    const tf::Vector3 meanV  = (point2 + point1) * .5f;
    const tfScalar lengthD = sqrt(cLandmarkRadius*cLandmarkRadius - deltaV.length2());
    const tf::Vector3 deltaVOrthoNorm = deltaV.cross(tf::Vector3(0,0,1)).normalized();
    const tf::Vector3 d = lengthD * deltaVOrthoNorm;
    const tf::Vector3 c1 = meanV + d;
    const tf::Vector3 c2 = meanV - d;
    return c1.length2() < c2.length2() ? c2 : c1;
}

void robmovil_ekf::LandmarkDetector::on_laser_scan(const sensor_msgs::LaserScanConstPtr& msg)
{
  if(!update_laser_tf(msg->header.stamp)){
    ROS_WARN_STREAM(laser_frame << " -> " << robot_frame << " transform not yet received, not publishing landmarks");
    return;
  }

  /* COMPLETAR: Convertir range,bearing a puntos cartesianos x,y,0.
   * Descartando aquellas mediciones por fuera de los rangos validos */
  std::vector<tf::Vector3> cartesian;

  /* Utilizar la informacion del mensaje para filtrar y convertir */
  const float &range_min = msg->range_min;
  const float &range_max = msg->range_max;

  const float &angle_min = msg->angle_min;
  const float &angle_increment = msg->angle_increment;

  ROS_INFO_STREAM("on_laser_scan::sensor_msgs::LaserScanConstPtr::ranges { ");

  for (int i = 0; i < msg->ranges.size(); ++i)
  {
	const float angle = angle_min + angle_increment * i;
	const float &range = msg->ranges[i];

	ROS_INFO_STREAM("" << range << ", ");

    /* COMPLETAR: p debe definirse con informacion valida y
     * en coordenadas cartesianas */

    // si NaN lo filtramos como inválido
    if (range == 0)
    	continue;

    tf::Vector3 p;
    p.setX(range * cos(angle));
    p.setY(range * sin(angle));
    p.setZ(0);

    /* convierto el punto en relacion al marco de referencia del laser al marco del robot */
    p = laser_transform * p;

    cartesian.push_back(p);
  }

  ROS_INFO_STREAM(" }");

  if (angle_min + angle_increment * (msg->ranges.size()-1) != msg->angle_max)
      ROS_WARN_STREAM("!!!!!!!!! LaserScanConstPtr angle range vector size inconsistente!!");

  /* Mensaje del arreglo de landmarks detectados */
  robmovil_msgs::LandmarkArray landmark_array;
  landmark_array.header.stamp = msg->header.stamp;
  landmark_array.header.frame_id = publish_robot_frame;

  /* VECTORES AUXILIARES: Pueden utilizar landmark_points para ir acumulando
   * mediciones cercanas */
  std::vector<tf::Vector3> landmark_points;

  // centroides estimados de los postes en coordenadas cartesianas
  std::vector<tf::Vector3> centroids;

  for (int i = 0; i < cartesian.size(); ++i)
  {
    /* COMPLETAR: Acumular, de manera secuencial, mediciones cercanas (distancia euclidea) */

    const tf::Vector3 &v = cartesian[i];
    landmark_points.push_back(v);

    if (i < (cartesian.size()-1))
    {
        const tf::Vector3 &next = cartesian[i+1];

        // si la distancia al punto anterior es menor que la cota, agrupamos
        if (v.distance(next) < LANDMARK_DIAMETER * (1+cLandmarkRayGroupTolerance))
            continue;
    }

    /* Al terminarse las mediciones provenientes al landmark que se venia detectando,
     * se calcula la pose del landmark como el centroide de las mediciones */

    ROS_INFO_STREAM("landmark con " << landmark_points.size() << " puntos");

    tf::Vector3 centroid(0,0,0);

    /* COMPLETAR: calcular el centroide de los puntos acumulados */
    if (landmark_points.size() == 1)
        centroid = landmark_points[0];
    else // si tenemos más de un punto el centroide queda unívocamente determinado
    {
        auto lm1 = landmark_points[0];
        auto lm2 = landmark_points[landmark_points.size()-1];

        centroid = CalculateCentroid(lm1, lm2);
    }

    ROS_INFO_STREAM("landmark detectado (cartesianas): " << centroid.getX() << " " << centroid.getY() << " " << centroid.getZ());
    centroids.push_back(centroid);

    /* Convertir el centroide a coordenadas polares, construyendo el mensaje requerido */
    robmovil_msgs::Landmark landmark;

    float r = centroid.length(); // distancia desde el robot al centroide
    landmark.range = r;

    float a = atan2(centroid.getY(), centroid.getX()); // angulo de la recta que conecta al robot con el centroide
    landmark.bearing = a;

    /* se agrega el landmark en coordenadas polares */
    landmark_array.landmarks.push_back(landmark);
    ROS_INFO_STREAM("landmark detectado (polares): " << i << ": " << landmark.range << " " << landmark.bearing);

    /* empiezo a procesar un nuevo landmark */
    landmark_points.clear();
  }

  /* Publicamos el mensaje de los landmarks encontrados */
  if (!landmark_array.landmarks.empty()){
    landmark_pub.publish(landmark_array);
    publish_pointcloud(landmark_array.header, centroids);
  }
}

bool robmovil_ekf::LandmarkDetector::update_laser_tf(const ros::Time& required_time)
{
  if (!listener->waitForTransform(robot_frame, laser_frame, required_time, ros::Duration(1)))
    return false;
  else
  {
    listener->lookupTransform(robot_frame, laser_frame, ros::Time(0), laser_transform);
    return true;
  }
}

void robmovil_ekf::LandmarkDetector::publish_pointcloud(const std_msgs::Header& header, const std::vector<tf::Vector3>& landmark_positions)
{
  sensor_msgs::PointCloud pointcloud;
  pointcloud.header.stamp = header.stamp;
  pointcloud.header.frame_id = header.frame_id;

  for (int i = 0; i < landmark_positions.size(); i++)
  {
    geometry_msgs::Point32 point;
    point.x = landmark_positions[i].getX();
    point.y = landmark_positions[i].getY();
    point.z = landmark_positions[i].getZ();
    pointcloud.points.push_back(point);
  }
  pointcloud_pub.publish(pointcloud);
}
