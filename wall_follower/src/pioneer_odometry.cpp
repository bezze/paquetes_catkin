#include "pioneer_odometry.h"
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

using namespace robmovil;

#define WHEEL_BASELINE 0.331
#define WHEEL_RADIUS 0.0975
#define ENCODER_TICKS 500.0
#define GEAR_REDUCTION 1.0

PioneerOdometry::PioneerOdometry(ros::NodeHandle& nh)
  : nh_(nh), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = nh.subscribe("/robot/cmd_vel", 1, &PioneerOdometry::on_velocity_cmd, this);

  vel_pub_left_ = nh.advertise<std_msgs::Float64>("/robot/left_wheel/cmd_vel", 1);
  vel_pub_right_ = nh.advertise<std_msgs::Float64>("/robot/right_wheel/cmd_vel", 1);

  encoder_sub_ = nh.subscribe("/robot/encoders", 1, &PioneerOdometry::on_encoder_ticks, this);

  pub_odometry_ = nh.advertise<nav_msgs::Odometry>("/robot/odometry", 1);

  tf_broadcaster = boost::make_shared<tf::TransformBroadcaster>();
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::Twist& twist)
{
  /** Completar los mensajes de velocidad */
  const double &Rpx = twist.linear.x;
  const double &Rpy = twist.linear.y;
  const double &Rpt = twist.angular.z;

  double vLeft = 1/WHEEL_RADIUS*Rpx + 0*Rpy - 0.5*WHEEL_BASELINE/WHEEL_RADIUS*Rpt; // fi_p_l
  double vRight= 1/WHEEL_RADIUS*Rpx + 0*Rpy + 0.5*WHEEL_BASELINE/WHEEL_RADIUS*Rpt; // fi_p_r


  // publish left velocity
  {
    std_msgs::Float64 msg;
    msg.data = vLeft;

    vel_pub_left_.publish( msg );
  }

  // publish right velocity
  {
    std_msgs::Float64 msg;
    msg.data = vRight;

    vel_pub_right_.publish( msg );
  }
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::EncoderTicks& encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (not ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_left_ = encoder.ticks_left.data;
    last_ticks_right_ = encoder.ticks_right.data;
    last_ticks_time = encoder.header.stamp;
    return;
  }

  int32_t delta_ticks_left = encoder.ticks_left.data - last_ticks_left_;
  int32_t delta_ticks_right = encoder.ticks_right.data - last_ticks_right_;

  // calcular el desplazamiento relativo

  /* Utilizar este delta de tiempo entre momentos */
  double delta_t = (encoder.header.stamp - last_ticks_time).toSec();

  const double arcoRight = GEAR_REDUCTION*M_PI*WHEEL_RADIUS*2*delta_ticks_right/ENCODER_TICKS;
  const double arcoLeft = GEAR_REDUCTION*M_PI*WHEEL_RADIUS*2*delta_ticks_left/ENCODER_TICKS;
  const double deltaTheta = (arcoRight-arcoLeft)/WHEEL_BASELINE;
  const double arcoMean = (arcoRight + arcoLeft)/2;

  /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
  theta_ += deltaTheta;
  x_ += arcoMean*cos(theta_);
  y_ += arcoMean*sin(theta_);

  // Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::Odometry msg;

  msg.header.stamp = encoder.header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

  msg.twist.twist.linear.x = arcoMean/delta_t;
  msg.twist.twist.linear.y = 0; 
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = deltaTheta/delta_t;

  pub_odometry_.publish( msg );

  // Actualizo las variables de estado

  last_ticks_left_ = encoder.ticks_left.data;
  last_ticks_right_ = encoder.ticks_right.data;
  last_ticks_time = encoder.header.stamp;

  /* Mando tambien un transform usando TF */
  tf::Transform t;
  tf::poseMsgToTF(msg.pose.pose, t);
  tf_broadcaster->sendTransform(tf::StampedTransform(t, encoder.header.stamp, "odom", "base_link"));

}
