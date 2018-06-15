#ifndef __ROBMOVIL_EKF_IMU_CALIBRATOR_H__
#define __ROBMOVIL_EKF_IMU_CALIBRATOR_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>

namespace robmovil_ekf {
  class IMUCalibrator
  {
    public:
      IMUCalibrator(ros::NodeHandle& n);

      void on_imu_measurement(const sensor_msgs::Imu& msg);

    private:

      void calculate_bias(const ros::TimerEvent &event);

      ros::NodeHandle& n;

      ros::Timer timer_;

      ros::Subscriber imu_sub;
      ros::Publisher imu_calib_pub;
      
      /* VARIABLES GLOBALES: Pueden utilizar estas variables y 
       * cualquier otra que necesiten para resolver los ejercicios */
      
      // tiempo de la ultima medicion
      ros::Time time_last_measure_;
      // flag indicando si todavia se esta calibrando
      bool is_calibrating_;
      // bias final calculado, luego del tiempo de calibracion
      tf::Vector3 bias_;
      // vector auxiliar para acumular velocidades angulares durante el periodo de calibracion
      std::vector<tf::Vector3> calibration_data_;
      // Orientacion estimada hasta el momento
      tf::Quaternion orientacion_estimada_;
  };
}

#endif // __ROBMOVIL_EKF_IMU_CALIBRATOR_H__
