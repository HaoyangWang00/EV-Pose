//
// Created by mpl on 23-4-5.
//
#include "type.h"

using namespace EVPose;

double IMUData::sigma_gyroscope_ = 0.0007294729852113113;
double IMUData::sigma_accelerometer_ = 0.0012655720309610252 ;

double IMUData::sigma_gyroscope_walk_ = 0.0012655720309610252;
double IMUData::sigma_accelerometer_walk_ = 5.6386016813618435e-05;

double IMUData::delta_timestamp_ = 0.005;

//Calculate discrete IMU sensor parameters
double IMUData::gyroscope_noise_rw2_ = IMUData::sigma_gyroscope_ * IMUData::sigma_gyroscope_/IMUData::delta_timestamp_;
double IMUData::accelerometer_noise_rw2_ = IMUData::sigma_accelerometer_ *  IMUData::sigma_accelerometer_/IMUData::delta_timestamp_;

Eigen::Matrix3d IMUData::gyroscope_meas_covariance_ = Eigen::Matrix3d::Identity() *  IMUData::gyroscope_noise_rw2_;
Eigen::Matrix3d IMUData::accelerometer_meas_covariance_ =  Eigen::Matrix3d::Identity() * IMUData::accelerometer_noise_rw2_;

double IMUData::gyroscope_bias_rw2_ = IMUData::sigma_gyroscope_walk_ * IMUData::sigma_gyroscope_walk_ * IMUData::delta_timestamp_;
double IMUData::accelerometer_bias_rw2_ = IMUData::sigma_accelerometer_walk_ * IMUData::sigma_accelerometer_walk_ * IMUData::delta_timestamp_;

Eigen::Matrix3d IMUData::gyroscope_bias_rw_covariance_ = Eigen::Matrix3d::Identity() * IMUData::gyroscope_bias_rw2_;
Eigen::Matrix3d IMUData::accelerometer_bias_rw_covariance_ = Eigen::Matrix3d::Identity() * IMUData::accelerometer_bias_rw2_;


