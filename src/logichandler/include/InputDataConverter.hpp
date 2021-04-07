/// Copyright (C) 2021 Alessandro Fornasier, Control of Networked Systems,
/// University of Klagenfurt, Austria.
///
/// All rights reserved.
///
/// This software is licensed under the terms of the BSD-2-Clause-License with
/// no commercial use allowed, the full terms of which are made available in the
/// LICENSE file. No license in patents is granted.
///
/// You can contact the author at alessandro.fornasier@ieee.org

#ifndef INPUTDATACONVERTER_HPP
#define INPUTDATACONVERTER_HPP

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <random>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "InputDataParser.hpp"

/**
* @brief Input Data Converter.
*
* This class has a series of functions that allows to generate transformations
* and noisy imu data starting from raw Input data overwriting the timestamps
*
*/
class InputDataConverter {

public:

  /**
  * @brief constructor
  */
  InputDataConverter(bool is_imu_noisy, bool is_imu_body_ref, bool is_gravity_added_imu);

  /**
  * @brief Set IMU noise values
  */
  void setImuNoise(const std::vector<double> &imu_noise_values);

  /**
  * @brief Set new data from parsed trajectory data
  */
  void setData(const InputDataParser::Input &data);

  /**
  * @brief Get noisy IMU data formatted as sensor_msgs::Imu
  *
  * @param Timestamp in seconds
  */
  const sensor_msgs::Imu &getNoisyImu(const double &timestamp);

 /**
  * @brief Get ground truth formatted as geometry_msgs::PoseStamped
  *
  * @param Timestamp in seconds
  */
  const geometry_msgs::PoseStamped &getGroundTruth(const double &timestamp);

  /**
  * @brief Get ground truth formatted as geometry_msgs::TransformStamped
  *
  * @param Timestamp in seconds
  */
  const geometry_msgs::TransformStamped &getTransform(const double &timestamp);

  /**
  * @brief Reset (set zeros) imu biases
  */
  void resetImuBiases();

private:

  /**
  * @brief Boolean that describes IMU data
  *
  * We consider different case, the case of body referenced or world referenced imu,
  * the case of linear acceleration with or without gravity, and the case of noisy or
  * noise-free IMU measurements
  */
  bool is_imu_noisy_, is_imu_body_ref_, is_gravity_added_imu_;

  /**
  * @brief Extended Data struct to be converted
  *
  * | timestamp (t)
  * | position (p)
  * | rotation - quaternion (q) [defined as: x = R(q)x_b]
  * | angular velocity (w)
  * | acceleration in world frame (a)
  *
  * acceleration in body frame (a_b)
  * gravity (g)
  */
  struct Data {
    double t_;
    Eigen::Vector3d p_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d w_;
    Eigen::Vector3d a_;
    Eigen::Vector3d a_b_;
    Eigen::Vector3d g_;
  } data_;

  /**
  * @brief Imu noise densities and random walk
  *
  * Data is ordered inside the vector as follows:
  * [0] acc noise density
  * [1] gyro noise density
  * [2] acc random walk
  * [3] gyro random walk
  */
  std::vector<double> imu_noise_values_;

  /**
  * @brief Imu biases
  *
  * Data is ordered inside the vector as follows:
  * [0] x acc bias
  * [1] y acc bias
  * [2] z acc bias
  * [4] x gyro bias
  * [5] y gyro bias
  * [6] z gyro bias
  */
  Eigen::VectorXd imu_biases_ = Eigen::VectorXd::Zero(6);

  /**
  * @brief Noisy Imu data
  */
  sensor_msgs::Imu noisy_imu_data_;

  /**
  * @brief Ground truth data
  */
  geometry_msgs::PoseStamped ground_truth_;

  /**
  * @brief Transformations
  */
  geometry_msgs::TransformStamped transform_;

};

#endif //INPUTDATACONVERTER_HPP
