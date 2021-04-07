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

#ifndef INPUTDATAGENERATOR_HPP
#define INPUTDATAGENERATOR_HPP

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <logichandler/startstop.h>
#include <flightgoggles/SetTransformStamped.h>
#include <flightgoggles/SetTransformStampedRequest.h>
#include <flightgoggles/SetTransformStampedResponse.h>

#include "InputDataParser.hpp"
#include "InputDataConverter.hpp"


/**
* @brief Input Data Generator
*
*/
class InputDataGenerator {

public:

  /**
  * @brief Constructor
  */
  InputDataGenerator(ros::NodeHandle &nh, std::string &files_dir, std::vector<double> &imu_noise_values,  bool is_imu_noisy, bool is_imu_body_ref, bool is_gravity_added_imu, bool synch, int divisor, int n_files);

  /**
  * @brief Destructor
  */
  ~InputDataGenerator();

  /**
  * @brief Service callback
  */
  bool generateData(logichandler::startstop::Request &req, logichandler::startstop::Response &res);

private:

  /**
  * @brief Ros node hanlder
  */
  ros::NodeHandle nh_;

  /**
  * @brief Our Input data parser and converter
  */
  InputDataParser* parser_ = new InputDataParser();
  InputDataConverter* converter_;

  /**
  * @brief Our Publisher
  */
  ros::Publisher pub_imu_;
  ros::Publisher pub_gt_;
  ros::Publisher pub_tf_;

  /**
  * @brief Ros service
  */
  ros::ServiceServer server_;
  ros::ServiceClient client_;
  flightgoggles::SetTransformStamped srv_;

  /**
  * @brief Our data
  */
  sensor_msgs::Imu imu_;
  geometry_msgs::PoseStamped groundtruth_;
  geometry_msgs::TransformStamped transform_;
  tf2_msgs::TFMessage tf2transform_;

  /**
  * @brief Parameters
  */
  std::string files_dir_;
  std::vector<double> imu_noise_values_;
  bool is_imu_noisy_, is_imu_body_ref_, is_gravity_added_imu_, synch_;
  int divisor_;
  int n_files_;

  /**
  * @brief Synch service connection
  */
  void ConnectSynchServer();
};

#endif //INPUTDATAGENERATOR_HPP
