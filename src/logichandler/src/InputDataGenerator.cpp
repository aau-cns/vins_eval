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

#include "InputDataGenerator.hpp"

InputDataGenerator::InputDataGenerator(ros::NodeHandle &nh, std::string &files_dir, std::vector<double> &imu_noise_values, bool is_imu_noisy, bool is_imu_body_ref, bool is_gravity_added_imu, bool synch, int divisor, int n_files) :
  nh_(nh), files_dir_(files_dir), imu_noise_values_(imu_noise_values), is_imu_noisy_(is_imu_noisy), is_imu_body_ref_(is_imu_body_ref), is_gravity_added_imu_(is_gravity_added_imu), synch_(synch), divisor_(divisor), n_files_(n_files) {

  // Publisher
  pub_imu_ = nh_.advertise<sensor_msgs::Imu>("/imu", 200);
  pub_gt_ = nh_.advertise<geometry_msgs::PoseStamped>("/groundtruth", 200);
  pub_tf_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 200);

  std::cout << std::endl;
  ROS_INFO("Publishing: %s", pub_imu_.getTopic().c_str());
  ROS_INFO("Publishing: %s", pub_gt_.getTopic().c_str());
  ROS_INFO("Publishing: %s", pub_tf_.getTopic().c_str());
  std::cout << std::endl;

  // Ros service callback
  server_ = nh_.advertiseService("/startstop_service", &InputDataGenerator::generateData, this);

  // Ros service connection with synch server
  ConnectSynchServer();

  // Initialize random seed
  std::srand (time(nullptr));

  // Initialize converter
  converter_ = new InputDataConverter(is_imu_noisy_, is_imu_body_ref_, is_gravity_added_imu_);
}

InputDataGenerator::~InputDataGenerator() {

  // Delete objects on the heap
  delete parser_;
  delete converter_;
}

void InputDataGenerator::ConnectSynchServer() {
  // Ros service client for synchronous rendering request (persistent connection)
  client_ = nh_.serviceClient<flightgoggles::SetTransformStamped>("/set_tf_service", true);
}

bool InputDataGenerator::generateData(logichandler::startstop::Request &req, logichandler::startstop::Response &res) {

  if (req.start) {  

    // Take randomly number_of_runs filenames from file_dir   
    std::string filename = files_dir_ + "Trajectory_" + std::to_string(std::rand() % n_files_ + 1) + ".csv";

    // Read and parse the data
    parser_->readParseCsv(filename);

    // Get the data
    std::vector<InputDataParser::Input> data;
    data = parser_->getData();

    // Reset imu biases
    converter_->resetImuBiases();

    // Set the discrete time imu noise values
    converter_->setImuNoise(imu_noise_values_);

    // Init counter to be compared with divisor_ in case of synch_
    int cnt = 1;

    // Timing
    double first_data_timestamp = data.at(0).t;
    double start_timestamp = ros::Time::now().toSec();
    double loop_delta_t, data_timestamp;

    // Generate data
    for (const auto &it : data) {

      if (synch_) {

        // Timing
        data_timestamp = it.t - first_data_timestamp;
        double timeSec = start_timestamp+data_timestamp;

        // Set data
        converter_->setData(it);

        // Convert Data
        imu_ = converter_->getNoisyImu(timeSec);
        groundtruth_ = converter_->getGroundTruth(timeSec);
        transform_ = converter_->getTransform(timeSec);

        // TransformStamped to Tf2
        tf2transform_.transforms.assign(1, transform_);

        // if valid data (quaternion norm != 0)
        double qx = transform_.transform.rotation.x;
        double qy = transform_.transform.rotation.y;
        double qz = transform_.transform.rotation.z;
        double qw = transform_.transform.rotation.w;
        double quaternion_norm = std::sqrt(std::pow(qx,2) + std::pow(qy,2) + std::pow(qz,2) + std::pow(qw,2));
        double epsilon = 1e-3;
        if(quaternion_norm > 1-epsilon && quaternion_norm < 1+epsilon) {

          ROS_DEBUG("Publishing imu and gt...");
          // Publish IMU and GT
          pub_imu_.publish(imu_);
          pub_gt_.publish(groundtruth_);

          // Set service request
          srv_.request.tf = transform_;

          // Check if cnt % divisor_ == 0 and if persistent connection valid
          if((cnt % divisor_) == 0) {
            if(client_.isValid()) {

              // Call service request
              if (client_.call(srv_)) {

                // Check responce
                if(srv_.response.result) {
                }

              } else {
                ROS_ERROR("Failed to request synchronous render given Tf!");
              }
            } else {
              ROS_ERROR("Connection with synch service failed! Reconnection...");
              client_.waitForExistence();
              ConnectSynchServer();
            }
          }
        }

      } else {

        // Timing
        loop_delta_t = ros::Time::now().toSec() - start_timestamp;
        data_timestamp = it.t - first_data_timestamp;

        // Sleep if data timestamp is grater than loop delta t
        // this makes possible to publish the data with the correct frequency
        if (data_timestamp > loop_delta_t) {
          ros::Duration(data_timestamp - loop_delta_t).sleep();
        }

        // Set data
        converter_->setData(it);

        // Convert Data
        imu_ = converter_->getNoisyImu(ros::Time::now().toSec());
        groundtruth_ = converter_->getGroundTruth(ros::Time::now().toSec());
        transform_ = converter_->getTransform(ros::Time::now().toSec());

        // TransformStamped to Tf2
        tf2transform_.transforms.assign(1, transform_);

        // if valid data (quaternion norm != 0)
        double qx = transform_.transform.rotation.x;
        double qy = transform_.transform.rotation.y;
        double qz = transform_.transform.rotation.z;
        double qw = transform_.transform.rotation.w;
        double quaternion_norm = std::sqrt(std::pow(qx,2) + std::pow(qy,2) + std::pow(qz,2) + std::pow(qw,2));
        double epsilon = 1e-3;
        if(quaternion_norm > 1-epsilon && quaternion_norm < 1+epsilon) {

          // Publish
          pub_imu_.publish(imu_);
          pub_gt_.publish(groundtruth_);
          pub_tf_.publish(tf2transform_);
        }
      }
      
      // Increase counter
      ++cnt;
    }

    // Publish service responce
    res.stop = true;
  }

  //Done!
  return true;
}
