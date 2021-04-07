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

// Main function
int main(int argc, char** argv) {

  // Launch ros node
  ros::init(argc, argv, "inputdatahandler");
  ros::NodeHandle nh("~");

  // Get parameters
  std::string files_dir;
  std::vector<double> imu_noise_values(4);
  std::vector<double> discrete_imu_noise_values(4);
  bool synch, is_imu_noisy, is_imu_body_ref, is_gravity_added_imu;
  int n_files, imu_rate, cam_rate, divisor;

  nh.param<bool>("synch", synch, false);

  if (synch) {
    ROS_INFO("Synchronous rendering mode");
  }

  if(!nh.getParam("files_dir", files_dir)) {
    std::cout << std::endl;
    ROS_ERROR("No files directory defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("n_files", n_files)) {
    std::cout << std::endl;
    ROS_ERROR("No number of files available in the directory defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("imu_rate", imu_rate)) {
    std::cout << std::endl;
    ROS_ERROR("No imu sample rate defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("cam_rate", cam_rate)) {
    std::cout << std::endl;
    ROS_ERROR("No camera sample rate defined");
    std::exit(EXIT_FAILURE);
  }

  if((imu_rate % cam_rate) != 0) {
    std::cout << std::endl;
    ROS_ERROR("Imu sample rate not divisible by camera sample rate! Change the camera sample rate to be a divisor of imu sample rate");
    std::exit(EXIT_FAILURE);
  }

  divisor = imu_rate/cam_rate;

  if(!nh.getParam("is_imu_noisy", is_imu_noisy)) {
    std::cout << std::endl;
    ROS_ERROR("Information on IMU noise missing");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("is_imu_body_ref", is_imu_body_ref)) {
    std::cout << std::endl;
    ROS_ERROR("IMU measurement reference frame missing");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("is_gravity_added_imu", is_gravity_added_imu)) {
    std::cout << std::endl;
    ROS_ERROR("Informationon gravity missing");
    std::exit(EXIT_FAILURE);
  }

  nh.param<double>("acc_noise_density", imu_noise_values[0], 1e-3);
  nh.param<double>("gyro_noise_density", imu_noise_values[1], 1e-4);
  nh.param<double>("acc_random_walk", imu_noise_values[2], 1e-4);
  nh.param<double>("gyro_random_walk", imu_noise_values[3], 1e-5);

  // Convert continuous time noise to discrete time
  discrete_imu_noise_values[0] = imu_noise_values[0] * std::sqrt(imu_rate);
  discrete_imu_noise_values[1] = imu_noise_values[1] * std::sqrt(imu_rate);
  discrete_imu_noise_values[2] = imu_noise_values[2] * std::sqrt(1.0/imu_rate);
  discrete_imu_noise_values[3] = imu_noise_values[3] * std::sqrt(1.0/imu_rate);

  if(!is_imu_noisy) {
    std::cout << "Convert continuous time noise to discrete time:" << std::endl;
    std::cout << "Accelerometer noise density (continuous time): " << imu_noise_values[0] << std::endl;
    std::cout << "Gyroscope noise density (continuous time): " << imu_noise_values[1] << std::endl;
    std::cout << "Accelerometer random walk (continuous time): " << imu_noise_values[2] << std::endl;
    std::cout << "Gyroscope random walk (continuous time): " << imu_noise_values[3] << std::endl;
    std::cout << "Accelerometer noise density (discrete time): " << discrete_imu_noise_values[0] << std::endl;
    std::cout << "Gyroscope noise density (discrete time): " << discrete_imu_noise_values[1] << std::endl;
    std::cout << "Accelerometer random walk (discrete time): " << discrete_imu_noise_values[2] << std::endl;
    std::cout << "Gyroscope random walk (discrete time): " << discrete_imu_noise_values[3] << std::endl;
  }

  // Instanciate data generator
  InputDataGenerator *generator = new InputDataGenerator(nh, files_dir, discrete_imu_noise_values, is_imu_noisy, is_imu_body_ref, is_gravity_added_imu, synch, divisor, n_files);

  // Spinning
  ros::spin();

  // Deleate varaibles on the heap;
  delete generator;

  // Done!
  std::cout << std::endl;
  ROS_INFO("Done!");
  return EXIT_SUCCESS;

}


