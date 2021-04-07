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

#include "InputDataConverter.hpp"

InputDataConverter::InputDataConverter(bool is_imu_noisy, bool is_imu_body_ref, bool is_gravity_added_imu) :
is_imu_noisy_(is_imu_noisy), is_imu_body_ref_(is_imu_body_ref), is_gravity_added_imu_(is_gravity_added_imu) {}

void InputDataConverter::setImuNoise(const std::vector<double> &imu_noise_values) {
  imu_noise_values_.clear();
  imu_noise_values_ = imu_noise_values;
}

void InputDataConverter::setData(const InputDataParser::Input &data) {

  // ENU world (the imu measure 9.81 along z when copter is not moving)
  data_.g_ << 0.0, 0.0, 9.81;

  // Ground truth position
  data_.p_.x() = data.p.x();
  data_.p_.y() = data.p.y();
  data_.p_.z() = data.p.z();

  // Ground truth orientation - Quaternion
  data_.q_.x() = data.q.x();
  data_.q_.y() = data.q.y();
  data_.q_.z() = data.q.z();
  data_.q_.w() = data.q.w();

  // Angular velocity
  data_.w_.x() = data.w.x();
  data_.w_.y() = data.w.y();
  data_.w_.z() = data.w.z();


  if(is_imu_body_ref_) {

    // Body referenced acceleration
    data_.a_b_.x() = data.a.x();
    data_.a_b_.y() = data.a.y();
    data_.a_b_.z() = data.a.z();

    if(!is_gravity_added_imu_) {

      // Convert acceleration to world frame
      data_.a_ = data_.q_.toRotationMatrix() * data_.a_b_;

      // Sum gravity
      data_.a_ += data_.g_;

      // Conver acceleration to body Frame
      data_.a_b_ = data_.q_.inverse().toRotationMatrix() * data_.a_;
    }

  } else {

    // Wold referenced acceleration
    data_.a_.x() = data.a.x();
    data_.a_.y() = data.a.y();
    data_.a_.z() = data.a.z();

    if(!is_gravity_added_imu_) {

      // Sum gravity
      data_.a_ += data_.g_;
    }

    // Conver acceleration to body Frame
    data_.a_b_ = data_.q_.inverse().toRotationMatrix() * data_.a_;
  }

  if (!is_imu_noisy_) {

    // Random number generator
    std::random_device dev;
    std::mt19937 generator(dev());

    // White noise
    std::normal_distribution<double> white_noise_acc(0.0, imu_noise_values_[0]);
    std::normal_distribution<double> white_noise_gyro(0.0, imu_noise_values_[1]);

    // Random walk
    std::normal_distribution<double> random_walk_acc(0.0, imu_noise_values_[2]);
    std::normal_distribution<double> random_walk_gyro(0.0, imu_noise_values_[3]);

    // Imu biases
    imu_biases_(0) += random_walk_acc(generator);
    imu_biases_(1) += random_walk_acc(generator);
    imu_biases_(2) += random_walk_acc(generator);
    imu_biases_(3) += random_walk_gyro(generator);
    imu_biases_(4) += random_walk_gyro(generator);
    imu_biases_(5) += random_walk_gyro(generator);

    // Generate noisy Imu
    data_.a_b_.x() += white_noise_acc(generator) +  imu_biases_(0);
    data_.a_b_.y() += white_noise_acc(generator) +  imu_biases_(1);
    data_.a_b_.z() += white_noise_acc(generator) +  imu_biases_(2);
    data_.w_.x() += white_noise_gyro(generator) +  imu_biases_(3);
    data_.w_.y() += white_noise_gyro(generator) +  imu_biases_(4);
    data_.w_.z() += white_noise_gyro(generator) +  imu_biases_(5);
  }
}

void InputDataConverter::resetImuBiases() {
  imu_biases_.setZero();
}

const sensor_msgs::Imu &InputDataConverter::getNoisyImu(const double &timestamp) {

  noisy_imu_data_.header.stamp = ros::Time(timestamp);
  noisy_imu_data_.linear_acceleration.x = data_.a_b_.x();
  noisy_imu_data_.linear_acceleration.y = data_.a_b_.y();
  noisy_imu_data_.linear_acceleration.z = data_.a_b_.z();
  noisy_imu_data_.angular_velocity.x = data_.w_.x();
  noisy_imu_data_.angular_velocity.y = data_.w_.y();
  noisy_imu_data_.angular_velocity.z = data_.w_.z();
  noisy_imu_data_.orientation.w = data_.q_.w();
  noisy_imu_data_.orientation.x = data_.q_.x();
  noisy_imu_data_.orientation.y = data_.q_.y();
  noisy_imu_data_.orientation.z = data_.q_.z();

  return noisy_imu_data_;

}

const geometry_msgs::PoseStamped &InputDataConverter::getGroundTruth(const double &timestamp) {

  ground_truth_.header.stamp = ros::Time(timestamp);
  ground_truth_.header.frame_id = "world";
  ground_truth_.pose.position.x = data_.p_.x();
  ground_truth_.pose.position.y = data_.p_.y();
  ground_truth_.pose.position.z = data_.p_.z();
  ground_truth_.pose.orientation.w = data_.q_.w();
  ground_truth_.pose.orientation.x = data_.q_.x();
  ground_truth_.pose.orientation.y = data_.q_.y();
  ground_truth_.pose.orientation.z = data_.q_.z();

  return ground_truth_;

}

const geometry_msgs::TransformStamped &InputDataConverter::getTransform(const double &timestamp) {

  // This expresses a transform from coordinate frame header.frame_id
  // to the coordinate frame child_frame_id
  transform_.header.stamp = ros::Time(timestamp);
  transform_.header.frame_id = "world";
  transform_.child_frame_id = "uav/imu";
  transform_.transform.translation.x = data_.p_.x();
  transform_.transform.translation.y = data_.p_.y();
  transform_.transform.translation.z = data_.p_.z();
  transform_.transform.rotation.w = data_.q_.w();
  transform_.transform.rotation.x = data_.q_.x();
  transform_.transform.rotation.y = data_.q_.y();
  transform_.transform.rotation.z = data_.q_.z();

  return transform_;
}
