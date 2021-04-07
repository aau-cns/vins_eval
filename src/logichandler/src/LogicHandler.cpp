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

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <logichandler/startstop.h>
#include <bag_recorder/Rosbag.h>
#include <flightgoggles/FGParam.h>

// Main function
int main(int argc, char** argv) {

  // Launch ros node
  ros::init(argc, argv, "logichandler");
  ros::NodeHandle nh("~");

  // Ros publisher
  ros::Publisher pub_start = nh.advertise<bag_recorder::Rosbag>("/start", 1);
  ros::Publisher pub_stop = nh.advertise<std_msgs::String>("/stop", 1);
  ros::Publisher pub_params = nh.advertise<flightgoggles::FGParam>("/uav/params", 1);

  std::cout << std::endl;
  ROS_INFO("Publishing: %s", pub_params.getTopic().c_str());
  std::cout << std::endl;

  // Ros messages
  bag_recorder::Rosbag start_control_msg;
  std_msgs::String stop_control_msg;
  flightgoggles::FGParam params_msg;

  // Control messages config
  start_control_msg.config = "outputdatahandler";
  stop_control_msg.data = "outputdatahandler";

  // Ros service
  ros::ServiceClient client = nh.serviceClient<logichandler::startstop>("/startstop_service");
  logichandler::startstop srv;

  // Get parameters
  int number_of_runs, attr, attr_lvl, img_width, img_height;
  float fisheye_dist_coeff, time_delay_s, fov;
  bool grayscale;
  std::string scene_name, illumination_changes_direction;

  if(!nh.getParam("number_of_runs", number_of_runs)) {
    std::cout << std::endl;
    ROS_ERROR("No number of runs defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("attribute", attr)) {
    std::cout << std::endl;
    ROS_ERROR("No attribute defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("attribute_level", attr_lvl)) {
    std::cout << std::endl;
    ROS_ERROR("No attribute level defined");
    std::exit(EXIT_FAILURE);
  }
  --attr_lvl; //Map from [1,N] to [0,N-1]

  if(!nh.getParam("image_width", img_width)) {
    std::cout << std::endl;
    ROS_ERROR("No image width defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("image_height", img_height)) {
    std::cout << std::endl;
    ROS_ERROR("No image height defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("camera_fov", fov)) {
    std::cout << std::endl;
    ROS_ERROR("No camera FOV defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("grayscale", grayscale)) {
    std::cout << std::endl;
    ROS_ERROR("No color defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("fisheye", fisheye_dist_coeff)) {
    std::cout << std::endl;
    ROS_ERROR("No fisheye distortion coefficient defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("time_delay_s", time_delay_s)) {
    std::cout << std::endl;
    ROS_ERROR("No time delay between camera and imu defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("scene_name", scene_name)) {
    std::cout << std::endl;
    ROS_ERROR("No scene name defined");
    std::exit(EXIT_FAILURE);
  }

  if(!nh.getParam("illumination_changes_direction", illumination_changes_direction)) {
    std::cout << std::endl;
    ROS_ERROR("No illumination changes direction defined");
    std::exit(EXIT_FAILURE);
  }

  // the actual attribute and attribute level that should not change over the number_of_runs
  // the randomness flag to allow changing object location / textures / illumination / object size / ...
  // trajectories are already different each others.

  // Set fixed params
  params_msg.header.stamp = ros::Time::now();
  params_msg.sim_scene_name = scene_name;
  params_msg.sim_cam_dimensions.x = img_width;
  params_msg.sim_cam_dimensions.y = img_height;
  params_msg.cam_is_grayscale = grayscale;
  params_msg.sim_cam_fov = fov;
  params_msg.illumination_changes_direction = illumination_changes_direction;

  // Check what is the actual evaluated parameter and set the evaluated parameter at the defined level
  // and the others to the optimal (=0) level, pay attention that imu noise is not managed here but
  // it is managed into InputDataHandle.

  // Dictionary:
  // 1 = features
  // 2 = illumination
  // 3 = clutter
  // 4 = motionblur
  // 5 = imunoise
  // 6 = timedelay
  // 7 = fisheye
  switch(attr)  {
  case 1:
    params_msg.level_features = attr_lvl;
    params_msg.level_illumination = 0.0;
    params_msg.level_clutter = 0.0;
    params_msg.level_motion_blur = 0.0;
    params_msg.cam_time_delay = time_delay_s;
    params_msg.cam_fisheye_distortion = fisheye_dist_coeff;
    break;
  case 2:
    params_msg.level_features = 0.0;
    params_msg.level_illumination = attr_lvl;
    params_msg.level_clutter = 0.0;
    params_msg.level_motion_blur = 0.0;
    params_msg.cam_time_delay = time_delay_s;
    params_msg.cam_fisheye_distortion = fisheye_dist_coeff;
    break;
  case 3:
    params_msg.level_features = 0.0;
    params_msg.level_illumination = 0.0;
    params_msg.level_clutter = attr_lvl;
    params_msg.level_motion_blur = 0.0;
    params_msg.cam_time_delay = time_delay_s;
    params_msg.cam_fisheye_distortion = fisheye_dist_coeff;
    break;
  case 4:
    params_msg.level_features = 0.0;
    params_msg.level_illumination = 0.0;
    params_msg.level_clutter = 0.0;
    params_msg.level_motion_blur = attr_lvl;
    params_msg.cam_time_delay = time_delay_s;
    params_msg.cam_fisheye_distortion = fisheye_dist_coeff;
    break;
  case 5:
    params_msg.level_features = 0.0;
    params_msg.level_illumination = 0.0;
    params_msg.level_clutter = 0.0;
    params_msg.level_motion_blur = attr_lvl;
    params_msg.cam_time_delay = time_delay_s;
    params_msg.cam_fisheye_distortion = fisheye_dist_coeff;
    break;
  case 6:
  case 7:
    params_msg.level_features = 0.0;
    params_msg.level_illumination = 0.0;
    params_msg.level_clutter = 0.0;
    params_msg.level_motion_blur = 0.0;
    params_msg.cam_time_delay = time_delay_s;
    params_msg.cam_fisheye_distortion = fisheye_dist_coeff;
    break;
  }

  // Send to FG scene parameters and sleep to let FG get ready
  ros::Duration(1).sleep();
  ROS_INFO("Send to FG scene parameters");

  // Loop through the number of runs
  for (int run = 0; run < number_of_runs; ++run) {

    // Publish params message to trigger randmoness and scene re-init
    pub_params.publish(params_msg);

    // Sleep to let FG be ready
    ros::Duration(1).sleep();

    // Publish start request for data recording and service
    // request to start input data generation using services
    // here is perfect because they block the execution until
    // the requested service has been completed
    srv.request.start = true;

    ROS_INFO("Start data generation and recording");
    // Publish start message
    start_control_msg.header.stamp = ros::Time::now();
    // naming convention: <attribute>_<attribute_level>_<run>.bag
    start_control_msg.bag_name = "INPUT_ATTR_" + std::to_string(attr) + "_" + "LVL_" + std::to_string(attr_lvl+1) + "_" + "RUN_" + std::to_string(run+1) + ".bag";
    pub_start.publish(start_control_msg);

    // Call service request
    if (client.call(srv)) {

      // Check responce
      if(srv.response.stop) {
        pub_stop.publish(stop_control_msg);
      }

      ROS_INFO("Data correctly generated and recorded");
    }
    else {
      ROS_ERROR("Failed to generate and record data!");
    }

    ros::spinOnce();
  }

  // Done!
  std::cout << std::endl;
  ROS_INFO("Done!");
  return EXIT_SUCCESS;

}
