/*
 * ScaleTruckController.h
 *
 *  Created on: June 2, 2020
 *      Auther: Hyeongyu Lee
 *   Institute: KMU, Avees Lab
 */

#pragma once

//
//ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

#include "lane_detect/lane_detect.h"

namespace scale_truck_control {

class ScaleTruckController {
  public:
    explicit ScaleTruckController(ros::NodeHandle nh);

    ~ScaleTruckController();

    void spin();
  private:
    bool readParameters();

    void init();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

    bool publishControlMsg(const geometry_msgs::Twist msg);

    ros::NodeHandle nodeHandle_;
    ros::Publisher ControlDataPublisher_;
    ros::Subscriber imageSubscriber_;
    
    lane_detect::LaneDetector laneDetector_;
    bool viewImage_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;

    float TargetSpeed_; // -1 ~ 1  - Twist msg linear.x
    float AngleDegree_; // -1 ~ 1  - Twist msg angular.z
    int centerLine_;
};

} /* namespace scale_truck_control */
