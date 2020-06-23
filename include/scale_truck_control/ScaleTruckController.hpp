/*
 * ScaleTruckController.h
 *
 *  Created on: June 2, 2020
 *      Author: Hyeongyu Lee
 *   Institute: KMU, Avees Lab
 */

#pragma once

//C++
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <boost/thread/thread.hpp>
#include <vector>

//ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

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
    float AngleMax_; // +degree
    float AngleMin_; // -degree
    float resultSpeed_;
    float centerErr_;

    //Thread

    std::thread controlThread_;

    std_msgs::Header imageHeader_;
    cv::Mat camImageCopy_;
    boost::shared_mutex mutexImageCallback_;

    bool imageStatus_ = false;
    boost::shared_mutex mutexImageStatus_;

    bool isNodeRunning_ = true;
    boost::shared_mutex mutexNodeStatus_;

    bool controlDone_ = false;
     
    bool isNodeRunning(void);

    bool getImageStatus(void);    

    void* lanedetectInThread();
    //void* objectdetectInThread();
};

} /* namespace scale_truck_control */
