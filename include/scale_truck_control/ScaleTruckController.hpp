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
#include <sys/time.h>

//ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <obstacle_detector/Obstacles.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

#include "lane_detect/lane_detect.hpp"
#include "sock_udp/sock_udp.hpp"

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
    void objectCallback(const obstacle_detector::Obstacles &msg);
    bool publishControlMsg(const geometry_msgs::Twist msg);

    ros::NodeHandle nodeHandle_;
    ros::Publisher ControlDataPublisher_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber objectSubscriber_;

    //image
    LaneDetect::LaneDetector laneDetector_;
    bool viewImage_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;

    float AngleDegree_; // -1 ~ 1  - Twist msg angular.z
    float TargetVel_; // -1 ~ 1  - Twist msg linear.x
    float SafetyVel_;
    float ResultVel_;

    //object
    int ObjSegments_;
    int ObjCircles_;
    float distance_;
    float distAngle_;
    float LVstopDist_;
    float FVstopDist_;
    float TargetDist_;
    float SafetyDist_;

    //Interval Control
    float Kp_d_;
    float Ki_d_;

    //UDP
    UDPsock::UDPsocket UDPsocket_;
    std::string ADDR_;
    std::string TRUCK_INFO_;
    bool info_ = true;
    int PORT_;
    float udpData_;

    //Thread
    std::thread controlThread_;
    std::thread udpsocketThread_;
    std::mutex mutex_;

    obstacle_detector::Obstacles Obstacle_;
    boost::shared_mutex mutexObjectCallback_;

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
    void* objectdetectInThread();
    void* UDPsocketInThread();
    void displayConsole();
};

} /* namespace scale_truck_control */
