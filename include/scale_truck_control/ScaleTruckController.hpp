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
#include <boost/thread/thread.hpp>
#include <vector>
#include <sys/time.h>
#include <string>

//ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <obstacle_detector/Obstacles.h>

//OpenCV
#include <cv_bridge/cv_bridge.h>

#include "lane_detect/lane_detect.hpp"
#include "zmq_class/zmq_class.h"

//custom msgs
#include <scale_truck_control/lrc2xav.h>
#include <scale_truck_control/xav2lrc.h>

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
    void XavSubCallback(const scale_truck_control::lrc2xav &msg);
    //bool publishControlMsg(const scale_truck_control::ctl msg);

    ros::NodeHandle nodeHandle_;
    ros::Publisher XavPublisher_;
    ros::Publisher LanecoefPublisher_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber objectSubscriber_;
    ros::Subscriber XavSubscriber_;
	
    double CycleTime_ = 0.0;
    //image
    LaneDetect::LaneDetector laneDetector_;
    bool viewImage_;
    int waitKeyDelay_;
    bool enableConsoleOutput_;
    int sync_flag_;
    bool Beta_ = false;

    float AngleDegree_; // -1 ~ 1  - Twist msg angular.z
    float TargetVel_; // -1 ~ 1  - Twist msg linear.x
    float SafetyVel_;
    float ResultVel_;
    float FVmaxVel_;

    //object
    int ObjSegments_;
    int ObjCircles_;
    float distance_;
    float distAngle_;
    float LVstopDist_;
    float FVstopDist_;
    float TargetDist_;
    float SafetyDist_;
    bool Gamma_ = false;

    //ZMQ
    ZMQ_CLASS ZMQ_SOCKET_;

    //Thread
    std::thread controlThread_;
    std::mutex mutex_;

    obstacle_detector::Obstacles Obstacle_;
    boost::shared_mutex mutexObjectCallback_;

    std_msgs::Header imageHeader_;
    cv::Mat camImageCopy_, camImageTmp_;
    boost::shared_mutex mutexImageCallback_;

    float CurVel_;
    float RefVel_;
    boost::shared_mutex mutexVelCallback_;

    bool imageStatus_ = false;
    boost::shared_mutex mutexImageStatus_;

    bool isNodeRunning_ = true;
    boost::shared_mutex mutexNodeStatus_;
	
    //bool cam_failure_ = false;
    boost::shared_mutex mutexCamStatus_;

    bool controlDone_ = false;
     
    bool isNodeRunning(void);
    bool getImageStatus(void);
	
    void* lanedetectInThread();
    void* objectdetectInThread();
    void* UDPsendInThread();
    void* UDPrecvInThread();
    void displayConsole();
};

} /* namespace scale_truck_control */
