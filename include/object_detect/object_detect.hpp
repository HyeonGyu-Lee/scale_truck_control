#pragma once

//ROS
#include <sensor_msgs/LaserScan>
#include <ros/ros.h>

//C++
#include <iostream>

namespace object_detect {

class ObjectDetector{
  public:
    ObjectDetector(ros::NodeHandle nh);
    ~ObjectDetector(void);
  private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

};

}
