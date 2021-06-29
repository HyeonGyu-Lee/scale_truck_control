#include <object_detect/object_detect.hpp>

namespace object_detect {

ObjectDetector::ObjectDetector(ros::NodeHandle nh){

}

ObjectDetector::~ObjectDetector(void){

}

ObjectDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  int cnt = scan->scan_time / scan->/time_increment;
  
  for(int i = 0; i < cnt; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    
  }
}

}
