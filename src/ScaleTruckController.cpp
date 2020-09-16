#include "scale_truck_control/ScaleTruckController.hpp"

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_) {
  if (!readParameters()) {
    ros::requestShutdown();
  }

  centerLine_ = 0;

  init();
}

ScaleTruckController::~ScaleTruckController() {
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  controlThread_.join();
  ROS_INFO("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, true);
  nodeHandle_.param("params/target_speed", TargetSpeed_, 0.5f); // m/s
  nodeHandle_.param("params/safety_speed", SafetySpeed_, 0.3f); // m/s
  nodeHandle_.param("params/speed_mode", speed_mode_, 4); // m/s
  nodeHandle_.param("params/target_dist", TargetDist_, 0.3f); // m
  nodeHandle_.param("params/safety_dist", SafetyDist_, 1.0f); // m
  nodeHandle_.param("params/angle_degree", AngleDegree_, 0.0f); // degree
  nodeHandle_.param("params/angle_limits/max",AngleMax_, 60.0f);
  nodeHandle_.param("params/angle_limits/min",AngleMin_, -60.0f);
  nodeHandle_.param("params/center_err",centerErr_, 640.0f);
  return true;
}

void ScaleTruckController::init() {
  ROS_INFO("[ScaleTruckController] init()");
  
  controlThread_ = std::thread(&ScaleTruckController::spin, this);

  std::string imageTopicName;
  int imageQueueSize;
  std::string objectTopicName;
  int objectQueueSize;
  std::string ControlDataTopicName;
  int ControlDataQueueSize;

  nodeHandle_.param("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size",imageQueueSize, 1);
  nodeHandle_.param("subscribers/obstacle_reading/topic", objectTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/obstacle_reading/queue_size",objectQueueSize, 100);
  nodeHandle_.param("publishers/control_data/topic", ControlDataTopicName, std::string("twist_msg"));
  nodeHandle_.param("publishers/control_data/queue_size", ControlDataQueueSize, 1);

  imageSubscriber_ = nodeHandle_.subscribe(imageTopicName, imageQueueSize, &ScaleTruckController::imageCallback, this);
  objectSubscriber_ = nodeHandle_.subscribe(objectTopicName, objectQueueSize, &ScaleTruckController::objectCallback, this);
  ControlDataPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(ControlDataTopicName, ControlDataQueueSize);

}

bool ScaleTruckController::getImageStatus(void){
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool ScaleTruckController::isNodeRunning(void){
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void* ScaleTruckController::lanedetectInThread() {
  Mat camImageTmp = camImageCopy_.clone();
  centerLine_ = laneDetector_.display_img(camImageTmp, waitKeyDelay_, viewImage_);
  float weight = (centerLine_ - centerErr_)/centerErr_;
  AngleDegree_ = weight * AngleMax_; // -1 ~ 1 
}

void* ScaleTruckController::objectdetectInThread() {
  float dist; 
  ObjSegments_ = Obstacle_.segments.size();
  ObjCircles_ = Obstacle_.circles.size();
  distance_ = 10.f;
  for(int i = 0; i < ObjCircles_; i++){
    dist = sqrt(pow(Obstacle_.circles[i].center.x,2)+pow(Obstacle_.circles[i].center.y,2));
    if(distance_ >= dist)
      distance_ = dist;
  }
  float tmp = 1.0f / speed_mode_;
  if(distance_ <= TargetDist_) {
    resultSpeed_ = 0;
  } else if(distance_ <= SafetyDist_) {
    for(int i = 1; i <= speed_mode_; i++) {
      if(distance_ < (TargetDist_ + (dist_level_*i))) {
          resultSpeed_ = (TargetSpeed_ - SafetySpeed_)*tmp + SafetySpeed_;
        break;
      }
    }
  } else
    resultSpeed_ = TargetSpeed_;

}

void ScaleTruckController::displayConsole() {
  printf("\033[2J");
  printf("\033[1;1H");
  printf("\nAngle  : %f degree", AngleDegree_);
  printf("\nSpeed  : %f m/s", resultSpeed_);
  printf("\nCenter : %d", centerLine_);
  printf("\nDist   : %f", distance_);
  printf("\nMinDist: %f", TargetDist_);
  if(ObjSegments_ > 0)
    printf("\nSegs   : %d", ObjSegments_);
  if(ObjCircles_ > 0)
    printf("\nCirs   : %d", ObjCircles_);
  printf("\n");
}

void ScaleTruckController::spin() {
  const auto wait_duration = std::chrono::milliseconds(2000);
  while(!getImageStatus()) {
    printf("Waiting for image.\n");
    if(!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }
  
  geometry_msgs::Twist msg;
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;
  
  dist_level_ = (SafetyDist_ - TargetDist_)/speed_mode_;

  int i = 0;

  const auto wait_image = std::chrono::milliseconds(20);

  while(!controlDone_) {
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    lanedetect_thread.join();
    objectdetect_thread.join();
    
    if(enableConsoleOutput_)
      displayConsole();
 
    //if((AngleDegree_ > AngleMax_) || (AngleDegree_ < AngleMin_))
    //  resultSpeed_ = 0.0f;
    msg.angular.z = AngleDegree_;
    msg.linear.x = resultSpeed_;   
    ControlDataPublisher_.publish(msg);
    if(!isNodeRunning()) {
      controlDone_ = true;
    } 
    std::this_thread::sleep_for(wait_image);
  }
}

void ScaleTruckController::objectCallback(const obstacle_detector::Obstacles& msg) {
  {
    boost::unique_lock<boost::shared_mutex> lockObjectCallback(mutexObjectCallback_);
    Obstacle_ = msg;
  }
}

void ScaleTruckController::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception : %s", e.what());
  }

  if(cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = msg->header;
      camImageCopy_ = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
  }
}

} /* namespace scale_truck_control */ 
