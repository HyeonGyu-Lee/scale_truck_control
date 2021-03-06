#include "scale_truck_control/ScaleTruckController.hpp"

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_) {
  if (!readParameters()) {
    ros::requestShutdown();
  }

  i_points_ = NULL;

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
  nodeHandle_.param("params/target_vel", TargetVel_, 0.5f); // m/s
  nodeHandle_.param("params/safety_vel", SafetyVel_, 0.3f); // m/s
  nodeHandle_.param("params/target_dist", TargetDist_, 0.3f); // m
  nodeHandle_.param("params/safety_dist", SafetyDist_, 1.0f); // m
  nodeHandle_.param("params/angle_degree", AngleDegree_, 0.0f); // degree
  nodeHandle_.param("LaneDetector/K1",K1_, 0.06f);
  nodeHandle_.param("LaneDetector/K2",K2_, 0.06f);
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
  i_points_ = laneDetector_.display_img(camImageTmp, waitKeyDelay_, viewImage_);
  AngleDegree_ = ((-1.0f * K1_) * i_points_[1]) + ((-1.0f * K2_) * i_points_[0]);
}

void* ScaleTruckController::objectdetectInThread() {
  float dist, angle; 
  ObjSegments_ = Obstacle_.segments.size();
  ObjCircles_ = Obstacle_.circles.size();
  distance_ = 10.f;
  
  for(int i = 0; i < ObjCircles_; i++){
    dist = sqrt(pow(Obstacle_.circles[i].center.x,2)+pow(Obstacle_.circles[i].center.y,2));
    angle = atanf(Obstacle_.circles[i].center.y/Obstacle_.circles[i].center.x)*(180.0f/M_PI);
    if(distance_ >= dist) {
      distance_ = dist;
      distAngle_ = angle;
    }
  }
  
  if(distance_ <= TargetDist_) {
    ResultVel_ = 0;
  } else if(distance_ <= SafetyDist_) {
    ResultVel_ = (TargetVel_-SafetyVel_)*((distance_-TargetDist_)/(SafetyDist_-TargetDist_))+SafetyVel_;
  } else
    ResultVel_ = TargetVel_;

}

void ScaleTruckController::displayConsole() {
  printf("\033[2J");
  printf("\033[1;1H");
  printf("\nAngle           : %2.3f degree", AngleDegree_);
  printf("\nTar/Saf/Cur Vel : %3.3f / %3.3f / %3.3f m/s", TargetVel_, SafetyVel_, ResultVel_);
  printf("\nTar/Saf/Cur Dist: %3.3f / %3.3f / %3.3f m/s", TargetDist_, SafetyDist_, distance_);
  printf("\nPreview Distance Error: %2f", i_points_[0]);
  printf("\nLateral Position Error: %2f", i_points_[1]);
  if(ObjCircles_ > 0) {
    printf("\nCirs   : %d", ObjCircles_);
    printf("\nDistAng: %2.3f", distAngle_);
  }
  if(ObjSegments_ > 0) {
    printf("\nSegs   : %d", ObjSegments_);
  }
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

  const auto wait_image = std::chrono::milliseconds(20);

  while(!controlDone_) {
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    lanedetect_thread.join();
    objectdetect_thread.join();
    
    if(enableConsoleOutput_)
      displayConsole();
 
    msg.angular.z = AngleDegree_;
    msg.linear.x = ResultVel_;
    msg.linear.y = distance_;
    msg.linear.z = TargetDist_;
    
    if (!readParameters()) {
    ros::requestShutdown();
    }
    
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
