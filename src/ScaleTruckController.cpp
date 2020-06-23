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
  nodeHandle_.param("params/target_speed", TargetSpeed_, 1.0f); // m/s
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
  std::string ControlDataTopicName;
  int ControlDataQueueSize;

  nodeHandle_.param("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size",imageQueueSize, 1);
  nodeHandle_.param("publishers/control_data/topic", ControlDataTopicName, std::string("twist_msg"));
  nodeHandle_.param("publishers/control_data/queue_size", ControlDataQueueSize, 1);

  imageSubscriber_ = nodeHandle_.subscribe(imageTopicName, imageQueueSize, &ScaleTruckController::imageCallback, this);
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
    geometry_msgs::Twist msg;
    centerLine_ = laneDetector_.display_img(camImageCopy_, waitKeyDelay_, viewImage_);
    AngleDegree_ = (centerLine_ - centerErr_)/centerErr_ * 90.0f; // -1 ~ 1 
    if((AngleDegree_ > AngleMax_) || (AngleDegree_ < AngleMin_))
      resultSpeed_ = 0.0f;
    else
      resultSpeed_ = TargetSpeed_;

    msg.angular.z = AngleDegree_;
    msg.linear.x = resultSpeed_;   
    if(enableConsoleOutput_) {
      printf("\033[2J");
      printf("\033[1;1H");
      printf("\nAngle  : %f degree", AngleDegree_);
      printf("\nSpeed  : %f m/s", resultSpeed_);
      printf("\nCenter : %d\n", centerLine_);
    }

    ControlDataPublisher_.publish(msg);
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
  //std::thread objectdetect_thread;

  int i = 0;
 
  while(!controlDone_) {
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    //objectdetect_thread = std::thread(&ScaleTruckControl::objectdetectInThread, this);
    
    lanedetect_thread.join();
    //objectdetect_thread.join();
    if(!isNodeRunning()) {
      controlDone_ = true;
    } 
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
