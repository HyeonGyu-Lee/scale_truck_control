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

void ScaleTruckController::spin() {
  geometry_msgs::Twist msg;
  ros::Rate loop_rate(60);
  int i = 0;
  while(ros::ok) {
    if((AngleDegree_ > AngleMax_) || (AngleDegree_ < AngleMin_))
      resultSpeed_ = 0.0f;
    
    AngleDegree_ = (centerLine_ - centerErr_)/centerErr_ * 90.0f; // -1 ~ 1 

    //TargetSpeed_ = resultSpeed_;

    msg.angular.z = AngleDegree_;
    msg.linear.x = TargetSpeed_;
    ControlDataPublisher_.publish(msg);
    
    if(enableConsoleOutput_ && (i++%10==0)) {
      printf("\033[2J");
      printf("\033[1;1H");
      printf("\nAngle  : %f degree", AngleDegree_);
      printf("\nSpeed  : %f m/s", TargetSpeed_);
      printf("\nCenter : %d\n", centerLine_);
    }
 
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void ScaleTruckController::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  Mat frame;
  try{
    frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8) -> image;
    centerLine_ = laneDetector_.display_img(frame, waitKeyDelay_, viewImage_);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception : %s", e.what());
  }
}

} /* namespace scale_truck_control */ 
