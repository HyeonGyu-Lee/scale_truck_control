#include "scale_truck_control/ScaleTruckController.hpp"

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_), UDPsocket_() {
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}

ScaleTruckController::~ScaleTruckController() {
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }

  geometry_msgs::Twist msg;
  msg.angular.z = 0;
  msg.linear.x = 0;
  msg.linear.y = distance_;
  msg.linear.z = TargetDist_;
       
  ControlDataPublisher_.publish(msg);
  controlThread_.join();
  //udpsocketThread_.join();

  ROS_INFO("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, true);
  nodeHandle_.param("params/target_vel", TargetVel_, 0.5f); // m/s
  nodeHandle_.param("params/safety_vel", SafetyVel_, 0.3f); // m/s
  nodeHandle_.param("params/lv_stop_dist", LVstopDist_, 0.5f); // m
  nodeHandle_.param("params/fv_stop_dist", FVstopDist_, 0.5f); // m
  nodeHandle_.param("params/safety_dist", SafetyDist_, 1.5f); // m
  nodeHandle_.param("params/target_dist", TargetDist_, 0.8f); // m
  nodeHandle_.param("params/udp_group_addr", ADDR_, std::string("239.255.255.250"));
  nodeHandle_.param("params/udp_group_port", PORT_, 9307);
  nodeHandle_.param("params/truck_info", TRUCK_INFO_, std::string("LV"));
  nodeHandle_.param("params/Kp_d", Kp_d_, 2.0f);
  nodeHandle_.param("params/Ki_d", Ki_d_, 0.4f);

  if(!TRUCK_INFO_.compare(std::string("LV")))
    info_ = true;
  else
    info_ = false;

  return true;
}

void ScaleTruckController::init() {
  ROS_INFO("[ScaleTruckController] init()");
  
  struct timeval start;
  gettimeofday(&start, NULL);
  laneDetector_.start_ = start;

  std::string imageTopicName;
  int imageQueueSize;
  std::string objectTopicName;
  int objectQueueSize; 
  std::string velTopicName;
  int velQueueSize;
  std::string ControlDataTopicName;
  int ControlDataQueueSize;

  nodeHandle_.param("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size",imageQueueSize, 1);
  nodeHandle_.param("subscribers/obstacle_reading/topic", objectTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/obstacle_reading/queue_size",objectQueueSize, 100);
  nodeHandle_.param("subscribers/velocity_reading/topic", velTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/velocity_reading/queue_size",velQueueSize, 100);
  nodeHandle_.param("publishers/control_data/topic", ControlDataTopicName, std::string("twist_msg"));
  nodeHandle_.param("publishers/control_data/queue_size", ControlDataQueueSize, 1);

  imageSubscriber_ = nodeHandle_.subscribe(imageTopicName, imageQueueSize, &ScaleTruckController::imageCallback, this);
  objectSubscriber_ = nodeHandle_.subscribe(objectTopicName, objectQueueSize, &ScaleTruckController::objectCallback, this);
  velSubscriber_ = nodeHandle_.subscribe(velTopicName, velQueueSize, &ScaleTruckController::velCallback, this);
  ControlDataPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(ControlDataTopicName, ControlDataQueueSize);
 
  UDPsocket_.GROUP_ = ADDR_.c_str();
  UDPsocket_.PORT_ = PORT_;
  if(info_) // send
  {
    UDPsocket_.sendInit();
    printf("\n SendInit() \n");
  }
  else // receive
  {
    UDPsocket_.recvInit();
    printf("\n RecvInit() \n");
  }

  controlThread_ = std::thread(&ScaleTruckController::spin, this);
  const auto wait_udp = std::chrono::milliseconds(100);
  std::this_thread::sleep_for(wait_udp);
  //udpsocketThread_ = std::thread(&ScaleTruckController::UDPsocketInThread, this);
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
  float AngleDegree;
  laneDetector_.get_steer_coef(ResultVel_);
  AngleDegree = laneDetector_.display_img(camImageTmp, waitKeyDelay_, viewImage_);
  AngleDegree_ = AngleDegree;
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

  if(info_){	// LV velocity
	  if(distance_ <= LVstopDist_) {
	    ResultVel_ = 0.0f;
	  } else if(distance_ <= SafetyDist_) {
	    ResultVel_ = (TargetVel_-SafetyVel_)*((distance_-LVstopDist_)/(SafetyDist_-LVstopDist_))+SafetyVel_;
	  } else
	    ResultVel_ = TargetVel_;
  }
  else{		// FV velocity
	  float dist_err, P_err, I_err;
	  if((distance_ <= FVstopDist_) || (TargetVel_ <= 0.1f)){	// Emergency
		ResultVel_ = 0.0f;
	  }
	  else {
	  	dist_err = distance_ - TargetDist_;
	  	P_err = Kp_d_ * dist_err;
	  	I_err = Ki_d_ * dist_err * 0.1f;
	  	ResultVel_ = P_err + I_err + TargetVel_;
	  	if (ResultVel_ > 0.8f) ResultVel_ = 0.8f;	// Max velocity
	  }
  }
}

void* ScaleTruckController::UDPsocketInThread()
{
    udpData_ = 0;
    const auto wait_udp = std::chrono::milliseconds(33);
    std::this_thread::sleep_for(wait_udp);

    while(!controlDone_)
    {
        if(info_) // send
        {
          udpData_ = ResultVel_;
          //std::this_thread::sleep_for(wait_udp);
          UDPsocket_.sendData(udpData_);
        }
        else // receive
        {
          float udpData;
          UDPsocket_.recvData(&udpData);
          std::this_thread::sleep_for(wait_udp);
          udpData_ = udpData;
          TargetVel_ = udpData;
        }
        if(!isNodeRunning()) {
          controlDone_ = true;
        }
    } 
}

void ScaleTruckController::displayConsole() {
  printf("\033[2J");
  printf("\033[1;1H");
  printf("\nAngle           : %2.3f degree", AngleDegree_);
  printf("\nTar/Saf/Cur Vel : %3.3f / %3.3f / %3.3f m/s", TargetVel_, SafetyVel_, CurVel_);
  printf("\nTar/Saf/Cur Dist: %3.3f / %3.3f / %3.3f m", TargetDist_, SafetyDist_, distance_);
  printf("\nUDP_data        : %3.3f m/s", udpData_);
  printf("\nUDP_data        : %s", UDPsocket_.GROUP_);
  printf("\nUDP_data        : %d", UDPsocket_.PORT_);
  printf("\nK1/K2           : %3.3f / %3.3f", laneDetector_.K1_, laneDetector_.K2_);
  if(ObjCircles_ > 0) {
    printf("\nCirs            : %d", ObjCircles_);
    printf("\nDistAng         : %2.3f degree", distAngle_);
  }
  if(ObjSegments_ > 0) {
    printf("\nSegs            : %d", ObjSegments_);
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
     
    ControlDataPublisher_.publish(msg);
    if(!isNodeRunning()) {
      controlDone_ = true;
      ros::requestShutdown();
    } 
    //std::this_thread::sleep_for(wait_image);
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

void ScaleTruckController::velCallback(const std_msgs::Float32 &msg) {
  {
    boost::unique_lock<boost::shared_mutex> lockVelCallback(mutexVelCallback_);
    CurVel_ = msg.data;
  }
}

} /* namespace scale_truck_control */ 
