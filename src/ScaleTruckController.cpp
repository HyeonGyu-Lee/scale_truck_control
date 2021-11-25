#include "scale_truck_control/ScaleTruckController.hpp"

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_), ZMQ_SOCKET_(nh){
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

  scale_truck_control::xav2lrc msg;
  msg.steer_angle = 0;
  msg.cur_dist = distance_;
  msg.tar_vel = ResultVel_;	//Xavier to LRC and LRC to OpenCR
  msg.tar_dist = TargetDist_;
  msg.beta = Beta_;
  msg.gamma = Gamma_;

  XavPublisher_.publish(msg);
  controlThread_.join();

  ROS_INFO("[ScaleTruckController] Stop.");
}

bool ScaleTruckController::readParameters() {
  /***************/
  /* View Option */
  /***************/
  nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, true);
  
  /*******************/
  /* Velocity Option */
  /*******************/
  nodeHandle_.param("params/target_vel", TargetVel_, 0.5f); // m/s
  nodeHandle_.param("params/safety_vel", SafetyVel_, 0.3f); // m/s
  nodeHandle_.param("params/fv_max_vel", FVmaxVel_, 0.8f); // m/s
  nodeHandle_.param("params/ref_vel", RefVel_, 0.0f); // m/s
  
  /*******************/
  /* Distance Option */
  /*******************/
  nodeHandle_.param("params/lv_stop_dist", LVstopDist_, 0.5f); // m
  nodeHandle_.param("params/fv_stop_dist", FVstopDist_, 0.5f); // m
  nodeHandle_.param("params/safety_dist", SafetyDist_, 1.5f); // m
  nodeHandle_.param("params/target_dist", TargetDist_, 0.8f); // m

  return true;
}

void ScaleTruckController::init() {
  ROS_INFO("[ScaleTruckController] init()");  
  
  gettimeofday(&laneDetector_.start_, NULL);
  
  std::string imageTopicName;
  int imageQueueSize;
  std::string objectTopicName;
  int objectQueueSize; 
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string XavPubTopicName;
  int XavPubQueueSize;
  std::string LanecoefTopicName;
  int LanecoefQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  nodeHandle_.param("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", imageQueueSize, 1);
  nodeHandle_.param("subscribers/obstacle_reading/topic", objectTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/obstacle_reading/queue_size", objectQueueSize, 100);
  nodeHandle_.param("lrcSubPub/lrc_to_xavier/topic", XavSubTopicName, std::string("/lrc2xav_msg"));
  nodeHandle_.param("lrcSubPub/lrc_to_xavier/queue_size", XavSubQueueSize, 1);
  
  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  nodeHandle_.param("publishers/lane_coef/topic", LanecoefTopicName, std::string("/lane_msg"));
  nodeHandle_.param("publishers/lane_coef/queue_size", LanecoefQueueSize, 10);
  nodeHandle_.param("lrcSubPub/xavier_to_lrc/topic", XavPubTopicName, std::string("/xav2lrc_msg"));
  nodeHandle_.param("lrcSubPub/xavier_to_lrc/queue_size", XavPubQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  imageSubscriber_ = nodeHandle_.subscribe(imageTopicName, imageQueueSize, &ScaleTruckController::imageCallback, this);
  objectSubscriber_ = nodeHandle_.subscribe(objectTopicName, objectQueueSize, &ScaleTruckController::objectCallback, this);
  XavSubscriber_ = nodeHandle_.subscribe(XavSubTopicName, XavSubQueueSize, &ScaleTruckController::XavSubCallback, this);
  
  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = nodeHandle_.advertise<scale_truck_control::xav2lrc>(XavPubTopicName, XavPubQueueSize);
  LanecoefPublisher_ = nodeHandle_.advertise<scale_truck_control::lane_coef>(LanecoefTopicName, LanecoefQueueSize);



  /**********************************/
  /* Control & Communication Thread */
  /**********************************/
  controlThread_ = std::thread(&ScaleTruckController::spin, this);

  /**********************/
  /* Safety Start Setup */
  /**********************/
  distance_ = 10.f;
  distAngle_ = 0;
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
  static int cnt = 10;
  Mat dst;
  std::vector<Mat>channels;
  int count = 0;
  if((!camImageTmp_.empty()) && (cnt != 0) && (TargetVel_ != 0))
  {
    bitwise_xor(camImageCopy_,camImageTmp_, dst);
    split(dst, channels);
    for(int ch = 0; ch<dst.channels();ch++) {
      count += countNonZero(channels[ch]);
    }
    if(count == 0 && Beta_)
      cnt -= 1;
    else 
      cnt = 10;
  }
  float AngleDegree;
  camImageTmp_ = camImageCopy_.clone();
  laneDetector_.get_steer_coef(CurVel_);
  AngleDegree = laneDetector_.display_img(camImageTmp_, waitKeyDelay_, viewImage_);
  if(cnt == 0){
    AngleDegree_ = -distAngle_;
  }
  else
    AngleDegree_ = AngleDegree;
}

void* ScaleTruckController::objectdetectInThread() {
  float dist, angle; 
  float dist_tmp, angle_tmp;
  ObjSegments_ = Obstacle_.segments.size();
  ObjCircles_ = Obstacle_.circles.size();
   
  dist_tmp = 10.f; 
  /**************/
  /* Lidar Data */
  /**************/
  for(int i = 0; i < ObjCircles_; i++)
  {
    //dist = sqrt(pow(Obstacle_.circles[i].center.x,2)+pow(Obstacle_.circles[i].center.y,2));
    dist = -Obstacle_.circles[i].center.x - Obstacle_.circles[i].true_radius;
    angle = atanf(Obstacle_.circles[i].center.y/Obstacle_.circles[i].center.x)*(180.0f/M_PI);
    if(dist_tmp >= dist) {
      dist_tmp = dist;
      angle_tmp = angle;
    }
  }
  if(ObjCircles_ != 0)
  {
    distance_ = dist_tmp;
    distAngle_ = angle_tmp;
  }
  /*****************************/
  /* Dynamic ROI Distance Data */
  /*****************************/
  if(dist_tmp < 1.24 && dist_tmp > 0.30) // 1.26 ~ 0.28
  {
    laneDetector_.distance_ = (int)((1.24 - dist_tmp)*490.0);
  } else {
    laneDetector_.distance_ = 0;
  }
  
  if(ZMQ_SOCKET_.zipcode_.compare(std::string("00000"))){	
      /***************/
      /* LV velocity */
      /***************/
	  if(distance_ <= LVstopDist_) {
		// Emergency Brake
	    ResultVel_ = 0.0f;
	  }
	  else if (distance_ <= SafetyDist_){
	    float TmpVel_ = (ResultVel_-SafetyVel_)*((distance_-LVstopDist_)/(SafetyDist_-LVstopDist_))+SafetyVel_;
		if (TargetVel_ < TmpVel_){
			ResultVel_ = TargetVel_;
		}
		else{
			ResultVel_ = TmpVel_;
		}
	  }
	  else{
		ResultVel_ = TargetVel_;
	  }
  }
  else{
      /***************/
      /* FV velocity */
      /***************/
	  if ((distance_ <= FVstopDist_) || (TargetVel_ <= 0.1f)){
		// Emergency Brake
		ResultVel_ = 0.0f;
	  } else {
		ResultVel_ = TargetVel_;
	  }
  }
}

void ScaleTruckController::displayConsole() {
  static std::string ipAddr = ZMQ_SOCKET_.getIPAddress();

  printf("\033[2J");
  printf("\033[1;1H");
  printf("%s (%s) - %s\n","-Client", ipAddr.c_str() , ZMQ_SOCKET_.udp_ip_.c_str());
  printf("\nAngle           : %2.3f degree", AngleDegree_);
  printf("\nRefer Vel       : %3.3f m/s", RefVel_);
  printf("\nSend Vel        : %3.3f m/s", ResultVel_);
  printf("\nTar/Cur Vel     : %3.3f / %3.3f m/s", TargetVel_, CurVel_);
  printf("\nTar/Cur Dist    : %3.3f / %3.3f m", TargetDist_, distance_);
  printf("\nK1/K2           : %3.3f / %3.3f", laneDetector_.K1_, laneDetector_.K2_);
  if(ObjCircles_ > 0) {
    printf("\nCirs            : %d", ObjCircles_);
    printf("\nDistAng         : %2.3f degree", distAngle_);
  }
  if(ObjSegments_ > 0) {
    printf("\nSegs            : %d", ObjSegments_);
  }
  printf("\nCycle Time      : %3.3f ms", CycleTime_);
  printf("\n");
}

void ScaleTruckController::spin() {
  double diff_time=0.0;
  int cnt = 0;
  
  const auto wait_duration = std::chrono::milliseconds(2000);
  while(!getImageStatus()) {
    printf("Waiting for image.\n");
    if(!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }
  
  scale_truck_control::xav2lrc msg;
  scale_truck_control::lane_coef lane;
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;
  
  const auto wait_image = std::chrono::milliseconds(20);

  static int zipcode, recv_sub, send_req, recv_req, send_rad, recv_dsh;
  std::istringstream iss;

  float TargetVel, TargetDist;

  while(!controlDone_ && ros::ok()) {
    struct timeval start_time, end_time;
    gettimeofday(&start_time, NULL);
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    
    lanedetect_thread.join();
    objectdetect_thread.join();

    ZMQ_SOCKET_.send_req_ = boost::str(boost::format("%s %f %f") % ZMQ_SOCKET_.zipcode_.c_str() % TargetVel_ % TargetDist_);
    
    TargetDist = TargetDist_;
    if(distance_ <= LVstopDist_ || TargetVel_ >= 2.0) {
      TargetVel = 0;
    }
    else {
      TargetVel = RefVel_;   
    }
    
    ZMQ_SOCKET_.send_rad_ = boost::str(boost::format("%s %f %f") % ZMQ_SOCKET_.zipcode_.c_str() % TargetVel % TargetDist);
     
    iss = std::istringstream(ZMQ_SOCKET_.recv_sub_);
    iss >> zipcode >> recv_sub;
    iss = std::istringstream(ZMQ_SOCKET_.recv_req_);
    iss >> zipcode >> recv_sub;
    iss = std::istringstream(ZMQ_SOCKET_.recv_dsh_);
    iss >> zipcode >> TargetVel_ >> TargetDist_;

    if(enableConsoleOutput_)
      displayConsole();

    msg.steer_angle = AngleDegree_;
    msg.cur_dist = distance_;
    msg.tar_vel = ResultVel_;	//Xavier to LRC and LRC to OpenCR
    msg.tar_dist = TargetDist_;
    msg.beta = Beta_;
    msg.gamma = Gamma_;

    lane = laneDetector_.lane_coef_;
    LanecoefPublisher_.publish(lane);
    XavPublisher_.publish(msg);

    if(!isNodeRunning()) {
      controlDone_ = true;
      ZMQ_SOCKET_.controlDone_ = true;
      ros::requestShutdown();
    }

    gettimeofday(&end_time, NULL);
    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0) + ((end_time.tv_usec - start_time.tv_usec) / 1000.0);
    cnt++;

    CycleTime_ = diff_time / (double)cnt;

    printf("cnt: %d\n", cnt);
    if (cnt > 3000){
	    diff_time = 0.0;
	    cnt = 0;
    }
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

  if(cam_image && !Beta_) {
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

void ScaleTruckController::XavSubCallback(const scale_truck_control::lrc2xav &msg){
  {
    boost::unique_lock<boost::shared_mutex> lockVelCallback(mutexVelCallback_);
    CurVel_ = msg.cur_vel;
  }	
}

} /* namespace scale_truck_control */ 
