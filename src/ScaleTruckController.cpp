#include "scale_truck_control/ScaleTruckController.hpp"

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_), UDPsend_(), UDPrecv_(), LRC_() {
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

  scale_truck_control::ctl msg;
  msg.index = Index_;
  msg.steer_angle = 0;
  msg.send_vel = 0;
  msg.cur_dist = distance_;
  msg.ref_dist = TargetDist_;

  ControlDataPublisher_.publish(msg);
  controlThread_.join();
  udprecvThread_.join();

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
  
  /**************/
  /* UDP Option */
  /**************/
  nodeHandle_.param("params/udp_group_addr", ADDR_, std::string("239.255.255.250"));
  nodeHandle_.param("params/udp_group_port", PORT_, 9307);
  nodeHandle_.param("params/truck_info", Index_, 0);

  return true;
}

void ScaleTruckController::init() {
  ROS_INFO("[ScaleTruckController] init()");  
  
  gettimeofday(&laneDetector_.start_, NULL);

  std::string imageTopicName;
  int imageQueueSize;
  std::string objectTopicName;
  int objectQueueSize; 
  std::string velTopicName;
  int velQueueSize;
  std::string ControlDataTopicName;
  int ControlDataQueueSize;
  std::string LanecoefTopicName;
  int LanecoefQueueSize;
  std::string LrcDataTopicName;
  int LrcDataQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  nodeHandle_.param("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size",imageQueueSize, 1);
  nodeHandle_.param("subscribers/obstacle_reading/topic", objectTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/obstacle_reading/queue_size",objectQueueSize, 100);
  nodeHandle_.param("subscribers/velocity_reading/topic", velTopicName, std::string("/vel_msg"));
  nodeHandle_.param("subscribers/velocity_reading/queue_size",velQueueSize, 100);
  
  /******************************/
  /* Ros Topic Publish Option */
  /******************************/
  nodeHandle_.param("publishers/control_data/topic", ControlDataTopicName, std::string("/ctl_msg"));
  nodeHandle_.param("publishers/control_data/queue_size", ControlDataQueueSize, 1);
  nodeHandle_.param("publishers/lane_coef/topic", LanecoefTopicName, std::string("/lane_msg"));
  nodeHandle_.param("publishers/lane_coef/queue_size", LanecoefQueueSize, 10);
  nodeHandle_.param("publishers/lrc_data/topic", LrcDataTopicName, std::string("/lrc_msg"));
  nodeHandle_.param("publishers/lrc_data/queue_size", LrcDataQueueSize, 1);

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  imageSubscriber_ = nodeHandle_.subscribe(imageTopicName, imageQueueSize, &ScaleTruckController::imageCallback, this);
  objectSubscriber_ = nodeHandle_.subscribe(objectTopicName, objectQueueSize, &ScaleTruckController::objectCallback, this);
  velSubscriber_ = nodeHandle_.subscribe(velTopicName, velQueueSize, &ScaleTruckController::velCallback, this);
  
  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  ControlDataPublisher_ = nodeHandle_.advertise<scale_truck_control::ctl>(ControlDataTopicName, ControlDataQueueSize);
  LanecoefPublisher_ = nodeHandle_.advertise<scale_truck_control::lane_coef>(LanecoefTopicName, LanecoefQueueSize);
  LrcDataPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc>(LrcDataTopicName, LrcDataQueueSize);
 
  /******************/
  /* UDP Multicast  */
  /******************/
  UDPsend_.GROUP_ = ADDR_.c_str();
  UDPsend_.PORT_ = PORT_;
  UDPsend_.sendInit();

  UDPrecv_.GROUP_ = ADDR_.c_str();
  UDPrecv_.PORT_ = PORT_;
  UDPrecv_.recvInit();

  /**********************************/
  /* Control & Communication Thread */
  /**********************************/
  controlThread_ = std::thread(&ScaleTruckController::spin, this);
  udprecvThread_ = std::thread(&ScaleTruckController::UDPrecvInThread, this);

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
    if(count == 0 && cam_failure_)
      cnt -= 1;
    else 
      cnt = 10;
  }
  float AngleDegree;
  camImageTmp_ = camImageCopy_.clone();
  laneDetector_.get_steer_coef(CurVel_);
  AngleDegree = laneDetector_.display_img(camImageTmp_, waitKeyDelay_, viewImage_);
  if(cnt == 0)
    AngleDegree_ = -distAngle_;
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
  
  if(!Index_){	
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

void* ScaleTruckController::UDPsendInThread()
{
    struct UDPsock::UDP_DATA udpData;

    udpData.index = Index_;
    udpData.to = 307;
    if(distance_ <= LVstopDist_ || TargetVel_ >= 2.0)
      udpData.target_vel = 0;
    else {
      udpData.target_vel = RefVel_;
    }
    udpData.current_vel = CurVel_;
    udpData.target_dist = TargetDist_;
    udpData.current_dist = distance_;
    udpData.current_angle = distAngle_;
    udpData.roi_dist = laneDetector_.distance_;
    udpData.coef[0].a = laneDetector_.lane_coef_.left.a;
    udpData.coef[0].b = laneDetector_.lane_coef_.left.b;
    udpData.coef[0].c = laneDetector_.lane_coef_.left.c;
    udpData.coef[1].a = laneDetector_.lane_coef_.right.a;
    udpData.coef[1].b = laneDetector_.lane_coef_.right.b;
    udpData.coef[1].c = laneDetector_.lane_coef_.right.c;
    udpData.coef[2].a = laneDetector_.lane_coef_.center.a;
    udpData.coef[2].b = laneDetector_.lane_coef_.center.b;
    udpData.coef[2].c = laneDetector_.lane_coef_.center.c;

    UDPsend_.sendData(udpData);
}

void* ScaleTruckController::UDPrecvInThread()
{
    struct UDPsock::UDP_DATA udpData;
	
    while(!controlDone_) { 
        UDPrecv_.recvData(&udpData);
        if(udpData.index == (Index_ - 1)) {
            udpData_.target_vel = udpData.target_vel;
            TargetVel_ = udpData_.target_vel;
            TargetDist_ = udpData_.target_dist;
        }
        if(udpData.index == 307) {
            if(udpData.to == Index_) {
                udpData_.index = udpData.index;
                udpData_.target_vel = udpData.target_vel;
                udpData_.target_dist = udpData.target_dist;
                udpData_.sync = udpData.sync;
				udpData_.cf = udpData.cf;
				sync_flag_ = udpData_.sync;
				
				{
				boost::shared_lock<boost::shared_mutex> lock(mutexCamStatus_);
				cam_failure_ = udpData_.cf;
				}
				
				TargetVel_ = udpData_.target_vel;
				TargetDist_ = udpData_.target_dist;
            }
        }
    }
}

void ScaleTruckController::displayConsole() {

  printf("\033[2J");
  printf("\033[1;1H");
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
  
  scale_truck_control::ctl msg;
  scale_truck_control::lane_coef lane;
  scale_truck_control::lrc lrc;
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;
  
  const auto wait_image = std::chrono::milliseconds(20);

  while(!controlDone_) {
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    udpsendThread_ = std::thread(&ScaleTruckController::UDPsendInThread, this);
    
    lanedetect_thread.join();
    objectdetect_thread.join();
    udpsendThread_.join();

    if(enableConsoleOutput_)
      displayConsole();

    msg.index = Index_;    
    msg.steer_angle = AngleDegree_;
    msg.send_vel = ResultVel_;
    msg.cur_dist = distance_;
    msg.ref_dist = TargetDist_;
    msg.sync = sync_flag_;
    msg.cf = cam_failure_;
    lane = laneDetector_.lane_coef_;
	LRC_.Save();
    ControlDataPublisher_.publish(msg);
    LanecoefPublisher_.publish(lane);
	LrcDataPublisher_.publish(lrc);

    if(!isNodeRunning()) {
      controlDone_ = true;
      ros::requestShutdown();
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

  if(cam_image && !cam_failure_) {
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

void ScaleTruckController::velCallback(const scale_truck_control::vel &msg) {
  {
    boost::unique_lock<boost::shared_mutex> lockVelCallback(mutexVelCallback_);
    CurVel_ = msg.cur_vel;
    RefVel_ = msg.ref_vel;
  }
}

} /* namespace scale_truck_control */ 
