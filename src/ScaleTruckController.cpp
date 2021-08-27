#include "scale_truck_control/ScaleTruckController.hpp"

namespace scale_truck_control{

ScaleTruckController::ScaleTruckController(ros::NodeHandle nh)
    : nodeHandle_(nh), laneDetector_(nodeHandle_), UDPsend_(), UDPrecv_() {
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
  udprecvThread_.join();

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
  nodeHandle_.param("params/truck_info", Index_, 0);
  nodeHandle_.param("params/Kp_d", Kp_d_, 2.0f);
  nodeHandle_.param("params/Ki_d", Ki_d_, 0.4f);

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
  std::string LanecoefTopicName;
  int LanecoefQueueSize;

  nodeHandle_.param("subscribers/camera_reading/topic", imageTopicName, std::string("/usb_cam/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size",imageQueueSize, 1);
  nodeHandle_.param("subscribers/obstacle_reading/topic", objectTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/obstacle_reading/queue_size",objectQueueSize, 100);
  nodeHandle_.param("subscribers/velocity_reading/topic", velTopicName, std::string("/raw_obstacles"));
  nodeHandle_.param("subscribers/velocity_reading/queue_size",velQueueSize, 100);
  nodeHandle_.param("publishers/control_data/topic", ControlDataTopicName, std::string("twist_msg"));
  nodeHandle_.param("publishers/control_data/queue_size", ControlDataQueueSize, 1);
  nodeHandle_.param("publishers/lane_coef/topic", LanecoefTopicName, std::string("lane_msg"));
  nodeHandle_.param("publishers/lane_coef/queue_size", LanecoefQueueSize, 10);

  imageSubscriber_ = nodeHandle_.subscribe(imageTopicName, imageQueueSize, &ScaleTruckController::imageCallback, this);
  objectSubscriber_ = nodeHandle_.subscribe(objectTopicName, objectQueueSize, &ScaleTruckController::objectCallback, this);
  velSubscriber_ = nodeHandle_.subscribe(velTopicName, velQueueSize, &ScaleTruckController::velCallback, this);
  ControlDataPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(ControlDataTopicName, ControlDataQueueSize);
  LanecoefPublisher_ = nodeHandle_.advertise<scale_truck_control::lane_coef>(LanecoefTopicName, LanecoefQueueSize);
 
  UDPsend_.GROUP_ = ADDR_.c_str();
  UDPsend_.PORT_ = PORT_;
  UDPsend_.sendInit();

  UDPrecv_.GROUP_ = ADDR_.c_str();
  UDPrecv_.PORT_ = PORT_;
  UDPrecv_.recvInit();

  controlThread_ = std::thread(&ScaleTruckController::spin, this);
  udpsendThread_ = std::thread(&ScaleTruckController::UDPsendInThread, this);
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
      if(dist < 1.25)
      {
        double height;
        laneDetector_.distance_ = (int)(480*(1.0 - (dist)/1.));
      }
      else
        laneDetector_.distance_ = 0;
    }
  }

  if(!Index_){	// LV velocity
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

void* ScaleTruckController::UDPsendInThread()
{
    struct UDPsock::UDP_DATA udpData;
    
    udpData.index = Index_;
    udpData.to = 307;
    udpData.target_vel = ResultVel_;
    udpData.current_vel = CurVel_;
    udpData.target_dist = TargetDist_;
    udpData.current_dist = distance_;

    UDPsend_.sendData(udpData);
}

void* ScaleTruckController::UDPrecvInThread()
{
    struct UDPsock::UDP_DATA udpData;
    UDPrecv_.recvData(&udpData);
    //std::this_thread::sleep_for(wait_udp);
   
    if(udpData.index == (Index_ - 1)) {
        TargetVel_ = udpData_.target_vel;
    }
    if(udpData.index == 307) {
        if(udpData.to == Index_) {
            udpData_.index = udpData.index;
            udpData_.target_vel = udpData.target_vel;
            udpData_.target_dist = udpData.target_dist;

            TargetVel_ = udpData_.target_vel;
            TargetDist_ = udpData_.target_dist;
        }
    }
}

void ScaleTruckController::displayConsole() {
  printf("\033[2J");
  printf("\033[1;1H");
  printf("\nAngle           : %2.3f degree", AngleDegree_);
  printf("\nTar/Saf/Cur Vel : %3.3f / %3.3f / %3.3f m/s", TargetVel_, ResultVel_, CurVel_);
  printf("\nTar/Saf/Cur Dist: %3.3f / %3.3f / %3.3f m", TargetDist_, SafetyDist_, distance_);
  printf("\nUDP_data        : %d (LV:0,FV1:1,FV2:2,CMD:307)", udpData_.index);
  printf("\nUDP_target_vel  : %3.3lf", udpData_.target_vel);
  printf("\nUDP_target_dist : %3.3lf", udpData_.target_dist);

  printf("\n%3.6f %3.6f %3.6f",laneDetector_.lane_coef_.center.a, laneDetector_.lane_coef_.center.b, laneDetector_.lane_coef_.center.c);
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
  scale_truck_control::lane_coef lane;
  std::thread lanedetect_thread;
  std::thread objectdetect_thread;
  
  const auto wait_image = std::chrono::milliseconds(20);

  while(!controlDone_) {
    lanedetect_thread = std::thread(&ScaleTruckController::lanedetectInThread, this);
    objectdetect_thread = std::thread(&ScaleTruckController::objectdetectInThread, this);
    udprecvThread_ = std::thread(&ScaleTruckController::UDPrecvInThread, this);
    
    lanedetect_thread.join();
    objectdetect_thread.join();
    udprecvThread_.join();

    if(enableConsoleOutput_)
      displayConsole();
	
    msg.angular.z = AngleDegree_;
    msg.linear.x = ResultVel_;
    msg.linear.y = distance_;
    msg.linear.z = TargetDist_;
    lane = laneDetector_.lane_coef_;

    ControlDataPublisher_.publish(msg);
    LanecoefPublisher_.publish(lane);

    int width = 500;
    int height = 500;
    Mat map_frame = Mat::zeros(Size(width,height), CV_8UC3);

    int dist = 50;
    for(int i=0;i<height;i+=dist)
	line(map_frame, Point(0,i), Point(width,i),Scalar::all(100));

    for(int i=0;i<width;i+=dist)
	line(map_frame, Point(i,0), Point(i,height),Scalar::all(100));

    int centerX = 400, centerY=250;
    double mul = 20;

    rectangle(map_frame, Rect(Point(centerY-10, centerX+26), Point(centerY+10, centerX)), Scalar(0,0,255), -1);
    for(int i = 0; i < ObjCircles_; i++){
      int Y = Obstacle_.circles[i].center.x*mul + centerX;
      int X = Obstacle_.circles[i].center.y*mul + centerY;
      circle(map_frame,Point(X,Y), 3, Scalar(50,250,50), -1);
      putText(map_frame,to_string(i),Point(X,Y), 2, 1.2, Scalar::all(255));
    }
    
    vector<Point> RpointList;
    vector<Point> LpointList;
    vector<Point> CpointList;
    float mul_line = 0.3;
    for(int i = -100; i < height*1.5; i++){
      Point temp_point;
      temp_point.y = i*mul_line + centerY/2;

      temp_point.x = (laneDetector_.lane_coef_.right.a*pow(i,2) + laneDetector_.lane_coef_.right.b*i + laneDetector_.lane_coef_.right.c)*mul_line;
      temp_point.x += centerX/2;
      RpointList.push_back(temp_point);

      temp_point.x = (laneDetector_.lane_coef_.left.a*pow(i,2) + laneDetector_.lane_coef_.left.b*i + laneDetector_.lane_coef_.left.c)*mul_line;
      temp_point.x += centerX/2;
      LpointList.push_back(temp_point);

      temp_point.x = (laneDetector_.lane_coef_.center.a*pow(i,2) + laneDetector_.lane_coef_.center.b*i + laneDetector_.lane_coef_.center.c)*mul_line;
      temp_point.x += centerX/2;
      CpointList.push_back(temp_point);
    }
    const Point* right_points_point = (const cv::Point*) Mat(RpointList).data;
    int right_points_number = Mat(RpointList).rows;
    const Point* left_points_point = (const cv::Point*) Mat(LpointList).data;
    int left_points_number = Mat(LpointList).rows;
    const Point* center_points_point = (const cv::Point*) Mat(CpointList).data;
    int center_points_number = Mat(CpointList).rows;

    polylines(map_frame, &right_points_point, &right_points_number, 1, false, Scalar::all(255), 2);
    polylines(map_frame, &left_points_point, &left_points_number, 1, false, Scalar::all(255), 2);
    polylines(map_frame, &center_points_point, &center_points_number, 1, false, Scalar(150,255,150), 2);

    imshow("Map", map_frame);

    waitKey(1);

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
