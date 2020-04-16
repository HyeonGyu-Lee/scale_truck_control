#include <lane_detect/lane_detect.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#define VIDEO
//#define CAMERA
using namespace cv;
using namespace std;

Mat frame_;

void set_params(LaneDetector* tmp, ros::NodeHandle nh) {
	tmp->width_ = 1280;
	tmp->height_ = 720;
	tmp->corners_.resize(4);
	tmp->warpCorners_.resize(4);
	tmp->corners_[0] = Point2f(300, 450);
	tmp->corners_[1] = Point2f(980, 450);
	tmp->corners_[2] = Point2f(50, 680);
	tmp->corners_[3] = Point2f(1230, 680);
	tmp->wide_extra_upside_ = 0;
	tmp->wide_extra_downside_ = 100;
	tmp->warpCorners_[0] = Point2f(tmp->wide_extra_upside_, 0.0);
	tmp->warpCorners_[0] = Point2f(tmp->width_, tmp->wide_extra_upside_);
	tmp->warpCorners_[0] = Point2f(tmp->wide_extra_downside_, (float)tmp->height_);
	tmp->warpCorners_[0] = Point2f(tmp->width_ - tmp->wide_extra_downside_, (float)tmp->height_);

	tmp->sobel_max_th_ = 255;
	tmp->hls_max_th_ = 255;
	
	nh.getParam("/Kp", tmp->Kp_);
	nh.getParam("/Ki", tmp->Ki_);
	nh.getParam("/Kd", tmp->Kd_);
	nh.getParam("/dt", tmp->dt_);
	nh.getParam("/clicker", tmp->clicker_);
	nh.getParam("/throttle", tmp->throttle_);
	nh.getParam("/hls_min",tmp->hls_min_th_);
	nh.getParam("/sobel_min", tmp->sobel_min_th_);
}

void ImageCallback(const sensor_msgs::ImageConstPtr &image_msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception : %s", e.what());
	}
	frame_ = cv_ptr->image;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pwm_control_node");
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber image_sub;
	geometry_msgs::Twist msg;

	msg.linear.x = 1500;
	msg.angular.z = 1500;
	pub = nh.advertise<geometry_msgs::Twist>("twist_msg",20);
	image_sub  = nh.subscribe("/usb_cam/image_raw", 20, ImageCallback);

	VideoCapture cap;
	Mat video;
	LaneDetector lane_detect;
	set_params(&lane_detect, nh);
#if defined(VIDEO)

	double fps;
	int delay;
	cap.open("/home/jaes/Videos/testroad.mp4");
	if(!cap.isOpened())
		ROS_INFO("Can't Opened Video");
	else {
		fps = cap.get(CAP_PROP_FPS);
		delay = cvRound(1000/fps);
		ROS_INFO("Video_run");
		while(ros::ok()){
			cap >> frame_;
			if(!frame_.empty()) {
				lane_detect.display_img(frame_, delay);
			}
			ros::spinOnce();
		}
	}
#elif defined(CAMERA)

#endif
	return 0;
}
