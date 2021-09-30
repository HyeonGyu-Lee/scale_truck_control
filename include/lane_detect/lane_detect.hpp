#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>
#include <iostream>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <scale_truck_control/lane_coef.h>
#include <time.h>

using namespace cv;
using namespace std;

namespace LaneDetect {

class LaneDetector{
public:
	LaneDetector(ros::NodeHandle nh);
	~LaneDetector(void);

	//Timer
	struct timeval start_, end_;

	float display_img(Mat _frame, int _delay, bool _view);
	void get_steer_coef(float vel);
	float K1_, K2_;
	int distance_ = 0;
	scale_truck_control::lane_coef lane_coef_;
	Mat frame_;

private:
	void LoadParams(void);
	int arrMaxIdx(int hist[], int start, int end, int Max);
	Mat polyfit(vector<int> x_val, vector<int> y_val);
	Mat warped_back_img(Mat _frame);
	Mat warped_img(Mat _frame);
	Mat detect_lines_sliding_window(Mat _frame, bool _view);
	Mat draw_lane(Mat _sliding_frame, Mat _frame);
	void calc_curv_rad_and_center_dist();
	void clear_release();
	void steer_error_log();

	ros::NodeHandle nodeHandle_;

	/********** Camera calibration **********/
	Mat map1_, map2_;

	/********** Lane_detect data ***********/
	int last_Llane_base_;
	int last_Rlane_base_;

	vector<int> left_lane_inds_;
	vector<int> right_lane_inds_;
	vector<int> left_x_;
	vector<int> left_y_;
	vector<int> right_x_;
	vector<int> right_y_;
	vector<int> center_x_;
	vector<int> center_y_;

	Mat left_coef_;
	Mat right_coef_;
	Mat center_coef_;
	float left_curve_radius_;
	float right_curve_radius_;
	float center_position_;
	float SteerAngle_;
	float eL_height_, trust_height_, e1_height_, lp_;
	float K_;
	double a_[5], b_[5];
	vector<float> e_values_;

	/********** PID control ***********/
	int prev_lane_, prev_pid_;
	double Kp_term_, Ki_term_, Kd_term_, err_, prev_err_, I_err_, D_err_, result_;
	
	/********** Lane_detect data ***********/
	vector<Point2f> corners_;
	vector<Point2f> warpCorners_;
	float wide_extra_upside_, wide_extra_downside_;

	int width_, height_;
	bool option_; // dynamic ROI
	int threshold_;
	double diff_;
};

}
