#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <time.h>

using namespace cv;
using namespace std;

namespace lane_detect {

class LaneDetector{
public:
	LaneDetector(ros::NodeHandle nh);
	~LaneDetector(void);

	float display_img(Mat _frame, int _delay, bool _view);

private:
	void LoadParams(void);
	int arrMaxIdx(int hist[], int start, int end, int Max);
	Mat polyfit(vector<int> x_val, vector<int> y_val);
	Mat warped_back_img(Mat _frame);
	Mat warped_img(Mat _frame);
	Mat detect_lines_sliding_window(Mat _frame, bool _view);
	Mat draw_lane(Mat _sliding_frame, Mat _frame, bool _view);
	void clear_release();
	void calc_curv_rad_and_center_dist(Mat _frame, bool _view);

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
	float center_height_, trust_height_, lp_, K1_, K2_;
	float e_values_[2];
	
	/********** PID control ***********/
	int prev_lane_, prev_pid_;
	double Kp_term_, Ki_term_, Kd_term_, err_, prev_err_, I_err_, D_err_, result_;
	
	/********** Lane_detect data ***********/
	vector<Point2f> corners_;
	vector<Point2f> warpCorners_;
	float wide_extra_upside_, wide_extra_downside_;

	int sobel_min_th_, sobel_max_th_;
	int hls_min_th_, hls_max_th_;
	int width_, height_;
	/********** PID control ***********/
	int clicker_, throttle_, filter_;
	double Kp_, Ki_, Kd_, dt_;

};

}
