#include "lane_detect/lane_detect.hpp"
#include <time.h>

using namespace std;
using namespace cv;

namespace lane_detect {

LaneDetector::LaneDetector(ros::NodeHandle nh)
  : nodeHandle_(nh) {
	/********** PID control ***********/
	prev_err_ = 0;

	last_Llane_base_ = 0;
	last_Rlane_base_ = 0;

	left_coef_ = Mat::zeros(3, 1, CV_32F);
	right_coef_ = Mat::zeros(3, 1, CV_32F);

	nodeHandle_.param("ROI/width", width_, 1280);
	nodeHandle_.param("ROI/height", height_, 720);
	center_position_ = width_/2;
	corners_.resize(4);
	warpCorners_.resize(4);

	float t_gap, b_gap, t_height, b_height, f_extra;
	int top_gap, bot_gap, top_height, bot_height, extra, extra_up, extra_down;
	nodeHandle_.param("ROI/top_gap",t_gap, 0.336f);
	nodeHandle_.param("ROI/bot_gap",b_gap, 0.078f);
	nodeHandle_.param("ROI/top_height",t_height, 0.903f);
	nodeHandle_.param("ROI/bot_height",b_height, 0.528f);
	nodeHandle_.param("ROI/extra",f_extra, 0.0f);
	nodeHandle_.param("ROI/extra_up",extra_up, 0);
	nodeHandle_.param("ROI/extra_down",extra_down, 0);

        top_gap = width_ * t_gap; 
        bot_gap = width_ * b_gap;
        top_height = height_ * t_height;
        bot_height = height_ * b_height;

	corners_[0] = Point2f(top_gap, bot_height);
	corners_[1] = Point2f(width_ - top_gap, bot_height);
	corners_[2] = Point2f(bot_gap, top_height);
	corners_[3] = Point2f(width_ - bot_gap, top_height);
	
	wide_extra_upside_ = extra_up;
	wide_extra_downside_ = extra_down;
	
	warpCorners_[0] = Point2f(wide_extra_upside_, 0.0);
	warpCorners_[1] = Point2f(width_ - wide_extra_upside_, 0.0);
	warpCorners_[2] = Point2f(wide_extra_downside_, height_);
	warpCorners_[3] = Point2f(width_ - wide_extra_downside_, height_);

	nodeHandle_.param("LaneDetector/sob_max_th",sobel_max_th_, 255);
	nodeHandle_.param("LaneDetector/sob_min_th",sobel_min_th_, 255);
	nodeHandle_.param("LaneDetector/hls_max_th",hls_max_th_, 175);
	nodeHandle_.param("LaneDetector/hls_min_th",hls_min_th_, 175);
	nodeHandle_.param("LaneDetector/pid_params/Kp",Kp_, 1.0);
	nodeHandle_.param("LaneDetector/pid_params/Ki",Ki_, 0.00001);
	nodeHandle_.param("LaneDetector/pid_params/Kd",Kd_, 0.0025);
	nodeHandle_.param("LaneDetector/pid_params/dt",dt_, 0.1);
	nodeHandle_.param("LaneDetector/filter_param",filter_, 5);
	nodeHandle_.param("LaneDetector/center_height",center_height_, 1.0);

}

	LaneDetector::~LaneDetector(void) {
		clear_release();
	}

	Mat LaneDetector::warped_img(Mat _frame) {
		Mat result, trans;
		trans = getPerspectiveTransform(corners_, warpCorners_);
		warpPerspective(_frame, result, trans, Size(width_, height_));

		return result;
	}

	Mat LaneDetector::warped_back_img(Mat _frame) {
		Mat result, trans;
		trans = getPerspectiveTransform(warpCorners_, corners_);
		warpPerspective(_frame, result, trans, Size(width_, height_));

		return result;
	}

	Mat LaneDetector::abs_sobel_thresh(Mat _frame) {
		Mat gray_frame, sobel_frame, abs_sobel_frame;
		double min, max, val;
		cvtColor(_frame, gray_frame, COLOR_BGR2GRAY);
		Sobel(gray_frame, sobel_frame, CV_64F, 1, 0);
		//abs_sobel_frame = abs(sobel_frame);
		sobel_frame.convertTo(abs_sobel_frame, CV_8U);

		minMaxLoc(abs_sobel_frame, &min, &max);

		Mat scaled_sobel_binary_frame = Mat::zeros(sobel_frame.rows, sobel_frame.cols, CV_8UC1);

		for (int j = 0; j < sobel_frame.rows; j++) {
			for (int i = 0; i < sobel_frame.cols; i++) {
				val = (abs_sobel_frame.at<uchar>(j, i)) * 255 / max;
				if ((val >= sobel_min_th_) && (val <= sobel_max_th_))
					scaled_sobel_binary_frame.at<uchar>(j, i) = 255;
			}
		}
		return scaled_sobel_binary_frame;
	}

	Mat LaneDetector::hls_lthresh(Mat _frame) {
		Mat hls_frame;
		vector<Mat> hls_images(3);
		double min, max;

		cvtColor(_frame, hls_frame, COLOR_BGR2HLS);
		split(hls_frame, hls_images);

		minMaxLoc(hls_images[1], &min, &max);

		for (int j = 0; j < hls_images[1].rows; j++) {
			for (int i = 0; i < hls_images[1].cols; i++) {
				if (((hls_images[1].at<uchar>(j, i) * 255 / max) > hls_min_th_) && ((hls_images[1].at<uchar>(j, i) * 255 / max) <= hls_max_th_))
					hls_images[1].at<uchar>(j, i) = 255;
			}
		}

		return hls_images[1];
	}

	Mat LaneDetector::pipeline_img(Mat _frame) {
		Mat abs_sobel_frame = abs_sobel_thresh(_frame);
		Mat l_frame = hls_lthresh(_frame);
		Mat combined_frame = Mat::zeros(_frame.rows, _frame.cols, CV_8UC1);
		for (int j = 0; j < combined_frame.rows; j++) {
			for (int i = 0; i < combined_frame.cols; i++) {
				if ((abs_sobel_frame.at<uchar>(j, i) == 255) || (l_frame.at<uchar>(j, i) == 255)) combined_frame.at<uchar>(j, i) = 255;
			}
		}
		return combined_frame;
	}

	int LaneDetector::arrMaxIdx(int hist[], int start, int end, int Max) {
		int max_index = -1;
		int max_val = 0;

		if (end > Max)
			end = Max;

		for (int i = start; i < end; i++) {
			if (max_val < hist[i]) {
				max_val = hist[i];
				max_index = i;
			}
		}
		if (max_index == -1)
			cout << "ERROR : hist range" << endl;
		return max_index;
	}

	double LaneDetector::gaussian(double x, double mu, double sig) {
		return exp((-1) * pow(x - mu, 2.0) / (2 * pow(sig, 2.0)));
	}

	Mat LaneDetector::polyfit(vector<int> x_val, vector<int> y_val) {
		Mat coef(3, 1, CV_32F);
		int i, j, k, n, N;
		N = (int)x_val.size();
		n = 2;
		double* x, * y;
		x = new double[N];
		y = new double[N];
		for (int q = 0; q < N; q++) {
			x[q] = (double)(x_val[q]);
			y[q] = (double)(y_val[q]);
		}
		double* X;
		X = new double[2 * n + 1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
		for (i = 0; i < (2 * n + 1); i++)
		{
			X[i] = 0;
			for (j = 0; j < N; j++)
				X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
		}
		double** B, * a;
		B = new double* [n + 1];
		for (int i = 0; i < (n + 1); i++)
			B[i] = new double[n + 2];
		a = new double[n + 1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
		for (i = 0; i <= n; i++)
			for (j = 0; j <= n; j++)
				B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
		double* Y;
		Y = new double[n + 1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
		for (i = 0; i < (n + 1); i++)
		{
			Y[i] = 0;
			for (j = 0; j < N; j++)
				Y[i] = Y[i] + pow(x[j], i) * y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
		}
		for (i = 0; i <= n; i++)
			B[i][n + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
		n = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations

		for (i = 0; i < n; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
			for (k = i + 1; k < n; k++)
				if (B[i][i] < B[k][i])
					for (j = 0; j <= n; j++)
					{
						double temp = B[i][j];
						B[i][j] = B[k][j];
						B[k][j] = temp;
					}

		for (i = 0; i < (n - 1); i++)            //loop to perform the gauss elimination
			for (k = i + 1; k < n; k++)
			{
				double t = B[k][i] / B[i][i];
				for (j = 0; j <= n; j++)
					B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
			}
		for (i = n - 1; i >= 0; i--)                //back-substitution
		{                        //x is an array whose values correspond to the values of x,y,z..
			a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
			for (j = 0; j < n; j++)
				if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
					a[i] = a[i] - B[i][j] * a[j];
			a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
			coef.at<float>(i, 0) = (float)a[i];
		}

		delete[] x;
		delete[] y;
		delete[] X;
		delete[] Y;
		delete[] B;
		delete[] a;

		return coef;
	}

	Mat LaneDetector::detect_lines_sliding_window(Mat _frame, bool _view) {
		Mat frame, result;
		int width = _frame.cols;
		int height = _frame.rows;

		_frame.copyTo(frame);
		Mat nonZero;
		findNonZero(frame, nonZero);

		vector<int> good_left_inds;
		vector<int> good_right_inds;
		int* hist = new int[width];

		double* weight_distrib = new double[width];
		for (int i = 0; i < width; i++) {
			hist[i] = 0;
		}

		//int hist_Max = 0; 
		for (int j = (height / 2); j < height; j++) { // hist 범위 절반부터 읽기
			for (int i = 0; i < width; i++) {
				if (frame.at <uchar>(j, i) == 255) {
					hist[i] += 1;
				}
			}
		}

		//hist_Max = arrMaxIdx(hist, 0, width, width);
		/*
		if (last_Llane_base_ != 0 || last_Rlane_base_ != 0) {

			int distrib_width = 120;
			double sigma = distrib_width / 12.8;

			int leftx_start = last_Llane_base_ - distrib_width / 2;
			int leftx_end = last_Llane_base_ + distrib_width / 2;

			int rightx_start = last_Rlane_base_ - distrib_width / 2;
			int rightx_end = last_Rlane_base_ + distrib_width / 2;

			for (int i = 0; i < _frame.cols; i++) {
				if ((i >= leftx_start) && (i <= leftx_end)) {
					weight_distrib[i] = gaussian(i, last_Llane_base_, sigma);
					hist[i] *= (int)weight_distrib[i];
				}
				else if ((i >= rightx_start) && (i <= rightx_end)) {
					weight_distrib[i] = gaussian(i, last_Rlane_base_, sigma);
					hist[i] *= (int)weight_distrib[i];
				}
			}
		}*/

		cvtColor(frame, result, COLOR_GRAY2BGR);

		int mid_point = width / 2; // 320
		int quarter_point = mid_point / 2; // 160
		int n_windows = 9;
		int margin = 120 * width / 1280;
		int min_pix = 50 * width / 1280;

		int window_width = margin;
		int window_height = height / n_windows;

		int offset = 0;
		int range = 120 / 4;
		int Lstart = quarter_point - offset; // 160 - 0
		int Rstart = mid_point + quarter_point + offset; // 480 - 0
		// mid_point = 320, Lstart +- range = 40 ~ 280
		//int Llane_base = arrMaxIdx(hist, Lstart - range, Lstart + range, _width);
		//int Rlane_base = arrMaxIdx(hist, Rstart - range, Rstart + range, _width);
		int Llane_base = arrMaxIdx(hist, 100, mid_point, width);
		int Rlane_base = arrMaxIdx(hist, mid_point, width - 100, width);

		int Llane_current = Llane_base;
		int Rlane_current = Rlane_base;

		last_Llane_base_ = Llane_base;
		last_Rlane_base_ = Rlane_base;

		int L_prev = Llane_current;
	    int R_prev = Rlane_current;
		int L_gap = 0;
		int R_gap = 0;

		unsigned int index;

		for (int window = 0; window < n_windows; window++) {
			int Ly_pos = height - (window + 1) * window_height - 1; // win_y_low , win_y_high = win_y_low - window_height
			int Ry_pos = height - (window + 1) * window_height - 1;

			int Lx_pos = Llane_current - margin; // win_xleft_low, win_xleft_high = win_xleft_low + margin*2
			int Rx_pos = Rlane_current - margin; // win_xrignt_low, win_xright_high = win_xright_low + margin*2
			if (_view) {
				rectangle(result, \
					Rect(Lx_pos, Ly_pos, window_width * 2, window_height), \
					Scalar(255, 50, 100), 1);
				rectangle(result, \
					Rect(Rx_pos, Ry_pos, window_width * 2, window_height), \
					Scalar(100, 50, 255), 1);
			}
			uchar* data_output = result.data;
			int nZ_y, nZ_x;

			good_left_inds.clear();
			good_right_inds.clear();

			for (unsigned int index = (nonZero.total() - 2); index > 1; index--) {
				nZ_y = nonZero.at<Point>(index).y;
				nZ_x = nonZero.at<Point>(index).x;

				if ((nZ_y >= Ly_pos) && \
					(nZ_y < (height - window_height * window)) && \
					(nZ_x >= Lx_pos) && \
					(nZ_x < (Lx_pos + margin * 2))) {
					if (_view) {
						result.at<Vec3b>(nonZero.at<Point>(index))[0] = 255;
						result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
						result.at<Vec3b>(nonZero.at<Point>(index))[2] = 0;
					}
					good_left_inds.push_back(index);
				}
				if ((nZ_y >= (Ry_pos)) && \
					(nZ_y < (height - window_height * window)) && \
					(nZ_x >= Rx_pos) && \
					(nZ_x < (Rx_pos + margin * 2))) {
					if (_view) {
						result.at<Vec3b>(nonZero.at<Point>(index))[0] = 0;
						result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
						result.at<Vec3b>(nonZero.at<Point>(index))[2] = 255;
					}
					good_right_inds.push_back(index);
				}
			}
			int Lsum, Rsum;
			Lsum = Rsum = 0;
			unsigned int _size;

			if (good_left_inds.size() > (size_t)min_pix) {
				_size = (unsigned int)(good_left_inds.size());
				for (index = 0; index < _size; index++) {
					Lsum += nonZero.at<Point>(good_left_inds.at(index)).x;
				}
				Llane_current = Lsum / _size;
				//circle(result, Point(Llane_current, Ly_pos + (window_height / 2)), 5, Scalar(255, 255, 255), -1);
				left_x_.insert(left_x_.end(), Llane_current);
				left_y_.insert(left_y_.end(), Ly_pos + (window_height / 2));
				//cout << "L : " << Llane_current << " / " << Ly_pos + (window_height / 2) << endl;
			} else
				Llane_current += (L_gap);

			if (good_right_inds.size() > (size_t)min_pix) {
				_size = (unsigned int)(good_right_inds.size());
				for (index = 0; index < _size; index++) {
					Rsum += nonZero.at<Point>(good_right_inds.at(index)).x;
				}
				Rlane_current = Rsum / _size;
				//circle(result, Point(Rlane_current, Ry_pos + (window_height / 2)), 5, Scalar(255, 255, 255), -1);
				right_x_.insert(right_x_.end(), Rlane_current);
				right_y_.insert(right_y_.end(), Ry_pos + (window_height / 2));
				//cout << "R : " << Rlane_current << " / " << Ry_pos + (window_height / 2) << endl;
			} else
				Rlane_current += (R_gap);
			if (window != 0) {
				if (Rlane_current != R_prev) {
					R_gap = (Rlane_current - R_prev);
				}

				if (Llane_current != L_prev) {
					L_gap = (Llane_current - L_prev);
				}
			}
			if ((Lsum != 0) && (Rsum != 0)) {
				center_x_.insert(center_x_.end(), (Llane_current + Rlane_current) / 2);
				center_y_.insert(center_y_.end(), Ly_pos + (window_height / 2));
				//circle(result, Point((Llane_current + Rlane_current) / 2, Ly_pos + (window_height / 2)), 5, Scalar(255, 255, 255), -1);
			}

			L_prev = Llane_current;
			R_prev = Rlane_current;


			//left_lane_inds_.insert(left_lane_inds_.end(), good_left_inds.begin(), good_left_inds.end());
			//right_lane_inds_.insert(right_lane_inds_.end(), good_right_inds.begin(), good_right_inds.end());
		}

		/*vector<int>::iterator iter;
		for (index = 0, iter = left_lane_inds_.begin(); iter != left_lane_inds_.end(); iter++, index++) {
			if ((int)(nonZero.total()) >= *iter) {
				left_x_.insert(left_x_.end(), nonZero.at<Point>(*iter).x);
				left_y_.insert(left_y_.end(), nonZero.at<Point>(*iter).y);
			}
		}
		for (index = 0, iter = right_lane_inds_.begin(); iter != right_lane_inds_.end(); iter++, index++) {
			if ((int)(nonZero.total()) >= *iter) {
				right_x_.insert(right_x_.end(), nonZero.at<Point>(*iter).x);
				right_y_.insert(right_y_.end(), nonZero.at<Point>(*iter).y);
			}
		}*/

		if (left_x_.size() != 0) {
			left_coef_ = polyfit(left_y_, left_x_);
		}
		if (right_x_.size() != 0) {
			right_coef_ = polyfit(right_y_, right_x_);
		}

		if ((left_x_.size() != 0) && (right_x_.size() != 0)) {
			center_coef_ = polyfit(center_y_, center_x_);
		}

		delete[] hist;
		delete[] weight_distrib;

		return result;
	}

	Mat LaneDetector::draw_lane(Mat _sliding_frame, Mat _frame, bool _view) {
		Mat new_frame, left_coef(left_coef_), right_coef(right_coef_), center_coef(center_coef_), trans;
		trans = getPerspectiveTransform(warpCorners_, corners_);
		_frame.copyTo(new_frame);

		vector<Point> left_point;
		vector<Point> right_point;
		vector<Point> center_point;

		vector<Point2f> left_point_f;
		vector<Point2f> right_point_f;
		vector<Point2f> center_point_f;

		vector<Point2f> warped_left_point;
		vector<Point2f> warped_right_point;
		vector<Point2f> warped_center_point;

		vector<Point> left_points;
		vector<Point> right_points;
		vector<Point> center_points;

		if ((!left_coef.empty()) && (!right_coef.empty())) {
			for (int i = 0; i <= height_; i++) {
				Point temp_left_point;
				Point temp_right_point;
				Point temp_center_point;

				temp_left_point.x = (int)((left_coef.at<float>(2, 0) * pow(i, 2)) + (left_coef.at<float>(1, 0) * i) + left_coef.at<float>(0, 0));
				temp_left_point.y = (int)i;
				temp_right_point.x = (int)((right_coef.at<float>(2, 0) * pow(i, 2)) + (right_coef.at<float>(1, 0) * i) + right_coef.at<float>(0, 0));
				temp_right_point.y = (int)i;
				temp_center_point.x = (int)((center_coef.at<float>(2, 0) * pow(i, 2)) + (center_coef.at<float>(1, 0) * i) + center_coef.at<float>(0, 0));
				temp_center_point.y = (int)i;

				left_point.push_back(temp_left_point);
				right_point.push_back(temp_right_point);
				left_point_f.push_back(temp_left_point);
				right_point_f.push_back(temp_right_point);
				center_point.push_back(temp_center_point);
				center_point_f.push_back(temp_center_point);
			}
			const Point* left_points_point_ = (const cv::Point*) Mat(left_point).data;
			int left_points_number_ = Mat(left_point).rows;
			const Point* right_points_point_ = (const cv::Point*) Mat(right_point).data;
			int right_points_number_ = Mat(right_point).rows;
			const Point* center_points_point_ = (const cv::Point*) Mat(center_point).data;
			int center_points_number_ = Mat(center_point).rows;

			if (_view) {
				polylines(_sliding_frame, &left_points_point_, &left_points_number_, 1, false, Scalar(255, 200, 200), 5);
				polylines(_sliding_frame, &right_points_point_, &right_points_number_, 1, false, Scalar(200, 200, 255), 5);
				polylines(_sliding_frame, &center_points_point_, &center_points_number_, 1, false, Scalar(200, 255, 200), 5);
			}
			perspectiveTransform(left_point_f, warped_left_point, trans);
			perspectiveTransform(right_point_f, warped_right_point, trans);
			perspectiveTransform(center_point_f, warped_center_point, trans);

			for (int i = 0; i <= height_; i++) {
				Point temp_left_point;
				Point temp_right_point;
				Point temp_center_point;

				temp_left_point.x = (int)warped_left_point[i].x;
				temp_left_point.y = (int)warped_left_point[i].y;
				temp_right_point.x = (int)warped_right_point[i].x;
				temp_right_point.y = (int)warped_right_point[i].y;
				temp_center_point.x = (int)warped_center_point[i].x;
				temp_center_point.y = (int)warped_center_point[i].y;

				left_points.push_back(temp_left_point);
				right_points.push_back(temp_right_point);
				center_points.push_back(temp_center_point);
			}

			const Point* left_points_point = (const cv::Point*) Mat(left_points).data;
			int left_points_number = Mat(left_points).rows;
			const Point* right_points_point = (const cv::Point*) Mat(right_points).data;
			int right_points_number = Mat(right_points).rows;
			const Point* center_points_point = (const cv::Point*) Mat(center_points).data;
			int center_points_number = Mat(center_points).rows;

			if (_view) {
				polylines(new_frame, &left_points_point, &left_points_number, 1, false, Scalar(255, 100, 100), 5);
				polylines(new_frame, &right_points_point, &right_points_number, 1, false, Scalar(100, 100, 255), 5);
				polylines(new_frame, &center_points_point, &center_points_number, 1, false, Scalar(100, 255, 100), 5);
			}
			left_point.clear();
			right_point.clear();
			center_point.clear();

			return new_frame;
		}
		return _frame;
	}

	void LaneDetector::clear_release() {
		left_lane_inds_.clear();
		right_lane_inds_.clear();
		left_x_.clear();
		left_y_.clear();
		right_x_.clear();
		right_y_.clear();
		center_x_.clear();
		center_y_.clear();
	}

	void LaneDetector::calc_curv_rad_and_center_dist(Mat _frame, bool _view) {
		Mat l_fit(left_coef_), r_fit(right_coef_), c_fit(center_coef_);
		//vector<int> lx(left_x_), ly(left_y_), rx(right_x_), ry(right_y_);
		int car_position = width_ / 2;
		int lane_center_position, lane_top, lane_bot;
		float center_position;
		//float left_cr;
		//float right_cr;

		float ym_per_pix = 3.048f / 100.f;
		/*float xm_per_pix = 3.7f / 378.0f;
		Mat left_coef_cr(3, 1, CV_32F);
		Mat right_coef_cr(3, 1, CV_32F);
		if (lx.size() != 0 && rx.size() != 0) {
			for (int i = 0; i < lx.size(); i++) {
				lx[i] = lx[i] * xm_per_pix;
				ly[i] = ly[i] * ym_per_pix;
			}
			for (int i = 0; i < rx.size(); i++) {
				rx[i] = rx[i] * xm_per_pix;
				ry[i] = ry[i] * ym_per_pix;
			}
			left_coef_cr = polyfit(lx, ly);
			right_coef_cr = polyfit(rx, ry);
			left_cr = powf((1 + powf(2 * left_coef_cr.at<float>(2, 0) * 0 * xm_per_pix + left_coef_cr.at<float>(1, 0), 2)), 1.5f) / fabs(2 * left_coef_cr.at<float>(2, 0) + 0.000001f);
			right_cr = powf((1 + powf(2 * right_coef_cr.at<float>(2, 0) * 0 * xm_per_pix + right_coef_cr.at<float>(1, 0), 2)), 1.5f) / fabs(2 * right_coef_cr.at<float>(2, 0) + 0.000001f);
			left_curve_radius_ = left_cr;
			right_curve_radius_ = right_cr;
		}*/

		if (!l_fit.empty() && !r_fit.empty()) {
			//lane_center_position = (l_fit.at<float>(0, 0) + r_fit.at<float>(0, 0)) / 2;
			//lane_center_position = c_fit.at<float>(0, 0);
			int i = height_*center_height_;
			lane_top = (int)((c_fit.at<float>(2, 0) * pow(0, 2)) + (c_fit.at<float>(1, 0) * 0) + c_fit.at<float>(0, 0));
			//lane_bot = (int)((c_fit.at<float>(2, 0) * pow(height_, 2)) + (c_fit.at<float>(1, 0) * height_) + c_fit.at<float>(0, 0));
			lane_center_position = (int)((c_fit.at<float>(2, 0) * pow(i, 2)) + (c_fit.at<float>(1, 0) * i) + c_fit.at<float>(0, 0));

			if ((lane_center_position > 0) && (lane_center_position < (float)width_)) {
				center_position = (car_position - lane_center_position) * ym_per_pix;
				err_ = (float)lane_center_position - center_position_;
				I_err_ += err_ * dt_;
				D_err_ = (err_ - prev_err_) / dt_;
				prev_err_ = err_;

				result_ = (Kp_ * err_) + (Ki_ * I_err_) + (Kd_ * D_err_); // PID
				center_position_ += (result_);
				if (_view) {
					line(_frame, Point(lane_center_position, 0), Point(lane_center_position, height_), Scalar(0, 255, 0), 5);
					//line(_frame, Point(left_curve_radius_, 0), Point(left_curve_radius_, height_), Scalar(255, 150, 0), 3);
					//line(_frame, Point(right_curve_radius_, 0), Point(right_curve_radius_, height_), Scalar(0, 150, 255), 3);
					line(_frame, Point(center_position_, 0), Point(center_position_, height_), Scalar(200, 150, 200), 5);
					line(_frame, Point(lane_top, 0), Point(lane_top, height_), Scalar(200, 100, 100), 5);
				}
			}
		}
	}

	int LaneDetector::display_img(Mat _frame, int _delay, bool _view) {
		Mat new_frame, warped_frame, gray_frame, blur_frame, binary_frame, sliding_frame, resized_frame;
		Mat filter(filter_, filter_, CV_8U, Scalar(1));

		resize(_frame, new_frame, Size(width_, height_));
		warped_frame = warped_img(new_frame);
		cvtColor(warped_frame, gray_frame, COLOR_BGR2GRAY);
		threshold(gray_frame, binary_frame, 0, 255, THRESH_BINARY|THRESH_OTSU);
		morphologyEx(binary_frame, blur_frame, MORPH_OPEN, filter);
		//binary_frame = pipeline_img(blur_frame);
		sliding_frame = detect_lines_sliding_window(blur_frame, _view);
		resized_frame = draw_lane(sliding_frame, new_frame, _view);
		calc_curv_rad_and_center_dist(resized_frame, _view);
		clear_release();
		if (_view) {
			namedWindow("Window1");
			moveWindow("Window1", 0, 0);
			namedWindow("Window2");
			moveWindow("Window2", 640, 0);
			namedWindow("Window3");
			moveWindow("Window3", 1280, 0);

			string TEXT = "ROI";
			Point2f T_pos(corners_[0]);
			putText(new_frame, TEXT, T_pos, FONT_HERSHEY_DUPLEX, 2, Scalar(0, 0, 255), 5, 8);

			line(new_frame, corners_[0], corners_[2], Scalar(0, 0, 255), 5);
			line(new_frame, corners_[2], corners_[3], Scalar(0, 0, 255), 5);
			line(new_frame, corners_[3], corners_[1], Scalar(0, 0, 255), 5);
			line(new_frame, corners_[1], corners_[0], Scalar(0, 0, 255), 5);

			resize(new_frame, new_frame, Size(640, 360));
			resize(sliding_frame, sliding_frame, Size(640, 360));
			resize(resized_frame, resized_frame, Size(640, 360));

			imshow("Window1", new_frame);
			imshow("Window2", sliding_frame);
			imshow("Window3", resized_frame);

			waitKey(_delay);
		}

		return center_position_;
	};

} /* namespace lane_detect */
