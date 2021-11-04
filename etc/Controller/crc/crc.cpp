#include "crc.hpp"

namespace CenterResiliencyCoordinator{
	
CenterRC::CenterRC()
	: UDPsend_(), UDPrecv_(){

	init();	
}

CenterRC::~CenterRC(){

}

void CenterRC::init(){
	Index_ = 100;

	UDPsend_.GROUP_ = "239.255.255.250";
	UDPsend_.PORT_ = 9392;
	UDPsend_.sendInit();

	UDPrecv_.GROUP_ = "239.255.255.250";
	UDPrecv_.PORT_ = 9392;
	UDPrecv_.recvInit();
}

void CenterRC::UDPsendData(float pred_vel, int to){
	struct UDPsock::UDP_DATA udpData;

	udpData.index = Index_;
	udpData.to = to;
	udpData.mode = CrcMode_;
	udpData.predict_vel = pred_vel;

	UDPsend_.sendData(udpData);
}

void CenterRC::UDPrecvTruckData(){
	struct UDPsock::UDP_DATA udpData;
	UDPrecv_.recvData(&udpData);

	if (udpData.index == 0 && udpData.to == 100){	//LV
		alpha0_ = udpData.alpha;
		beta0_ = udpData.beta;
		gamma0_ = udpData.gamma;
		lv_cur_vel_ = udpData.current_vel;
		lv_cur_dist_ = udpData.current_dist;
		lv_lrc_mode_ = udpData.mode;
	}
	if (udpData.index == 1 && udpData.to == 100){	//FV1
		fv1_prev_cur_dist_ = fv1_cur_dist_;

		alpha1_ = udpData.alpha;
		beta1_ = udpData.beta;
		gamma1_ = udpData.gamma;
		fv1_cur_vel_ = udpData.current_vel;
		fv1_cur_dist_ = udpData.current_dist;
		fv1_lrc_mode_ = udpData.mode;
	}
	if (udpData.index == 2 && udpData.to == 100){	//FV2
		fv2_prev_cur_dist_ = fv2_cur_dist_;

		alpha2_ = udpData.alpha;
		beta2_ = udpData.beta;
		gamma2_ = udpData.gamma;
		fv2_cur_vel_ = udpData.current_vel;
		fv2_cur_dist_ = udpData.current_dist;
		fv2_lrc_mode_ = udpData.mode;
	}
}

void CenterRC::UDPrecvControlCenterData(){
	struct UDPsock::UDP_DATA udpData;
	UDPrecv_.recvData(&udpData);
}

float CenterRC::PredictVelocity(int index){
	if (index == 0){	//LV
		if (!alpha0_){
			lv_pred_vel_ = lv_cur_vel_;
		}
		else if (alpha0_ && !alpha1_){
			lv_pred_vel_ = ((fv1_cur_dist_ - fv1_prev_cur_dist_) / sampling_time_) + fv1_cur_vel_;
		}
		else if ((alpha0_ || alpha1_) && !alpha2_){
			lv_pred_vel_ = ((fv1_cur_dist_ - fv1_prev_cur_dist_) / sampling_time_) + ((fv2_cur_dist_ - fv2_prev_cur_dist_) / sampling_time_) + fv2_cur_vel_;
		}
		else{	//All trucks' velocity sensors are fail 
			lv_pred_vel_ = 0;
			CrcMode_ = 2;
		}
		return lv_pred_vel_;
	}
	else if (index == 1){	//FV1
		if (!alpha1_){
			fv1_pred_vel_ = fv1_cur_vel_;
		}
		else if (alpha1_ && !alpha0_){
			fv1_pred_vel_ = ((-1.0) * ((fv1_cur_dist_ - fv1_prev_cur_dist_) / sampling_time_)) + lv_cur_vel_;
		}
		else if ((alpha1_ || alpha0_) && !alpha2_){
			fv1_pred_vel_ = ((fv2_cur_dist_ - fv2_prev_cur_dist_) / sampling_time_) + fv2_cur_vel_;
		}
		else{	//All trucks' velocity sensors are fail
			fv1_pred_vel_ = 0;
			CrcMode_ = 2;
		}
		return fv1_pred_vel_;
	}
	else if (index == 2){	//FV2
		if (!alpha2_){
			fv2_pred_vel_ = fv2_cur_vel_;
		}
		else if (alpha2_ && !alpha1_){
			fv2_pred_vel_ = ((-1.0) * ((fv2_cur_dist_ - fv2_prev_cur_dist_) / sampling_time_)) + fv1_cur_vel_;
		}
		else if ((alpha2_ || alpha1_) && !alpha0_){
			fv2_pred_vel_ = ((-1.0) * ((fv1_cur_dist_ - fv1_prev_cur_dist_) / sampling_time_)) + ((-1.0) * ((fv2_cur_dist_ - fv2_prev_cur_dist_) / sampling_time_)) + lv_cur_vel_;
		}
		else{	//All trucks' velocity sensors are fail
			fv2_pred_vel_ = 0;
			CrcMode_ = 2;
		}
		return fv2_pred_vel_;
	}
	else{
		printf("Invalid Truck index!\n");
		return -1;
	}
}

void CenterRC::ModeCheck(uint8_t lv_mode, uint8_t fv1_mode, uint8_t fv2_mode){
	if ((lv_mode == 0) && (fv1_mode == 0) && (fv2_mode == 0)){
		CrcMode_ = 0;
	}
	else if (((lv_mode == 2) || (fv1_mode == 2) || (fv2_mode == 2)) || ((lv_mode == 1) && (fv1_mode == 1) && (fv2_mode == 1))){
		CrcMode_ = 2;
	}
	else{
		CrcMode_ = 1;
	}
}

void CenterRC::Communicate(){	
	UDPrecvTruckData();
	ModeCheck(lv_lrc_mode_, fv1_lrc_mode_, fv2_lrc_mode_);

	UDPsendData(PredictVelocity(0), 0);
	UDPsendData(PredictVelocity(1), 1);
	UDPsendData(PredictVelocity(2), 2);

	printf("\033[2J\033[1;1H");
	printf("CRC is running ...\n");
	printf("CRC and each MODEs of LV, FV1, FV2:\t%d || %d, %d, %d\n", CrcMode_, lv_lrc_mode_,fv1_lrc_mode_,fv2_lrc_mode_);
	printf("Predict Velocitys of LV, FV1, FV2:\t%.3f, %.3f, %.3f\n", lv_pred_vel_, fv1_pred_vel_, fv2_pred_vel_);
}

}
