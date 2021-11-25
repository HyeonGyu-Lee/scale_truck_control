#include <iostream>
#include <chrono>
#include <thread>
#include <sys/time.h>

#include "sock_udp.hpp"

namespace CenterResiliencyCoordinator{

class CenterRC{
	public:
		CenterRC();
		~CenterRC();

		void Communicate();
	private:
		UDPsock::UDPsocket UDPsend_;
		UDPsock::UDPsocket UDPrecv_;

		void init();
		void UDPsendData(float pred_vel, int to);
		void UDPrecvTruckData();
		void UDPrecvControlCenterData();
		float PredictVelocity(int index);
		void ModeCheck(uint8_t lv_mode, uint8_t fv1_mode, uint8_t fv2_mode);

		int Index_;
		float TarVel_ = 0;
		float PredVel_ = 0;
		float TarDist_ = 0.8;
		uint8_t CrcMode_ = 0;
		float sampling_time_ = 0.1;

		bool alpha0_ = 0;
		bool beta0_ = 0;
		bool gamma0_ = 0;
		float lv_cur_vel_ = 0;
		float lv_pred_vel_ = 0;
		float lv_cur_dist_;
		uint8_t lv_lrc_mode_ = 0;
		
		bool alpha1_ = 0;
		bool beta1_ = 0;
		bool gamma1_ = 0;
		float fv1_cur_vel_ = 0;
		float fv1_pred_vel_ = 0;
		float fv1_cur_dist_ = 0;
		float fv1_prev_cur_dist_ = 0;
		uint8_t fv1_lrc_mode_ = 0;

		bool alpha2_ = 0;
		bool beta2_ = 0;
		bool gamma2_ = 0;
		float fv2_cur_vel_ = 0;
		float fv2_pred_vel_ = 0;
		float fv2_cur_dist_ = 0;
		float fv2_prev_cur_dist_ = 0;
		uint8_t fv2_lrc_mode_ = 0;		
};

}

