#pragma once

#include <iostream>
#include <ros/ros.h>

#include "sock_udp/sock_udp.hpp"

#include <scale_truck_control/ctl.h>
#include <scale_truck_control/lrc.h>

using namespace std;

namespace LocalResiliencyCoordinator{

class LocalRC{
	public:
		LocalRC(ros::NodeHandle nh);
		~LocalRC();

		void lrcPub();
		void udpSend();
		void udpRecv();

		struct UDPsock::UDP_DATA udpData_;
		float TargetVel_;
		float TargetDist_;

	private:
		void init();
		void ctlCallback(const scale_truck_control::ctl &msg);
		void lrcCallback(const scale_truck_control::lrc &lrc);
	
		ros::NodeHandle nodeHandle_;
		ros::Publisher lrcPublisher_;
		ros::Subscriber lrcSubscriber_;
		ros::Subscriber ctlSubscriber_;
	
		//UDP
		UDPsock::UDPsocket UDPsend_;
		UDPsock::UDPsocket UDPrecv_;
		std::string ADDR_;
		int Index_;
		int PORT_;
	
		bool Alpha_;
		float CurVel_;
		float CrcVel_ = 0;
		float CurDist_;
		int sync_flag_;
		bool cam_failure_;
		int Mode_ = static_cast<int>(MODE::TM);
};

}
