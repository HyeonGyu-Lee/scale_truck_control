#pragma once

#include "sock_udp/sock_udp.hpp"

#include <scale_truck_control/lrc.h>

namespace LRC{
class LRC{
public:
	LRC(ros::NodeHandle nh);
	~LRC(void);
private:
	void init();
	void Save();
	void velCallback(const scale_truck_control::vel &msg);
	void distCallback(const scale_truck_control::ctl &msg);
	void send();
	void receive();

	ros::NodeHandle nodeHandle_;
	
	//UDP
	UDPsock::UDPsocket UDPsend_;
	UDPsock::UDPsocket UDPrecv_;
	std::string ADDR_;
	int Index_;
	int PORT_;
	struct UDPsock::UDP_DATA udpData_;

	bool Alpha_;
	float CurVel_;
	float TargetVel_;
	float CurDist_;
	float TargetDist_;
	enum MODE Mode_ = TM;
}
}
