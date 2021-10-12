#include "lrc/lrc.hpp"

namespace LRC{

LRC::LRC(ros::NodeHandle nh)
	: nodeHandle_(nh), UDPsend_(), UDPrecv_(){
	
	init();	

}

LRC::~LRC(){
	
}

void LRC::init(){
	/**************/
	/* UDP Option */
	/**************/
	nodeHandle_.param("params/udp_group_addr", ADDR_, std::string("239.255.255.250"));
	nodeHandle_.param("params/udp_group_port", PORT_, 9307);
	nodeHandle_.param("params/truck_info", Index_, 0); 

	UDPsend_.GROUP_ = ADDR_.c_str();
	UDPsend_.PORT_ = PORT_;
	UDPsend_.sendInit();

	UDPrecv_.GROUP_ = ADDR_.c_str();
	UDPrecv_.PORT_ = PORT_;
	UDPrecv_.recvInit();

}
void LRC::Save(){
	scale_truck_control::lrc msg;

	msg.index = Index_;
	msg.mode = Mode_;
	msg.lrc_vel = LrcVel_; 
	msg.alpha = Alpha_;

	lrcPublisher_.publish(msg);
}

void LRC::velCallback(const scale_truck_control::vel &msg){
	CurVel_ = msg.cur_vel;
	Alpha_ = msg.alpha;
}

void LRC::distCallback(const scale_truck_control::ctl &msg){
	CurDist_ = msg.cur_dist;
}

void LRC::send(){
	struct UDPsock::UDP_DATA udpDataLRC;

    udpDataLRC.index = Index_;
    udpDataLRC.to = 307;
    udpDataLRC.current_vel = CurVel_;
    udpDataLRC.current_dist = CurDist_;
	udpDataLRC.alpha = Alpha_;	
	udpDataLRC.mode = Mode_;

    UDPsend_.sendData(udpDataLRC);
}

void LRC::receive(){	
	struct UDPsock::UDP_DATA udpData;

    while(1) { 
        UDPrecv_.recvData(&udpData);
        if(udpData.index == (Index_ - 1)) {
            udpData_.target_vel = udpData.target_vel;
            TargetVel_ = udpData_.target_vel;
            TargetDist_ = udpData_.target_dist;
        }
        if(udpData.index == 307) {	//CRC index
            if(udpData.to == Index_) {
				udpData_.index = udpData.index;
				udpData_.CRCvel = udpData.crc_vel;	//receive crc_vel from CRC
				udpData_.target_vel = udpData.target_vel;
				udpData_.target_dist = udpData.target_dist;
            }
        }
    }
	
}

}
