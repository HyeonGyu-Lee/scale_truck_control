#include "lrc/lrc.hpp"

using namespace std;

namespace LocalResiliencyCoordinator{

LocalRC::LocalRC(ros::NodeHandle nh)
	: nodeHandle_(nh), UDPsend_(), UDPrecv_(){
	
	init();	

}

LocalRC::~LocalRC(){
	
}

void LocalRC::init(){
	std::string lrcSubTopicName;
	int lrcSubQueueSize;
	std::string ctlSubTopicName;
	int ctlSubQueueSize;
	std::string lrcPubTopicName;
	int lrcPubQueueSize;

	/******************************/
	/* ROS Topic Subscribe Option */
	/******************************/
	nodeHandle_.param("subscribers/lrc_reading/topic", lrcSubTopicName, std::string("/lrc_msg"));
	nodeHandle_.param("subscribers/lrc_reading/queue_size", lrcSubQueueSize, 1);
//	nodeHandle_.param("publishers/control_data/topic", ctlSubTopicName, std::string("/ctl_msg"));
//	nodeHandle_.param("publishers/control_data/queue_size", ctlSubQueueSize, 1);

	/******************************/
	/* ROS Topic Publish Option */
	/******************************/
	nodeHandle_.param("publishers/lrc_data/topic", lrcPubTopicName, std::string("/lrc_msg"));
	nodeHandle_.param("publishers/lrc_data/queue_size", lrcPubQueueSize, 1);

	/************************/
	/* ROS Topic Subscriber */ 
	/************************/
	lrcSubscriber_ = nodeHandle_.subscribe(lrcSubTopicName, lrcSubQueueSize, &LocalRC::lrcCallback, this);
//	ctlSubscriber_ = nodeHandle_.subscribe(ctlSubTopicName, ctlSubQueueSize, &LocalRC::ctlCallback, this);

	/************************/
	/* ROS Topic Publisher */ 
	/************************/
	lrcPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc>(lrcPubTopicName, lrcPubQueueSize);

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
void LocalRC::lrcPub(){
	scale_truck_control::lrc lrc;

	lrc.crc_vel = CrcVel_;
	lrc.alpha = Alpha_;

	lrcPublisher_.publish(lrc);
}

//void LocalRC::ctlCallback(const scale_truck_control::ctl &msg){
//	CurDist_ = msg.cur_dist;
//}

void LocalRC::lrcCallback(const scale_truck_control::lrc &lrc){
	CurVel_ = lrc.cur_vel;
	Alpha_ = lrc.alpha;
}

void LocalRC::udpSend(){
	struct UDPsock::UDP_DATA udpDataLRC;

    udpDataLRC.index = Index_;
    udpDataLRC.to = 100;
    udpDataLRC.current_vel = CurVel_;
    udpDataLRC.current_dist = CurDist_;
	udpDataLRC.alpha = Alpha_;

    UDPsend_.sendData(udpDataLRC);
}

void LocalRC::udpRecv(){	
	struct UDPsock::UDP_DATA udpData;
 
	UDPrecv_.recvData(&udpData);
	if(udpData.index == (Index_ - 1) && (Mode_ == 0)) {	//TM mode
		udpData_.target_vel = udpData.target_vel;
		TargetVel_ = udpData_.target_vel;
		TargetDist_ = udpData_.target_dist;
	}
	if(udpData.index == 307) {	//Control Center
		if(udpData.to == Index_) {
			udpData_.index = udpData.index;
			udpData_.target_vel = udpData.target_vel;
			udpData_.target_dist = udpData.target_dist;
			udpData_.sync = udpData.sync;
			udpData_.cf = udpData.cf;

			TargetVel_ = udpData_.target_vel;
			TargetDist_ = udpData_.target_dist;
		}

	}
	if(udpData.index == 100) {	//CRC
		if(udpData.to == Index_) {
			udpData_.crc_vel = udpData.crc_vel;	//receive crc_vel from CRC
			udpData_.Mode = udpData.Mode;	
			CrcVel_ = udpData_.crc_vel;
			Mode_ = udpData.Mode;
			if (Mode_ == 1){	//RCM mode
				TargetVel_ = CrcVel_;
			}
			else if (Mode_ == 2){	//GDM mode
				TargetVel_ = 0;
			}
		}
	}	
}

}
