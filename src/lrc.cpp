#include "lrc/lrc.hpp"

#define PATH "/home/avees/catkin_ws/logfiles/"

using namespace std;

namespace LocalResiliencyCoordinator{

LocalRC::LocalRC(ros::NodeHandle nh)
	: nodeHandle_(nh), UDPsend_(), UDPrecv_(){
	
	init();	

}

LocalRC::~LocalRC(){
	{
		const std::lock_guard<std::mutex> lock(mutexNodeStatus_);
		isNodeRunning_ = false;
	}

	spinThread_.join();
	udprecvThread_.join();
}

void LocalRC::init(){
	isNodeRunning_ = true;

	std::string XavSubTopicName;
	int XavSubQueueSize;
	std::string OcrSubTopicName;
	int OcrSubQueueSize;
	std::string XavPubTopicName;
	int XavPubQueueSize;
	std::string OcrPubTopicName;
	int OcrPubQueueSize;

	nodeHandle_.param("params/truck_info", Index_, 0);

	nodeHandle_.param("LrcParams/udp_group_addr", ADDR_, std::string("239.255.255.250"));
	nodeHandle_.param("LrcParams/udp_group_port", PORT_, 9392);	
	nodeHandle_.param("LrcParams/lrc_log_path", PATH_, std::string("/home/jetson/catkin_ws/logfiles/"));
	nodeHandle_.param("LrcParams/epsilon", Epsilon_, 1.0f);
	nodeHandle_.param("LrcParams/lu_ob_A", A_, 0.6817f);
	nodeHandle_.param("LrcParams/lu_ob_B", B_, 0.3183f);
	nodeHandle_.param("LrcParams/lu_ob_L", L_, 0.2817f);
	nodeHandle_.param("LrcParams/enable_console_output", EnableConsoleOutput_, true);

	/******************************/
	/* ROS Topic Subscribe Option */
	/******************************/
	nodeHandle_.param("LrcSubPub/xavier_to_lrc/topic", XavSubTopicName, std::string("/xav2lrc_msg"));
	nodeHandle_.param("LrcSubPub/xavier_to_lrc/queue_size", XavSubQueueSize, 1);
	nodeHandle_.param("LrcSubPub/ocr_to_lrc/topic", OcrSubTopicName, std::string("/ocr2lrc_msg"));
	nodeHandle_.param("LrcSubPub/ocr_to_lrc/queue_size", OcrSubQueueSize, 1);

	/******************************/
	/* ROS Topic Publish Option */
	/******************************/
	nodeHandle_.param("LrcSubPub/lrc_to_xavier/topic", XavPubTopicName, std::string("/lrc2xav_msg"));
	nodeHandle_.param("LrcSubPub/lrc_to_xavier/queue_size", XavSubQueueSize, 1);
	nodeHandle_.param("LrcSubPub/lrc_to_ocr/topic", OcrPubTopicName, std::string("/lrc2ocr_msg"));
	nodeHandle_.param("LrcSubPub/lrc_to_ocr/queue_size", OcrPubQueueSize, 1);

	/************************/
	/* ROS Topic Subscriber */ 
	/************************/
	XavSubscriber_ = nodeHandle_.subscribe(XavSubTopicName, XavSubQueueSize, &LocalRC::XavCallback, this);
	OcrSubscriber_ = nodeHandle_.subscribe(OcrSubTopicName, OcrSubQueueSize, &LocalRC::OcrCallback, this);

	/************************/
	/* ROS Topic Publisher */ 
	/************************/
	XavPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc2xav>(XavPubTopicName, XavPubQueueSize);
	OcrPublisher_ = nodeHandle_.advertise<scale_truck_control::lrc2ocr>(OcrPubTopicName, OcrPubQueueSize);

	/*****************/
	/* UDP Multicast */
	/*****************/
	UDPsend_.GROUP_ = ADDR_.c_str();
	UDPsend_.PORT_ = PORT_;
	UDPsend_.sendInit();

	UDPrecv_.GROUP_ = ADDR_.c_str();
	UDPrecv_.PORT_ = PORT_;
	UDPrecv_.recvInit();
	
	/*********************/
	/* spin & udp thread */
	/*********************/
	spinThread_ = std::thread(&LocalRC::spin, this);
	udprecvThread_ = std::thread(&LocalRC::UDPrecvInThread, this);
}

bool LocalRC::isNodeRunning(){
	const std::lock_guard<std::mutex> lock(mutexNodeStatus_);
	return isNodeRunning_;
}


void LocalRC::XavCallback(const scale_truck_control::xav2lrc &msg){
	const std::lock_guard<std::mutex> lock(mutexXavCallback_);
	AngleDegree_ = msg.steer_angle;
	CurDist_ = msg.cur_dist;
	TarDist_ = msg.tar_dist;
	TarVel_ = msg.tar_vel;
	Beta_ = msg.beta;
	Gamma_ = msg.gamma;
}

void LocalRC::OcrCallback(const scale_truck_control::ocr2lrc &msg){
	const std::lock_guard<std::mutex> lock(mutexOcrCallback_);
	CurVel_ = msg.cur_vel;
	SatVel_ = msg.u_k;	//saturated velocity
}

void LocalRC::LrcPub(){
	scale_truck_control::lrc2xav xav;
	scale_truck_control::lrc2ocr ocr;
	
	xav.cur_vel = CurVel_;
	ocr.index = Index_;
	ocr.steer_angle = AngleDegree_;
	ocr.cur_dist = CurDist_;
	ocr.tar_dist = TarDist_;
	ocr.tar_vel = TarVel_;
	ocr.pred_vel = PredVel_;
	ocr.alpha = Alpha_;

	XavPublisher_.publish(xav);
	OcrPublisher_.publish(ocr);
}

void LocalRC::UDPsendFunc()
{
    struct UDPsock::UDP_DATA udpData;

    udpData.index = Index_;
    udpData.to = 100;	//CRC index
    udpData.alpha = Alpha_;
    udpData.beta = Beta_;
    udpData.gamma = Gamma_;
    udpData.current_vel = CurVel_;
    udpData.current_dist = CurDist_;
    udpData.mode = LrcMode_;

    UDPsend_.sendData(udpData);
}

void* LocalRC::UDPrecvInThread()
{
	struct UDPsock::UDP_DATA udpData;
	
	while(ros::ok()) { 
		UDPrecv_.recvData(&udpData);
		if(udpData.index == 100 && udpData.to == Index_) {	//CRC index	
			PredVel_ = udpData.predict_vel;
			CrcMode_ = udpData.mode;
			if (CrcMode_ >= LrcMode_){
				LrcMode_ = CrcMode_;
			}
		}
	}
}

void LocalRC::VelocitySensorCheck(){
	{
		const std::lock_guard<std::mutex> lock(mutexOcrCallback_);
		HatVel_ = A_ * HatVel_ + B_ * SatVel_ + L_ * (CurVel_ - HatVel_);
	}
	if(fabs(CurVel_ - HatVel_) > Epsilon_){
		Alpha_ = true;
	}
/*	
	else{	//Recovery
		Alpha_ = false;
	}
*/
}

void LocalRC::ModeCheck(uint8_t crc_mode){
	if(!Index_){	//LV
		if(Beta_){	//Camera sensor failure
			LrcMode_ = 2;	//GDM
		}
		else if((Alpha_ || Gamma_) && (crc_mode == 0 || crc_mode == 1)){
			LrcMode_ = 1;	//RCM
		}
		else if(crc_mode == 0){
			LrcMode_ = 0;	//TM
		}
	}
	else{	//FV1, FV2
		if(Beta_ && Gamma_){
			LrcMode_ = 2;	
		}
		else if((Alpha_ || Beta_ || Gamma_) && (crc_mode == 0 || crc_mode == 1)){
			LrcMode_ = 1;	
		}
		else if(crc_mode == 0){
			LrcMode_ = 0;
		}
	}
}

void LocalRC::RecordData(struct timeval *startTime){
	struct timeval currentTime;
	char file_name[] = "LRC_log00.csv";
	static char file[128] = {0x00, };
	char buf[256] = {0x00,};
	static bool flag = false;
	ifstream read_file;
	ofstream write_file;
	if(!flag){
		for(int i = 0; i < 100; i++){
			file_name[7] = i/10 + '0';	//ASCII
			file_name[8] = i%10 + '0';
			sprintf(file, "%s%s", PATH_.c_str(), file_name);
			read_file.open(file);
			if(read_file.fail()){	//Check if the file exists
				read_file.close();
				write_file.open(file);
				break;
			}
			read_file.close();
		}
		write_file << "Time[s],Predict,Target,Current,Saturation,Estimate,Alpha" << endl;
		flag = true;
	}
	else{
		gettimeofday(&currentTime, NULL);
		Time_ = ((currentTime.tv_sec - startTime->tv_sec)) + ((currentTime.tv_usec - startTime->tv_usec)/1000000.0);
		sprintf(buf, "%.3e,%.3f,%.3f,%.3f,%.3f,%.3f,%d", Time_, PredVel_, TarVel_, CurVel_, SatVel_, fabs(CurVel_ - HatVel_), Alpha_);
		write_file.open(file, std::ios::out | std::ios::app);
		write_file << buf << endl;
	}
	write_file.close();
}

void LocalRC::PrintData(){
	static int cnt = 0;
	if (cnt > 100 && EnableConsoleOutput_){
		printf("\nEstimated Velocity:\t%.3f", fabs(CurVel_ - HatVel_));
		printf("\nPredict Velocity:\t%.3f", PredVel_);
		printf("\nTarget Velocity:\t%.3f", TarVel_);
		printf("\nCurrent Velocity:\t%.3f", CurVel_);
		printf("\nSaturated Velocity:\t%.3f", SatVel_);
		printf("\nEstimated Value:\t%.3f", fabs(CurVel_ - HatVel_));
		printf("\nalpha, beta, gamma:\t%d, %d, %d", Alpha_, Beta_, Gamma_); 
		printf("\nMODE:\t%d", LrcMode_);
		printf("\n");
		cnt = 0;
	}
	cnt++;
}

void LocalRC::spin(){
	struct timeval startTime;
	gettimeofday(&startTime, NULL);
	while(ros::ok()){
		VelocitySensorCheck();
		ModeCheck(CrcMode_);
		LrcPub();
		UDPsendFunc();
		RecordData(&startTime);
		PrintData();

		if(!isNodeRunning()){
			ros::requestShutdown();
			break;
		}
	}
}

}
