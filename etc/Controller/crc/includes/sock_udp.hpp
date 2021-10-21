#pragma once

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

namespace UDPsock{

struct lane_coef{
    float a;
    float b;
    float c;
};

struct UDP_DATA{
    int index;
    int to;
    int sync;
    int cf;
    float target_vel;
    float current_vel;
	float predict_vel;
    float target_dist;
    float current_dist;
    float current_angle;
    float roi_dist;
	bool alpha;
	bool beta;
	bool gamma;
	uint8_t mode;
    struct lane_coef coef[3];
};

class UDPsocket {
public:
    UDPsocket();
    ~UDPsocket();
    int recvInit();
    int sendInit();
    int recvData(struct UDP_DATA*);
    int sendData(struct UDP_DATA);
    const char* GROUP_;
    int PORT_;

private:
    int fd_;
    struct sockaddr_in addr_;
    struct ip_mreq mreq_;
};
}
