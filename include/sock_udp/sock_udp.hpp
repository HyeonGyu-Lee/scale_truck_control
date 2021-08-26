#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

namespace UDPsock{

#pragma pack(1)
struct UDP_DATA{
    int index;
    float target_vel;
    float current_vel;
    float target_dist;
    float current_dist;
};
#pragma pack(4)

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
