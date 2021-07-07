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
class UDPsocket {
public:
    UDPsocket();
    ~UDPsocket();
    int recvInit();
    int sendInit();
    int recvData(float* Data);
    int sendData(float Data);
    const char* GROUP_;
    int PORT_;

private:
    int fd_;
    struct sockaddr_in addr_;
    struct ip_mreq mreq_;
};
}