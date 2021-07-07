#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

class UDPsocket {
public:
    UDPsocket(const char* GROUP, int PORT);
    ~UDPsocket();
    int recvInit();
    int sendInit();
    int recvData(float* Data);
    int sendData(float Data);

private:
    int fd_;
    const char* GROUP_;
    int PORT_;
    struct sockaddr_in addr_;
    struct ip_mreq mreq_;
};