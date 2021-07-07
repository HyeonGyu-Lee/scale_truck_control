#include "sock_udp/sock_udp.h"

UDPsocket::UDPsocket(const char* IP_addr, int PORT)
{
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    
    if (fd_ < 0) {
        perror("socket");
        return 1;
    }
    GROUP_ = IP_addr;
    PORT_ = PORT;
}

UDPsocket::~UDPsocket()
{
    close(fd_);
    exit(0);
}

void UDPsocket::recvInit()
{
    u_int yes = 1;
    if (setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, (char*) &yes, sizeof(yes)) < 0)
    {
       perror("Reusing ADDR failed");
       return 1;
    }

    memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
    addr_.sin_port = htons(PORT_);

    if (bind(fd_, (struct sockaddr*) &addr_, sizeof(addr_)) < 0)
    {
        perror("bind");
        return 1;
    }

    mreq_.imr_multiaddr.s_addr = inet_addr(group);
    mreq_.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq_, sizeof(mreq_)) < 0)
    {
        perror("setsockopt");
        return 1;
    }
}

void UDPsocket::sendInit()
{
    memset(&addr_, 0, sizeof(addr_));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(GROUP_);
    addr.sin_port = htons(PORT_);
}

float UDPsocket::recvData(d)
{
    float Data;
    int addrlen = sizeof(addr_);
    int nbytes = recvfrom(fd_, &Data, sizeof(float), 0, (struct sockaddr *) &addr_, (socklen_t*)&addrlen);
    if (nbytes < 0) {
        perror("recvfrom");
        return 1;
    }
    return Data;
}

void UDPsocket::sendData(float Data)
{
    int nbytes = sendto(fd_, &Data, sizeof(float), 0, (struct sockaddr*) &addr_, sizeof(addr_));
    if (nbytes < 0)
    {
        perror("sendto");
        return 1;
    }
}