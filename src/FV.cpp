#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    /*
    if (argc != 3) {
       printf("Command line args should be multicast group and port\n");
       printf("(e.g. for SSDP, `sender 239.255.255.250 1900`)\n");
       return 1;
    }

    char* group = argv[1]; // e.g. 239.255.255.250 for SSDP
    int port = atoi(argv[2]); // 0 if error, which is an invalid port
    */
    const char* group = "239.255.255.250"; // e.g. 239.255.255.250 for SSDP
    int port = atoi("1900"); // 0 if error, which is an invalid port


    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(group);
    addr.sin_port = htons(port);

    while (1) {
        float vel;
        int addrlen = sizeof(addr);
        int nbytes = recvfrom(
            fd,
            &vel,
            sizeof(vel),
            0,
            (struct sockaddr *) &addr,
            (socklen_t*)&addrlen
        );
        if (nbytes < 0) {
            perror("recvfrom");
            return 1;
        }
        printf("%f\n",vel);
     }


    return 0;
}
