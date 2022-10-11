#ifndef ROBORTS_REFEREE_SOCKET_H
#define ROBORTS_REFEREE_SOCKET_H

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <sys/socket.h>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>


namespace roborts_referee{

struct datagive{
    bool begin_ = false;
};


class Service{
 public:
    ~Service() {
        close(servfd);
        close(clitfd);
    };

    int ServerInit(const char *SIP, uint16_t SPORT);

    void ServerSend(datagive DG);
    int servfd, clitfd;
    struct sockaddr_in serv_addr, clit_addr;
};

class Client{
 public:
    ~Client(){
        close(clitfd);
    };

    int ClientInit(const char *CIP, uint16_t CPORT);
    void ClientReceived();
    int clitfd;
    struct sockaddr_in serv_addr;
    datagive result;
};

}
#endif //ROBORTS_REFEREE_SOCKET_H