#include "socket/socket.h"
#include <ros/ros.h>
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/FriendReferee.h"
namespace roborts_referee{

void Service::ServerSend(datagive DG){
    write(clitfd, (char *)&DG, (sizeof(DG) + 2));
}

int Client::ClientInit(const char *CIP, uint16_t CPORT)
{
    memset(&serv_addr, 0, sizeof(serv_addr));

    if ((clitfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
        std::cout << "create socket failed:" << strerror(errno) << std::endl;
        return 0;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(CPORT);
    serv_addr.sin_addr.s_addr = inet_addr(CIP);

    std::cout << "try to connect ... " << std::endl;
    if (connect(clitfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1){
        std::cout << "connet failed : " << strerror(errno) << std::endl;
        return 0;
    }
    std::cout << "connect success !" << std::endl;
}

void Client::ClientReceived()
{
    int rdcnt = read(clitfd, (char *)&result, (sizeof(result) + 2));
    if (rdcnt == -1){
        perror(NULL);
        return;
    }

    if (rdcnt){
        std::cout << "(Client)recv : " << result.begin_ << std::endl;
    }
    else{
        std::cout << "Server has closed ! " << std::endl;
        std::cout << "Client will close..." << std::endl;
        return;
    }
}

int Service::ServerInit(const char *SIP, uint16_t SPORT)
{
    memset(&serv_addr, 0, sizeof(serv_addr));
    memset(&clit_addr, 0, sizeof(clit_addr));

    if ((servfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        std::cout << "create socket failed:" << strerror(errno) << std::endl;
        return 0;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SPORT);
    serv_addr.sin_addr.s_addr = inet_addr(SIP);

    if (bind(servfd, (sockaddr *)&serv_addr, sizeof(serv_addr)) == -1){
        std::cout << "bind failed:" << strerror(errno) << std::endl;
        return 0;
    }

    if (listen(servfd, 1024) == -1){
        std::cout << "listen failed:" << strerror(errno) << std::endl;
        return 0;
    }

    std::cout << "Init Success!" << std::endl;
    std::cout << "ip:" << inet_ntoa(serv_addr.sin_addr) << "port:" << ntohs(serv_addr.sin_port) << std::endl;
    std::cout << "Waiting for connecting..." << std::endl; 

    socklen_t clit_size = 0;

    if ((clitfd = accept(servfd, (sockaddr *)&clit_addr, &clit_size)) == -1){
        std::cout << "accept failed:" << std::endl;
        return 0;
    }

    std::cout << "Client success --ip:" << inet_ntoa(clit_addr.sin_addr) << "--port:" << ntohs(clit_addr.sin_port) << std::endl;

}

void GameStartStatusCallback(const roborts_msgs::GameStatusConstPtr& status){
    status_ = *status;

    if (status_.game_status == 4) {
        start_ = true;
    }
    else{
        start_ = false;
    }
}
void GetStart() const{
    return start_;
}
int main(int argc, char *argv[]){
    ros::init(argc, argv, "roborts_referee");
    ros::NodeHandle nh_;
    ros::Publisher publ_ = nh_.advertuse<roborts_msgs::FriendReferee>("friend_referee", 1);
    ros::Rate loop_rate(10);
    roborts_msgs::FriendReferee fr_;
    Service serv;
    datagive DG;
    DG = GetStart();
    serv.ServerInit("192.168.1.s", 14541);
    serv.ServerSend(DG);
    fr_.begin_ = DG.begin_;
    std::cout << "send!" << std::endl;
    publ_.publish(fr_);
    ros::spinOnce();
    loop_rate.sleep();
}
}