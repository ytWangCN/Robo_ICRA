#include <ros/ros.h>
#include "socket/socket.h"
#include "roborts_msgs/FriendReferee.h"

namespace roborts_referee {
int main(int argc, char *argv[]) {
    datagive DG;
    Client serv;
    ros::init(argc, argv, "roborts_referee");
    ros::NodeHandle nh;
    ros::Publisher pub_ = nh.advertise<roborts_msgs::FriendReferee>("friendreferee", 1);
    roborts_msgs::FriendReferee fr_;

    serv.ClientInit("192.168.1.x", 14541);
    while (ros::ok()){
        serv.ClientReceived();
        fr_.begin_ = serv.result.begin_;
        pub_.publish(fr_);
    }
    
}
}