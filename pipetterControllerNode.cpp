#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdlib.h>
#include "PipetterController.h"

void loadParams(PipetterController *pc) {

}


int main(int argc, char **argv) {
    const char *gcfile = "..gantryConfig.conf";
    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "pipetterControllerNode");
    ros::NodeHandle nh;
    PipetterController pc;
    /* Instantiate publishers and subscribers*/
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>(\
                         "cdxbot/pipetter_zpos", 100);
    ros::Rate rate(100);
    while(ros::ok()) {
        msg.vector.x = 0;
        msg.vector.y = 0;
        msg.vector.z = pc.getZPos();
        rate.sleep();
    }
    return 0;
}
