#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdlib.h>
#include "PipetterController.h"

void loadParams(PipetterController *pc){

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pipetterControllerNode");
    ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(\
    // "cdxbot/pipetter_zpos", 100);
    while(ros::ok()) {

    }
    return 0;
}
