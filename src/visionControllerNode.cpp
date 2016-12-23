#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdlib.h>
#include "VisionController.h"

void loadParams(VisionController *vc){

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "visionControllerNode");
    ros::NodeHandle nh;
    VisionController *vc;
    while(ros::ok()) {

    }

    return 0;
}
