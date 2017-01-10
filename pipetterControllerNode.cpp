#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdlib.h>
#include "PipetterController.h"

void loadParams(PipetterController *pc) {

}

void loadParams(ros::NodeHandle &nh, PipetterController &pc) {

    /* TODO: nam - Add value checks for all parameters loaded from user-defined
     * configuration files. Also consider restricting choices to allowed values
     * via GUI.- Tue 20 Dec 2016 11:59:36 AM MST */

    if(!nh.getParam("/pc_conf/type", pc.type)) {
        nh.getParam("/pc_defaults/type", pc.type);
        ROS_WARN("No parameter 'type' found in configuration file.\
                        Initializing pipetter with default value %s",\
                 pc.type);
    }
    if(!nh.getParam("/pc_conf/driver_name", pc.driver_name)) {
        nh.getParam("/pc_defaults/driver_name", pc.driver_name);
        ROS_WARN_STREAM("No parameter \"driver_name\" found in configuration file.\
                            Initializing pipetter with default value " <<\
                        pc.driver_name);
    }
    if(!nh.getParam("/pc_conf/driver_path", pc.driver_path)) {
        nh.getParam("/pc_defaults/driver_path", pc.driver_path);
        ROS_WARN_STREAM("No parameter \"driver_path\" found in configuration file.\
                                Initializing pipetter with default value " <<\
                        pc.driver_path);
    }
}

int main(int argc, char **argv) {
    const char *pcfile = "..pipetterConfig.conf";
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
