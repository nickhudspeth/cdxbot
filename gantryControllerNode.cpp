/************************************************************************
Title:    gantry-controller.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     gantry-controller.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file gantry-controller.h.

LICENSE:
    Copyright (C) 2016 Nicholas Morrow

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <stdlib.h>
#include <iostream>
#include <atomic>
#include "std_msgs/String.h"
#include "GantryController.h"
#include "cdxbot/gc_cmd.h"
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
GantryController gc;

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
* Function :   loadParams()
* Purpose  :   Loads the pipetter configuration parameters from the
*              parameter server.
* Input    :   PipetterController gc
* Returns  :   void
*************************************************************************/
void loadParams(ros::NodeHandle &nh, GantryController &gc) {

    /* TODO: nam - Add value checks for all parameters loaded from user-defined
     * configuration files. Also consider restricting choices to allowed values
     * via GUI.- Tue 20 Dec 2016 11:59:36 AM MST */

    if(!nh.getParam("/gc_conf/type", gc.getTypeRef())) {
        nh.getParam("/gcdefaults/type", gc.getTypeRef());
        ROS_WARN("No parameter 'type' found in configuration file.\
                        Initializing gantry controller with default value %s",\
                 gc.getType());
    }
    if(!nh.getParam("/gc_conf/driver_name", gc.getDriverNameRef())) {
        nh.getParam("/gcdefaults/driver_name", gc.getDriverNameRef());
        ROS_WARN_STREAM("No parameter \"driver_name\" found in configuration file.\
                            Initializing gantry controller with default value " <<\
                        gc.getDriverName());
    }
    if(!nh.getParam("/gc_conf/driver_path", gc.getDriverPathRef())) {
        nh.getParam("/gcdefaults/driver_path", gc.getDriverPathRef());
        ROS_WARN_STREAM("No parameter \"driver_path\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getDriverPath());
    }

    if(!nh.getParam("/gc_conf/ip_address", gc.getIPAddressRef())) {
        nh.getParam("/gcdefaults/ip_address", gc.getIPAddressRef());
        ROS_WARN_STREAM("No parameter \"ip_address\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getIPAddress());
    }
    ROS_WARN_STREAM(" IP = " << gc.getIPAddress());



    if(!nh.getParam("/gc_conf/port", gc.getPortRef())) {
        nh.getParam("/gcdefaults/port", gc.getPortRef());
        ROS_WARN_STREAM("No parameter \"port\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getPort());
    }
    if(!nh.getParam("/gc_conf/port", gc.getTimeoutRef())) {
        nh.getParam("/gcdefaults/port", gc.getTimeoutRef());
        ROS_WARN_STREAM("No parameter \"timeout\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getTimeout());
    }
    if(!nh.getParam("/gc_conf/port", gc.getBufferSizeRef())) {
        nh.getParam("/gcdefaults/port", gc.getBufferSizeRef());
        ROS_WARN_STREAM("No parameter \"buffer_size\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getBufferSize());
    }
    if(!nh.getParam("/gc_conf/port", gc.getUnitsRef())) {
        nh.getParam("/gcdefaults/port", gc.getUnitsRef());
        ROS_WARN_STREAM("No parameter \"units\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getUnits());
    }
    if(!nh.getParam("/gc_conf/port", gc.getTraverseVelocityRef())) {
        nh.getParam("/gcdefaults/port", gc.getTraverseVelocityRef());
        ROS_WARN_STREAM("No parameter \"traverse_velocity\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getTraverseVelocity());
    }
    if(!nh.getParam("/gc_conf/port", gc.getRapidFeedVelocityRef())) {
        nh.getParam("/gcdefaults/port", gc.getRapidFeedVelocityRef());
        ROS_WARN_STREAM("No parameter \"rapid_feed_velocity\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getRapidFeedVelocity());
    }
    if(!nh.getParam("/gc_conf/port", gc.getXPosMinRef())) {
        nh.getParam("/gcdefaults/port", gc.getXPosMinRef());
        ROS_WARN_STREAM("No parameter \"xpos_min\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getXPosMin());
    }
    if(!nh.getParam("/gc_conf/port", gc.getXPosMaxRef())) {
        nh.getParam("/gcdefaults/port", gc.getXPosMaxRef());
        ROS_WARN_STREAM("No parameter \"xpos_max\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getXPosMax());
    }
    if(!nh.getParam("/gc_conf/port", gc.getYPosMinRef())) {
        nh.getParam("/gcdefaults/port", gc.getYPosMinRef());
        ROS_WARN_STREAM("No parameter \"ypos_min\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getYPosMin());
    }
    if(!nh.getParam("/gc_conf/port", gc.getYPosMaxRef())) {
        nh.getParam("/gcdefaults/port", gc.getYPosMaxRef());
        ROS_WARN_STREAM("No parameter \"ypos_max\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getYPosMax());
    }
    if(!nh.getParam("/gc_conf/port", gc.getZPosMinRef())) {
        nh.getParam("/gcdefaults/port", gc.getZPosMinRef());
        ROS_WARN_STREAM("No parameter \"zpos_min\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getZPosMin());
    }
    if(!nh.getParam("/gc_conf/port", gc.getZPosMaxRef())) {
        nh.getParam("/gcdefaults/port", gc.getZPosMaxRef());
        ROS_WARN_STREAM("No parameter \"zpos_max\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc.getZPosMax());
    }
}

void gcPubCallback(const cdxbot::gc_cmd msg) {

}

//ros::Subscriber shutdown = nh.subscribe("shutdown", 100, shutdownCallback);
void shutdownCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("GantryControllerNode: Received shutdown directive.");
    gc.driver_deinit();
    ros::shutdown();
}

int main(int argc, char **argv) {
    const char *gcfile = "./gantryConfig.conf";
    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "gantryControllerNode");
    ros::NodeHandle nh;
    loadParams(nh, gc);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gantry_pos", 1000);
    ros::Subscriber sub = nh.subscribe("gc_pub", 100, gcPubCallback);
    ros::Subscriber sd = nh.subscribe("/sd_pub", 1000, &shutdownCallback);
    ros::Rate rate(100);
    std::cout << "Initialized gc with addr: " << &gc << std::endl;
    gc.loadDriver();
    //gc.driver_init(gc);
    while(ros::ok()) {
        ros::spinOnce();
        msg.vector.x = gc.getPos('x');
        msg.vector.y = gc.getPos('y');
        msg.vector.z = gc.getPos('z');
        pos_pub.publish(msg);
        rate.sleep();
    }


    return 0;
}
