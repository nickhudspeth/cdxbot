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
#include "GantryController.h"
#include "cdxbot/gc_cmd.h"
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

/*************************************************************************
* Function :   loadParams()
* Purpose  :   Loads the pipetter configuration parameters from the
*              parameter server.
* Input    :   GantryController *gc
* Returns  :   void
*************************************************************************/
void loadParams(ros::NodeHandle &nh, GantryController &gc) {

    /* TODO: nam - Add value checks for all parameters loaded from user-defined
     * configuration files. Also consider restricting choices to allowed values
     * via GUI.- Tue 20 Dec 2016 11:59:36 AM MST */

    if(!nh.getParam("/gc/type", gc.type)) {
        nh.getParam("/gcdefaults/type", gc.type);
        ROS_WARN("No parameter 'type' found in configuration file.\
                        Initializing gantry controller with default value %s",\
                 gc.type);

        if(!nh.getParam("/gc/cominterface", gc.cominterface)) {
            nh.getParam("/gcdefaults/cominterface", gc.cominterface);
            ROS_WARN_STREAM("No parameter \"cominterface\" found in configuration file.\
                            Initializing gantry controller with default value " <<\
                            gc.cominterface);

            if(!nh.getParam("/gc/cominterface", gc.cominterface)) {
                nh.getParam("/gcdefaults/cominterface", gc.cominterface);
                ROS_WARN_STREAM("No parameter \"cominterface\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                                gc.cominterface);
            }

            /* Load ethernet configuration parameters only if ethernet interface used.*/
            if(gc.cominterface == "ethernet") {
                if(!nh.getParam("/gc/ipaddress", gc.ipaddress)) {
                    nh.getParam("/gcdefaults/ipaddress", gc.ipaddress);
                    ROS_WARN_STREAM("No parameter \"ipaddress\" found in configuration file.\
                                    Initializing gantry controller with default value " <<\
                                    gc.ipaddress);
                }
                if(!nh.getParam("/gc/port", gc.port)) {
                    nh.getParam("/gcdefaults/port", gc.port);
                    ROS_WARN_STREAM("No parameter \"port\" found in configuration file.\
                                        Initializing gantry controller with default value " <<\
                                    gc.port);
                }
                if(!nh.getParam("/gc/timeout", gc.timeout)) {
                    nh.getParam("/gcdefaults/timeout", gc.timeout);
                    ROS_WARN_STREAM("No parameter \"timeout\" found in configuration file.\
                                        Initializing gantry controller with default value " <<\
                                    gc.timeout);
                }

            }
        }
        /* Load usb configuration parameters only if usb interface used.*/
        else if(gc.cominterface == "usb") {

        }
    }

}

int main(int argc, char **argv) {
    const char *gcfile = "./gantryConfig.conf";
    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "gantryControllerNode");
    ros::NodeHandle nh;
    GantryController gc(gcfile);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gantry_pos", 1000);
    ros::Rate rate(100);
    loadParams(nh, gc);

    while(ros::ok()) {
        msg.vector.x = gc.getPos('x');
        msg.vector.y = gc.getPos('y');
        msg.vector.z = gc.getPos('z');
        pos_pub.publish(msg);
        rate.sleep();
    }


    return 0;
}
