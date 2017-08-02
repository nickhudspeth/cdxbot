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
#include "GantryModule.h"
#include "cdxbot/gantryEStopToggle.h"
#include "cdxbot/gantryGetCurrentPosition.h"
#include "cdxbot/gantryHome.h"
#include "cdxbot/gantryMotorsToggle.h"
#include "cdxbot/gantryMove.h"
#include "cdxbot/gantrySetAccelerations.h"
#include "cdxbot/gantrySetAxisStepsPerUnit.h"
#include "cdxbot/gantrySetFeedrates.h"
#include "cdxbot/gantrySetUnits.h"
#include "cdxbot/gc_cmd.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <atomic>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
// #include "GantryController.h"
/*********************    CONSTANTS AND MACROS    **********************/
#define MOVE_MODE_ABSOLUTE 0
#define MOVE_MODE_RELATIVE 1

/***********************    GLOBAL VARIABLES    ************************/
GantryModule *gc;
std::string driver_path, driver_name;
void *driver_handle;
create_t *create_gm;
destroy_t *destroy_gm;
/*******************    FUNCTION IMPLEMENTATIONS    ********************/


GantryModule * loadDriver(std::string file) {
    char *error;
    void *driver_handle = dlopen(file.c_str(), RTLD_LAZY);
    if(driver_handle == NULL) {
        ROS_ERROR_STREAM("Error loading gantry controller driver from " <<\
                         file);
        std::cerr << dlerror() << std::endl;
        //exit(-1);
        gc->deinit();
        ros::shutdown();
    }
    dlerror();
    ROS_INFO_STREAM("Successfully loaded gantry controller driver from " <<\
                    file);
    create_gm = (create_t*)dlsym(driver_handle, "create");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading maker function.\n " << error);
        dlerror();
    }
    GantryModule *gm = create_gm();

    destroy_gm = (destroy_t*)dlsym(driver_handle, "destroy");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading destroy function.\n " << error);
        dlerror();
    }

    return gm;
}



/*************************************************************************
* Function :   loadParams()
* Purpose  :   Loads the gantry controller configuration parameters from the
*              parameter server.
* Input    :   ros::NodeHandle &nh
* Returns  :   void
*************************************************************************/
void loadParams(ros::NodeHandle &nh) {

    /* TODO: nam - Add value checks for all parameters loaded from user-defined
     * configuration files. Also consider restricting choices to allowed values
     * via GUI.- Tue 20 Dec 2016 11:59:36 AM MST */

    if(!nh.getParam("/gc_conf/driver_name", driver_name)) {
        nh.getParam("/gcdefaults/driver_name", driver_name);
        ROS_WARN("No parameter \"driver_name\" found in configuration file.\
                            Initializing gantry controller with default value\
                            %s", driver_name);
    }
    if(!nh.getParam("/gc_conf/driver_path", driver_path)) {
        nh.getParam("/gcdefaults/driver_path", driver_path);
        ROS_WARN_STREAM("No parameter \"driver_path\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        driver_path);
    }

    gc = loadDriver(driver_path + driver_name);

    if(!nh.getParam("/gc_conf/type", gc->getTypeRef())) {
        nh.getParam("/gcdefaults/type", gc->getTypeRef());
        ROS_WARN("No parameter 'type' found in configuration file.\
                        Initializing gantry controller with default value %s",\
                 gc->getType());
    }

    if(!nh.getParam("/gc_conf/ip_address", gc->getIPAddressRef())) {
        nh.getParam("/gcdefaults/ip_address", gc->getIPAddressRef());
        ROS_WARN_STREAM("No parameter \"ip_address\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getIPAddress());
    }



    if(!nh.getParam("/gc_conf/port", gc->getPortRef())) {
        nh.getParam("/gcdefaults/port", gc->getPortRef());
        ROS_WARN_STREAM("No parameter \"port\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getPort());
    }
    if(!nh.getParam("/gc_conf/timeout", gc->getTimeoutRef())) {
        nh.getParam("/gcdefaults/timeout", gc->getTimeoutRef());
        ROS_WARN_STREAM("No parameter \"timeout\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getTimeout());
    }
    if(!nh.getParam("/gc_conf/buffer_size", gc->getBufferSizeRef())) {
        nh.getParam("/gcdefaults/buffer_size", gc->getBufferSizeRef());
        ROS_WARN_STREAM("No parameter \"buffer_size\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getBufferSize());
    }
    // if(!nh.getParam("/gc_conf/units", gc->getUnitsRef())) {
    // ROS_WARN_STREAM("No parameter \"units\" found in configuration file.\
    // Initializing gantry controller with default value " <<\
    // gc->getUnits());
    // }
    if(!nh.getParam("/gc_conf/traverse_velocity", gc->getTraverseVelocityRef())) {
        nh.getParam("/gcdefaults/traverse_velocity", gc->getTraverseVelocityRef());
        ROS_WARN_STREAM("No parameter \"traverse_velocity\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getTraverseVelocity());
    }
    if(!nh.getParam("/gc_conf/rapid_feed_velocity", gc->getRapidFeedVelocityRef())) {
        nh.getParam("/gcdefaults/rapid_feed_velocity", gc->getRapidFeedVelocityRef());
        ROS_WARN_STREAM("No parameter \"rapid_feed_velocity\" found in configuration file.\
         Initializing gantry controller with default value " <<\
                        gc->getRapidFeedVelocity());
    }
    if(!nh.getParam("/gc_conf/xpos_min", gc->getXPosMinRef())) {
        nh.getParam("/gcdefaults/xpos_min", gc->getXPosMinRef());
        ROS_WARN_STREAM("No parameter \"xpos_min\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getXPosMin());
    }
    if(!nh.getParam("/gc_conf/xpos_max", gc->getXPosMaxRef())) {
        nh.getParam("/gcdefaults/xpos_max", gc->getXPosMaxRef());
        ROS_WARN_STREAM("No parameter \"xpos_max\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getXPosMax());
    }
    if(!nh.getParam("/gc_conf/ypos_min", gc->getYPosMinRef())) {
        nh.getParam("/gcdefaults/ypos_min", gc->getYPosMinRef());
        ROS_WARN_STREAM("No parameter \"ypos_min\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getYPosMin());
    }
    if(!nh.getParam("/gc_conf/ypos_max", gc->getYPosMaxRef())) {
        nh.getParam("/gcdefaults/ypos_max", gc->getYPosMaxRef());
        ROS_WARN_STREAM("No parameter \"ypos_max\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getYPosMax());
    }
    if(!nh.getParam("/gc_conf/zpos_min", gc->getZPosMinRef())) {
        nh.getParam("/gcdefaults/zpos_min", gc->getZPosMinRef());
        ROS_WARN_STREAM("No parameter \"zpos_min\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getZPosMin());
    }
    if(!nh.getParam("/gc_conf/zpos_max", gc->getZPosMaxRef())) {
        nh.getParam("/gcdefaults/zpos_max", gc->getZPosMaxRef());
        ROS_WARN_STREAM("No parameter \"zpos_max\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        gc->getZPosMax());
    }
    std::string tmp = "";
    if(!nh.getParam("/gc_conf/move_mode", tmp)) {
        nh.getParam("/gcdefaults/move_mode", tmp);
        ROS_WARN_STREAM("No parameter \"move_mode\" found in configuration file.\
                                Initializing gantry controller with default value " <<\
                        tmp);
    }
    if(tmp == "relative") {
        gc->setMoveMode(1);
    } else {
        gc->setMoveMode(0);
    }
}

void waitForGantry(const cdxbot::gc_cmd &msg) {
    double dist[] = {0, 0, 0};
    dist[0] = abs(gc->getPos('x') - msg.x);
    dist[1] = abs(gc->getPos('y') - msg.y);
    dist[2] = abs(gc->getPos('z') - msg.z);
    double max_dist = *std::max_element(dist, dist + 2);
    // Multiply by 60 since the velocity is stored in mm/min
    double travel_time =  (60 * max_dist) / gc->getTraverseVelocity();
    ROS_WARN_STREAM("Waiting " << travel_time << " seconds for gantry to arrive at destination...");
    usleep(travel_time * 100000);
    return;
}



void gcPubCallback(const cdxbot::gc_cmd &msg) {
    gc->setMoveMode(0);
    //ROS_DEBUG_STREAM("Got message: " << msg.cmd << msg.x << msg.y << msg.z << msg.vel);

    /* TODO: nam - Publish gantry status 1, and delay for calculated travel time
     * so that pipette motion is delayed until gantry is in place. Publish
     * gantry status 0 after delay. - Mon 10 Apr 2017 03:39:52 PM MDT */

    double coords[3] = {msg.x, msg.y, msg.z};
    /* Bounds check on all coordinates. If out of bounds, trim coordinate to
     * range and print error. THIS SHOULD PROBABLY NOT FAIL GRACEFULLY IN THE
     * PRODUCTION VERSION OF THIS SOFTWARE. */
    std::string modifier;
    std::string axis;
    double orig = 0;
    bool coord_err = 0;

    if(msg.x < gc->getXPosMin()) {
        coord_err = 1;
        orig = coords[0];
        coords[0] = gc->getXPosMin();
        axis = "x";
        modifier = "less";
    }
    if(coords[0] > gc->getXPosMax()) {
        coord_err = 1;
        orig = coords[0];
        coords[0] = gc->getXPosMax();
        axis = "x";
        modifier = "greater";
    }
    if(msg.y < gc->getYPosMin()) {
        coord_err = 1;
        orig = coords[1];
        coords[1] = gc->getYPosMin();
        axis = "y";
        modifier = "less";
    }
    if(coords[1] > gc->getYPosMax()) {
        coord_err = 1;
        orig = coords[1];
        coords[1] = gc->getYPosMax();
        axis = "y";
        modifier = "greater";
    }
    if(msg.z < gc->getZPosMin()) {
        coord_err = 1;
        orig = coords[2];
        coords[2] = gc->getZPosMin();
        axis = "z";
        modifier = "less";
    }
    if(coords[2] > gc->getZPosMax()) {
        coord_err = 1;
        orig = coords[2];
        coords[2] = gc->getZPosMax();
        axis = "z";
        modifier = "greater";
    }

    if(coord_err == 1) {
        ROS_ERROR_STREAM("GantryControllerNode:: Commanded "<< axis <<
                         "-coordinate (" << orig << ") is " << modifier
                         << " than the maximum allowable " << axis << " coordinate "
                         << gc->getXPosMin() << "." << std::endl);
        coord_err = 0;
    }

    if(msg.cmd == "move") {
        if(gc->getMoveMode() == MOVE_MODE_ABSOLUTE) {
            gc->moveAbsolute(coords[0], coords[1], coords[2]);
            waitForGantry(msg);
        } else if(gc->getMoveMode() == MOVE_MODE_RELATIVE) {
            gc->moveRelative(coords[0], coords[1], coords[2]);
            waitForGantry(msg);
        }
    } else if(msg.cmd == "wait") {
        gc->dwell(msg.time);
    } else if(msg.cmd == "movexy") {
        if(gc->getMoveMode() == MOVE_MODE_ABSOLUTE) {
            gc->moveAbsolute(coords[0], coords[1], 0);
            waitForGantry(msg);
        } else if(gc->getMoveMode() == MOVE_MODE_RELATIVE) {
            gc->moveRelative(coords[0], coords[1], 0);
            waitForGantry(msg);
        }
    } else if (msg.cmd == "movez") {
        if(gc->getMoveMode() == MOVE_MODE_ABSOLUTE) {
            gc->moveAbsolute(0, 0, coords[2]);
            waitForGantry(msg);
        } else if(gc->getMoveMode() == MOVE_MODE_RELATIVE) {
            gc->moveRelative(0, 0, coords[2]);
            waitForGantry(msg);
        }
    } else if(msg.cmd == "setvel") {
        if(msg.vel < 0) {
            gc->setTraverseVelocity(gc->getRapidFeedVelocity());
        } else {
            gc->setTraverseVelocity(msg.vel);
        }
        ROS_INFO_STREAM("Setting velocity to "<< gc->getTraverseVelocity() << " mm/min.");
    } else if(msg.cmd == "estop") {
        gc->emergencyStop();
    } else if(msg.cmd == "estoprst") {
        gc->emergencyStopReset();
    } else if(msg.cmd == "home") {
        gc->home();
        waitForGantry(msg);
    }
}

void shutdownCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_WARN_STREAM("GantryControllerNode: Received shutdown directive.");
    gc->deinit();
    ros::shutdown();
}

bool moveCallback(cdxbot::gantryMove::Request &req,
                  cdxbot::gantryMove::Response &resp) {
    /* Should bounds-check coordinates here */
    if(!req.move_mode) {
        gc->moveAbsolute(req.x, req.y, req.z);
    } else {
        gc->moveRelative(req.x, req.y, req.z);
    }
    // while(  (gc->getPos('x') != req.x) |
            // (gc->getPos('y') != req.y) |
            // (gc->getPos('z') != req.z)) {
        /* WAIT FOR GANTRY TO ARIVE */
    // }

    ROS_INFO_STREAM("LEAVING MOVE CALLBACK");
    resp.ok = true;
    return true;
}

bool eStopToggleCallback(cdxbot::gantryEStopToggle::Request &req,
                         cdxbot::gantryEStopToggle::Response &resp) {
    if(!req.state) {
        gc->emergencyStopReset();
    } else {
        gc->emergencyStop();
    }
    return (resp.ok = true);
}

bool homeCallback(cdxbot::gantryHome::Request &req,
                  cdxbot::gantryHome::Response &resp) {
    ROS_INFO_STREAM("ENTERED HOME CALLBACK");
    if(req.all){
        gc->home(AXIS_ALL);
    }
    if(req.x) {
        gc->home(AXIS_X);
    }
    if(req.y) {
        gc->home(AXIS_Y);
    }
    if(req.z) {
        gc->home(AXIS_Z);
    }
    ROS_INFO_STREAM("LEAVING HOME CALLBACK");
    resp.ok = true;
    return true;
    // return (resp.ok = true);
}

bool motorsToggleCallback(cdxbot::gantryMotorsToggle::Request &req,
                          cdxbot::gantryMotorsToggle::Response &resp) {
    if(!req.x_status) {
        gc->motorsDisable(AXIS_X);
    } else {
        gc->motorsEnable(AXIS_X);
    }
    if(!req.y_status) {
        gc->motorsDisable(AXIS_Y);
    } else {
        gc->motorsEnable(AXIS_Y);
    }
    if(!req.z_status) {
        gc->motorsDisable(AXIS_Z);
    } else {
        gc->motorsEnable(AXIS_Z);
    }

}

bool setUnitsCallback(cdxbot::gantrySetUnits::Request &req,
                      cdxbot::gantrySetUnits::Response &resp) {
    if(!req.units) {
        gc->setUnits(UNITS_MM);
    } else {
        gc->setUnits(UNITS_IN);
    }

}

bool setAxisStepsPerUnitCallback(cdxbot::gantrySetAxisStepsPerUnit::Request &req,
                                 cdxbot::gantrySetAxisStepsPerUnit::Response &resp) {
    if(req.axis == 0) {
        gc->setAxisStepsPerUnit(AXIS_X, req.steps);
    } else if(req.axis == 1) {
        gc->setAxisStepsPerUnit(AXIS_Y, req.steps);
    } else if(req.axis == 2) {
        gc->setAxisStepsPerUnit(AXIS_Z, req.steps);
    } else if(req.axis == 3) {
        gc->setAxisStepsPerUnit(AXIS_ALL, req.steps);
    } else {
        /* Error: Invalid axis specified. */
        return false;
    }
    return true;
}

bool getCurrentPositionCallback(cdxbot::gantryGetCurrentPosition::Request &req,
                                cdxbot::gantryGetCurrentPosition::Response &resp) {
    resp.x = gc->getPos(AXIS_X);
    resp.y = gc->getPos(AXIS_Y);
    resp.z = gc->getPos(AXIS_Z);

}

bool setFeedratesCallback(cdxbot::gantrySetFeedrates::Request &req,
                          cdxbot::gantrySetFeedrates::Response &resp) {
    if((req.x_current_percentage < 0) ||
            (req.y_current_percentage < 0) ||
            (req.z_current_percentage < 0)) {
        /* ERROR: Current percentage out of bounds. */
        resp.ok = false;
        return false;
    }
    if(req.set_min) {
        gc->setMinFeedrate(AXIS_X, req.x_min);
        gc->setMinFeedrate(AXIS_Y, req.y_min);
        gc->setMinFeedrate(AXIS_Z, req.z_min);
    }
    if(req.set_max) {
        gc->setMaxFeedrate(AXIS_X, req.x_max);
        gc->setMaxFeedrate(AXIS_Y, req.y_max);
        gc->setMaxFeedrate(AXIS_Z, req.z_max);
    }
    if(req.set_cur) {
        gc->setMaxFeedrate(AXIS_X, req.x_current_percentage*req.x_max);
        gc->setMaxFeedrate(AXIS_Y, req.y_current_percentage*req.y_max);
        gc->setMaxFeedrate(AXIS_Z, req.z_current_percentage*req.z_max);
    }
    resp.ok = true;
    return true;
}

bool setAccelerationsCallback(cdxbot::gantrySetAccelerations::Request &req,
                              cdxbot::gantrySetAccelerations::Response &resp) {
    if((req.x_current_percentage < 0) ||
            (req.y_current_percentage < 0) ||
            (req.z_current_percentage < 0)) {
        /* ERROR: Current percentage out of bounds. */
        resp.ok = false;
        return false;
    }
    if(req.set_min) {
        gc->setMinAccel(AXIS_X, req.x_min);
        gc->setMinAccel(AXIS_Y, req.y_min);
        gc->setMinAccel(AXIS_Z, req.z_min);
    }
    if(req.set_max) {
        gc->setMaxAccel(AXIS_X, req.x_max);
        gc->setMaxAccel(AXIS_Y, req.y_max);
        gc->setMaxAccel(AXIS_Z, req.z_max);
    }
    if(req.set_cur) {
        gc->setMaxAccel(AXIS_X, req.x_current_percentage*req.x_max);
        gc->setMaxAccel(AXIS_Y, req.y_current_percentage*req.y_max);
        gc->setMaxAccel(AXIS_Z, req.z_current_percentage*req.z_max);
    }
    resp.ok = true;
    return true;
}



int main(int argc, char **argv) {
    //const char *gcfile = "./gantryConfig.conf";

    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "gantryControllerNode");
    ros::NodeHandle nh;
    loadParams(nh);
    if(gc->init() < 0) {
        ROS_ERROR_STREAM("GC Init return code: " << gc->init());
        return -1;
    }
    /* CREATE PUBLISHERS / SUBSCRIBERS */
    ros::Subscriber shutdown = nh.subscribe("/sd_pub", 100, &shutdownCallback);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Vector3Stamped>("gantry_pos", 1000);
    ros::Publisher gc_status_pub = nh.advertise<std_msgs::Bool>("gantry_status", 1000);

    // ros::Subscriber sub = nh.subscribe("/gc_pub", 100, &gcPubCallback);

    /* INSTANTIATE SERVICE SERVERS */
    ros::ServiceServer gantryMoveServer = nh.advertiseService("gantry_move", &moveCallback);
    ros::ServiceServer gantryEStopToggle = nh.advertiseService("gantry_estop_toggle", &eStopToggleCallback);
    ros::ServiceServer gantryHome = nh.advertiseService("gantry_home", &homeCallback);
    ros::ServiceServer gantryMotorsToggle = nh.advertiseService("gantry_motors_toggle", &motorsToggleCallback);
    ros::ServiceServer gantrySetUnits = nh.advertiseService("gantry_set_units", &setUnitsCallback);
    ros::ServiceServer gantrySetAxisStepsPerUnitServer = nh.advertiseService("gantry_set_axis_steps_per_unit", &setAxisStepsPerUnitCallback);
    ros::ServiceServer gantryGetCurrentPositionServer = nh.advertiseService("gantry_get_current_position", &getCurrentPositionCallback);
    ros::ServiceServer gantrySetFeedratesServer = nh.advertiseService("gantry_set_feedrates", &setFeedratesCallback);
    ros::ServiceServer gantrySetAccelerationsServer = nh.advertiseService("gantry_set_accelerations", &setAccelerationsCallback);
    ros::Rate rate(100);
    ROS_DEBUG_STREAM("Initialized gc with addr: " << &gc);
    //gc.driver_init(gc);
    while(ros::ok()) {
        ros::spinOnce();
        msg.vector.x = gc->getPos('x');
        msg.vector.y = gc->getPos('y');
        msg.vector.z = gc->getPos('z');
        pos_pub.publish(msg);
        rate.sleep();
    }

    return 0;
}
