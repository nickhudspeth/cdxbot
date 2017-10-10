/************************************************************************
Title:    pipetterControllerNode.cpp
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     pipetterControllerNode.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file pipetterControllerNode.h, if available.

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
#include "PipetterModule.h"
#include "cdxbot/gc_cmd.h"
#include "cdxbot/pc_cmd.h"
#include "cdxbot/pipetterAspirate.h"
#include "cdxbot/pipetterDispense.h"
#include "cdxbot/pipetterEjectTip.h"
#include "cdxbot/pipetterHome.h"
#include "cdxbot/pipetterMakeContainerGeometry.h"
#include "cdxbot/pipetterMakeDeckGeometry.h"
#include "cdxbot/pipetterMakeLiquidClass.h"
#include "cdxbot/pipetterMoveZ.h"
#include "cdxbot/pipetterPickUpTip.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <stdlib.h>


/***********************    GLOBAL VARIABLES    ************************/
PipetterModule *pc;
std::string type;
std::string driver_path, driver_name;
void *driver_handle;
create_t *create_pm;
destroy_t *destroy_pm;

/*******************    FUNCTION IMPLEMENTATIONS    ********************/
void handleDebugMessages(const std::string &msg) {
    ROS_DEBUG("PipetterControllerNode: %s", msg.c_str());
}
void handleInfoMessages(const std::string &msg) {
    ROS_INFO("PipetterControllerNode: %s", msg.c_str());
}
void handleWarningMessages(const std::string &msg) {
    ROS_WARN("PipetterControllerNode: %s", msg.c_str());
}
void handleErrorMessages(const std::string &msg) {
    ROS_ERROR("PipetterControllerNode: %s", msg.c_str());
}

PipetterModule * loadDriver(std::string file) {
    char *error;
    void *driver_handle = dlopen(file.c_str(), RTLD_LAZY);
    if(driver_handle == NULL) {
        ROS_ERROR_STREAM("Error loading pipetter controller driver from " <<\
                         file);
        std::cerr << dlerror() << std::endl;
        //exit(-1);
        pc->deinit();
        ros::shutdown();
    }
    dlerror();
    ROS_INFO_STREAM("Successfully loaded pipetter controller driver from " <<\
                    file);
    create_pm = (create_t*)dlsym(driver_handle, "create");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading maker function.\n " << error);
        dlerror();
    }
    PipetterModule *pm = create_pm();

    destroy_pm = (destroy_t*)dlsym(driver_handle, "destroy");
    if((error = dlerror()) != NULL) {
        ROS_ERROR_STREAM("Error loading destroy function.\n " << error);
        dlerror();
    }
    pm->setDebugMsgCallback(handleDebugMessages);
    pm->setInfoMsgCallback(handleInfoMessages);
    pm->setWarningMsgCallback(handleWarningMessages);
    pm->setErrorMsgCallback(handleErrorMessages);

    return pm;
}


void loadParams(ros::NodeHandle &nh) {

    /* TODO: nam - Add value checks for all parameters loaded from user-defined
     * configuration files. Also consider restricting choices to allowed values
     * via GUI.- Tue 20 Dec 2016 11:59:36 AM MST */

    if(!nh.getParam("/pc_conf/type", type)) {
        nh.getParam("/pc_defaults/type", type);
        ROS_WARN("No parameter 'type' found in configuration file.\
                        Initializing pipetter with default value %s",\
                 type.c_str());
    }
    if(!nh.getParam("/pc_conf/driver_name", driver_name)) {
        nh.getParam("/pc_defaults/driver_name", driver_name);
        ROS_WARN_STREAM("No parameter \"driver_name\" found in configuration file.\
                            Initializing pipetter with default value " <<\
                        driver_name);
    }
    if(!nh.getParam("/pc_conf/driver_path", driver_path)) {
        nh.getParam("/pc_defaults/driver_path", driver_path);
        ROS_WARN_STREAM("No parameter \"driver_path\" found in configuration file.\
                                Initializing pipetter with default value " <<\
                        driver_path);
    }
    pc = loadDriver(driver_path + driver_name);

    if(!nh.getParam("/pc_conf/z_axis_enabled", pc->getZAxisEnabledRef())) {

        nh.getParam("/pc_defaults/z_axis_enabled", pc->getZAxisEnabledRef());
        ROS_WARN_STREAM("No parameter \"_z_axis_enabled\" found in configuration file.\
                                Initializing pipetter with default value " <<\
                        pc->getZAxisEnabled());
    }
    if(pc->getZAxisEnabled()) {
        /* The feed plane is only relevant if the pipetter has a mobile
         * z-axis. */
        if(!nh.getParam("/cdxbot/feed_plane", pc->getFeedPlaneRef())) {
            nh.getParam("/pc_defaults/feed_plane", pc->getFeedPlaneRef());
            ROS_WARN_STREAM("No parameter \"feed_plane\" found in configuration file.\
               Initializing pipetter with default value " << pc->getFeedPlane());
        }

    }
    if(!nh.getParam("/cdxbot/tip_pickup_speed", pc->getTipPickupSpeedRef())) {
        nh.getParam("/pc_defaults/pickup_speed",pc->getTipPickupSpeedRef());
        ROS_WARN_STREAM("No parameter \"tip_pickup_speed\" found in configuration file.\
               Initializing pipetter with default value " << pc->getTipPickupSpeed());
    }
}



void shutdownCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_WARN_STREAM("PipetterControllerNode: Received shutdown directive.");
    pc->deinit();
    ros::shutdown();
}

bool moveZCallback(cdxbot::pipetterMoveZ::Request &req,
                   cdxbot::pipetterMoveZ::Response &resp) {
    return pc->moveZ(req.pos, 1);
}

bool pickUpTipCallback(cdxbot::pipetterPickUpTip::Request &req,
                       cdxbot::pipetterPickUpTip::Response &resp) {
    ROS_INFO_STREAM("PipetterControllerNode: PickUpTipCallback entered.");
    bool success = pc->pickUpTip(req.tip_type_table_index,
                                 req.deck_geometry_table_index,
                                 req.tip_pickup_speed);
    ROS_INFO_STREAM("PipetterControllerNode: pickUpTipCallback: result = " << success);
    return success;
}

bool ejectTipCallback(cdxbot::pipetterEjectTip::Request & req,
                      cdxbot::pipetterEjectTip::Response &resp) {
    return pc->ejectTip();
}

bool homeCallback(cdxbot::pipetterHome::Request &req,
                  cdxbot::pipetterHome::Response &resp) {
    bool success = pc->home(req.init_z, req.init_dosing);
    ROS_INFO_STREAM("PipetterControllerNode: homing pipetter.");
    if(!success) {
        ROS_ERROR_STREAM("PipetterControllerNode: Could not home pipetter z-axis.");
    }
    return success;
}

bool aspirateCallback(cdxbot::pipetterAspirate::Request & req,
                      cdxbot::pipetterAspirate::Response &resp) {
    // pc->getCheckHeightRef() = req.check_height;
    pc->setLLDActive(true);
    return pc->aspirate(req.vol, req.gc_idx, req.dg_idx, req.lc_idx, req.liquid_surface);
}

bool dispenseCallback(cdxbot::pipetterDispense::Request & req,
                      cdxbot::pipetterDispense::Response &resp) {
    pc->setLLDActive(false);
    return pc->dispense(req.vol, req.gc_idx, req.dg_idx, req.lc_idx,  req.liquid_surface);
}

bool makeDeckGeometryCallback(cdxbot::pipetterMakeDeckGeometry::Request &req,
                              cdxbot::pipetterMakeDeckGeometry::Response &resp) {
    ROS_DEBUG_STREAM("PipetterControllerNode: MakeDeckGeometry callback entered.");
    pc->makeDeckGeometry(req.index, req.traverse_height, req.container_offset_z, req.engagement_length, req.tip_deposit_height);
    return true;
}

bool makeContainerGeometryCallback(cdxbot::pipetterMakeContainerGeometry::Request &req,
                                   cdxbot::pipetterMakeContainerGeometry::Response &resp) {
    ROS_DEBUG_STREAM("PipetterControllerNode: MakeContainerGeometry callback entered.");
    return pc->makeContainerGeometry(req.index, req.geometry, req.diameter, req.len_x,
                                     req.len_y, req.second_section_height, req.second_section,
                                     req.max_depth, req.bottom_search_offset, req.dispense_offset);
}

bool makeLiquidClassCallback(cdxbot::pipetterMakeLiquidClass::Request &req,
                             cdxbot::pipetterMakeLiquidClass::Response &resp) {
    ROS_DEBUG_STREAM("PipetterControllerNode: MakeLiquidClass callback entered.");
    ROS_DEBUG_STREAM("PipetterControllerNode: Creating liquid class at index " << req.index << ".");
    LiquidClass *l = pc->makeLiquidClass(req.name, req.index);
    ROS_DEBUG_STREAM("PipetterControllerNode: Call to pc->makeLiquidClass complete.");
    l->getADCRef() = req.adc;
    l->getAspirateBlowoutVolumeRef() = req.aspirate_blowout_volume;
    l->getAspirateSettlingTimeRef() = req.aspirate_settling_time;
    l->getAspirateSpeedRef() = req.aspirate_speed;
    l->getAspirateSwapSpeedRef() = req.aspirate_swap_speed;
    l->getAspirateTransportAirVolumeRef() = req.aspirate_transport_air_volume;
    l->getAspirateTypeRef() = req.aspirate_type;
    l->getCLLDSensitivityRef() = req.clld_sensitivity;
    l->getCutoffSpeedRef() = req.cutoff_speed;
    l->getDispenseBlowoutVolumeRef() = req.dispense_blowout_volume;
    l->getDispenseHeightRef() = req.dispense_height;
    l->getDispenseSettlingTimeRef() = req.dispense_settling_time;
    l->getDispenseSpeedRef() = req.dispense_speed;
    l->getDispenseSwapSpeedRef() = req.dispense_swap_speed;
    l->getDispenseTransportAirVolumeRef() = req.dispense_transport_air_volume;
    l->getDispenseTypeRef() = req.dispense_type;
    l->getImmersionDepthRef() = req.immersion_depth;
    l->getImmersionDirectionRef() = req.immersion_direction;
    l->getLLDHeightDifferenceRef() = req.lld_height_difference;
    l->getLLDModeRef() = req.lld_mode;
    l->getLeavingHeightRef() = req.leaving_height;
    l->getPLLDSensitivityRef() = req.plld_sensitivity;
    l->getPrewetVolumeRef() = req.prewet_volume;
    l->getStopBackVolumeRef() = req.stop_back_volume;
    l->getTransportSpeedRef() = req.transport_speed;
    bool res =  pc->setLiquidClass(req.index);
    if(!res) {
        ROS_ERROR_STREAM("PipetterControllerNode: Could not create liquid class\
                         at index " << req.index);
    }
    return res;
}


int main(int argc, char **argv) {
    // const char *pcfile = "..pipetterConfig.conf";
    geometry_msgs::Vector3Stamped msg;
    ros::init(argc, argv, "pipetterControllerNode");
    ros::NodeHandle nh;
    loadParams(nh);
    pc->init();
    /* Instantiate publishers and subscribers*/
    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>(\
                         "cdxbot/pipetter_zpos", 100);
    ros::Subscriber shutdown = nh.subscribe("/sd_pub", 100, &shutdownCallback);
    ROS_DEBUG_STREAM("Initialized pc with addr: " << &pc);
    /* Instantiate service servers */
    ros::ServiceServer moveZServer = nh.advertiseService("pipetter_move_z",
                                     &moveZCallback);
    ros::ServiceServer pickUpTipServer = nh.advertiseService("pipetter_pick_up_tip",
                                         &pickUpTipCallback);
    ros::ServiceServer ejectTipServer = nh.advertiseService("pipetter_eject_tip",
                                        &ejectTipCallback);
    ros::ServiceServer aspirateServer = nh.advertiseService("pipetter_aspirate",
                                        &aspirateCallback);
    ros::ServiceServer dispenseServer = nh.advertiseService("pipetter_dispense",
                                        &dispenseCallback);
    ros::ServiceServer makeDeckGeometryServer = nh.advertiseService("pipetter_make_deck_geometry",
            &makeDeckGeometryCallback);
    ros::ServiceServer makeContainerGeometrySever = nh.advertiseService("pipetter_make_container_geometry",
            &makeContainerGeometryCallback);
    ros::ServiceServer makeLiquidClassServer = nh.advertiseService("pipetter_make_liquid_class",
            &makeLiquidClassCallback);
    ros::ServiceServer homeServer = nh.advertiseService("pipetter_home", &homeCallback);
    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();
        msg.vector.x = 0;
        msg.vector.y = 0;
        msg.vector.z = pc->getZPos();
    // pub.publish(msg);
        rate.sleep();
    }
    return 0;
}
