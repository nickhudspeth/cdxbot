/************************************************************************
Title:    cdxbot.cpp
Author:   Nicholas Morrow <nmorrow@crystaldiagnostics.com>
File:     cdxbot.cpp
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)
Usage:    Refer to the header file cdxbot.h.

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
#include "CDXBot.h"
#include "cdxbot/gantryEmergencyStop.h"
#include "cdxbot/gantryGetCurrentPosition.h"
#include "cdxbot/gantryHome.h"
#include "cdxbot/gantryMotorsToggle.h"
#include "cdxbot/gantryMove.h"
#include "cdxbot/gantrySetAccelerations.h"
#include "cdxbot/gantrySetAxisStepsPerUnit.h"
#include "cdxbot/gantrySetFeedrates.h"
#include "cdxbot/gantrySetUnits.h"
#include "cdxbot/gc_cmd.h"
#include "cdxbot/gc_cmd_s.h"
#include "cdxbot/nodeInit.h"
#include "cdxbot/nodeShutdown.h"
#include "cdxbot/pc_cmd.h"
#include "cdxbot/pc_cmd_s.h"
#include "cdxbot/pipetterAspirate.h"
#include "cdxbot/pipetterDispense.h"
#include "cdxbot/pipetterEjectTip.h"
#include "cdxbot/pipetterEmergencyStop.h"
#include "cdxbot/pipetterHome.h"
#include "cdxbot/pipetterMakeContainerGeometry.h"
#include "cdxbot/pipetterMakeDeckGeometry.h"
#include "cdxbot/pipetterMakeLiquidClass.h"
#include "cdxbot/pipetterMoveZ.h"
#include "cdxbot/pipetterPickUpTip.h"
#include "cdxbot/pipetterSetLLDActive.h"
#include "cdxbot/sc_cmd.h"
#include "cdxbot/pipetterGetContainerVolume.h"
#include "cdxbot/shakerReset.h"
#include "cdxbot/shakerSetFreq.h"
#include "cdxbot/shakerSetPower.h"
#include "cdxbot/shakerStart.h"
#include "cdxbot/shakerStop.h"
#include "cdxbot/vc_cmd.h"
#include "cdxbot/vc_cmd_s.h"
#include "commands.h"
#include "common.h"
#include "qt_startcommand/startcommand.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <boost/bind.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <ros/ros.h>
#include <stdbool.h>
#include <string>

/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
CDXBot cd;

/* Create subscriber for GUI commands */
ros::Subscriber guiSub;

/* Create publishers for the various controller */
// ros::Publisher gc_pub; //= nh.advertise<cdx
// ros::Publisher pc_pub;
// ros::Publisher sc_pub;
// ros::Publisher vc_pub;
// ros::Publisher shutdown_pub;

/* Create service clients */
ros::ServiceClient gantryEmergencyStopClient;
ros::ServiceClient gantryGetCurrentPositionClient;
ros::ServiceClient gantryHomeClient;
ros::ServiceClient gantryInitClient;
ros::ServiceClient gantryMotorsToggleClient;
ros::ServiceClient gantryMoveClient;
ros::ServiceClient gantrySetAccelerationsClient;
ros::ServiceClient gantrySetAxisStepsPerUnitClient;
ros::ServiceClient gantrySetFeedratesClient;
ros::ServiceClient gantrySetUnitsClient;
ros::ServiceClient gantryShutdownClient;
ros::ServiceClient gcClient;
ros::ServiceClient pcClient;
ros::ServiceClient pipetterAspirateClient;
ros::ServiceClient pipetterDispenseClient;
ros::ServiceClient pipetterEjectTipClient;
ros::ServiceClient pipetterEmergencyStopClient;
ros::ServiceClient pipetterHomeClient;
ros::ServiceClient pipetterInitClient;
ros::ServiceClient pipetterMakeContainerGeometryClient;
ros::ServiceClient pipetterMakeDeckGeometryClient;
ros::ServiceClient pipetterMakeLiquidClassClient;
ros::ServiceClient pipetterMoveZClient;
ros::ServiceClient pipetterPickUpTipClient;
ros::ServiceClient pipetterShutdownClient;
ros::ServiceClient pipetterSetLLDActiveClient;
ros::ServiceClient shakerInitClient;
ros::ServiceClient shakerResetClient;
ros::ServiceClient shakerSetFreqClient;
ros::ServiceClient shakerSetPowerClient;
ros::ServiceClient shakerStartClient;
ros::ServiceClient shakerStopClient;
ros::ServiceClient shakerShutdownClient;
ros::ServiceClient vcClient;
ros::ServiceClient pipetterGetContainerVolumeClient;

bool run_flag = false; /* Has node already been started by UI? If so, ignore incoming start requests. */
/*******************    FUNCTION IMPLEMENTATIONS    ********************/

int loadConfig(ros::NodeHandle nh, CDXBot &cd) {
    char buf[64];
    memset(buf, ' ',64);
    int ret = 0;

    nh.getParam("/cdxbot/num_containers", ret);
    // printf("Read value of %d for num_containers.\n", ret);
    if(!nh.getParam("/cdxbot/num_containers", ret)) {
        nh.getParam("/cdxbot_defaults/num_containers", ret);
        ROS_WARN("No parameter 'num_containers' found in configuration file.\
                        Initializing cdxbot with default value %s",\
                 std::to_string(cd.getNumContainers()));
    }
    cd.setNumContainers(ret);
    memset(buf, ' ',64);
    sprintf(buf,"/cdxbot/pipetter_has_z");
    if(!nh.getParam(buf, cd.getPipetterHasZRef())) {
        nh.getParam("/cdxbot_defaults/pipetter_has_z", cd.getPipetterHasZRef());
        ROS_WARN("No parameter pipetter_has_z found in configuration file.\
                        Initializing cdxbot with default value %s",\
                 std::to_string(cd.getPipetterHasZRef()));
    }
    memset(buf, ' ',64);
    sprintf(buf,"/cdxbot/feed_plane");
    if(!nh.getParam(buf, cd.getFeedPlaneHeightRef())) {
        nh.getParam("/cdxbot_defaults/feed_plane", cd.getFeedPlaneHeightRef());
        ROS_WARN("No parameter feed_plane found in configuration file.\
                        Initializing cdxbot with default value %s",\
                 std::to_string(cd.getFeedPlaneHeightRef()));
    }
    memset(buf, ' ',64);
    sprintf(buf,"/cdxbot/eject_x");
    if(!nh.getParam(buf, cd.getEjectPosRef(AXIS_X))) {
        nh.getParam("/cdxbot_defaults/eject_x", cd.getEjectPosRef(AXIS_X));
        ROS_WARN_STREAM("No parameter eject_x found in configuration file. Initializing cdxbot with default value" << cd.getEjectPos(AXIS_X));
    }

    memset(buf, ' ',64);
    sprintf(buf,"/cdxbot/eject_y");
    if(!nh.getParam(buf, cd.getEjectPosRef(AXIS_Y))) {
        nh.getParam("/cdxbot_defaults/eject_y", cd.getEjectPosRef(AXIS_Y));
        ROS_WARN_STREAM("No parameter eject_y found in configuration file. Initializing cdxbot with default value" << cd.getEjectPos(AXIS_Y));
    }

    for(unsigned int i=0; i < cd.getNumContainers(); i++) {
        sprintf(buf,"/cdxbot/containers/c%d/type", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTypeRef())) {
            nh.getParam("/cdxbot_defaults/type",cd.getContainer(i).getTypeRef());
            ROS_WARN("No parameter containers:%d:type found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getType());
        }
        // printf("Container %d initialized with type %s.\n", i, cd.getContainer(i).getType().c_str());

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/len_y", i);
        if(!nh.getParam(buf, cd.getContainer(i).getLengthRef('y'))) {
            nh.getParam("/cdxbot_defaults/len_y", cd.getContainer(i).getLengthRef('y'));
            ROS_WARN("No parameter containers:%d:len_y found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getLength('y')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/len_x", i);
        if(!nh.getParam(buf, cd.getContainer(i).getLengthRef('x'))) {
            nh.getParam("/cdxbot_defaults/len_x", cd.getContainer(i).getLengthRef('x'));
            ROS_WARN("No parameter containers:%d:len_x found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getLengthRef('x')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/len_z", i);
        if(!nh.getParam(buf, cd.getContainer(i).getLengthRef('z'))) {
            nh.getParam("/cdxbot_defaults/len_z", cd.getContainer(i).getLengthRef('z'));
            ROS_WARN("No parameter containers:%d:len_z found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getLengthRef('z')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/offset_x", i);
        if(!nh.getParam(buf, cd.getContainer(i).getOffsetRef('x'))) {
            nh.getParam("/cdxbot_defaults/offset_x", cd.getContainer(i).getOffsetRef('x'));
            ROS_WARN("No parameter containers:%d:offset_x found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getOffsetRef('x')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/offset_y", i);
        if(!nh.getParam(buf, cd.getContainer(i).getOffsetRef('y'))) {
            nh.getParam("/cdxbot_defaults/offset_y", cd.getContainer(i).getOffsetRef('y'));
            ROS_WARN("No parameter containers:%d:offset_y found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getOffsetRef('y')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/offset_z", i);
        if(!nh.getParam(buf, cd.getContainer(i).getOffsetRef('z'))) {
            nh.getParam("/cdxbot_defaults/offset_z", cd.getContainer(i).getOffsetRef('z'));
            ROS_WARN("No parameter containers:%d:offset_z found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getOffsetRef('z')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/rows", i);
        if(!nh.getParam(buf, cd.getContainer(i).getRowsRef())) {
            nh.getParam("/cdxbot_defaults/rows", cd.getContainer(i).getRowsRef());
            ROS_WARN("No parameter containers:%d:rows found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getRowsRef()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/cols", i);
        if(!nh.getParam(buf, cd.getContainer(i).getColsRef())) {
            nh.getParam("/cdxbot_defaults/cols", cd.getContainer(i).getColsRef());
            ROS_WARN("No parameter containers:%d:cols found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getColsRef()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/row_spacing", i);
        if(!nh.getParam(buf, cd.getContainer(i).getRowSpacingRef())) {
            nh.getParam("/cdxbot_defaults/row_spacing", cd.getContainer(i).getRowSpacingRef());
            ROS_WARN("No parameter containers:%d:row_spacing found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getRowSpacingRef()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/col_spacing", i);
        if(!nh.getParam(buf, cd.getContainer(i).getColSpacingRef())) {
            nh.getParam("/cdxbot_defaults/col_spacing", cd.getContainer(i).getColSpacingRef());
            ROS_WARN("No parameter containers:%d:col_spacing found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getColSpacingRef()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/traverse_height", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTraverseHeightRef())) {
            nh.getParam("/cdxbot_defaults/traverse_height", cd.getContainer(i).getTraverseHeightRef());
            ROS_WARN("No parameter containers:%d:traverse_height found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getTraverseHeightRef()));
        }

        /* PARAMETERS SPECIFIC TO TIP-HOLDING CONTAINERS */
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/tip_type_index", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTipTypeRef())) {
            nh.getParam("/cdxbot_defaults/tip_type_index", cd.getContainer(i).getTipTypeRef());
            ROS_WARN("No parameter containers:%d:tip_type_index found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getTipType()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/tip_engagement_len", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTipEngagementLenRef())) {
            nh.getParam("/cdxbot_defaults/tip_engagement_len", cd.getContainer(i).getTipEngagementLenRef());
            ROS_WARN("No parameter containers:%d:tip_engagement_len found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getTipEngagementLen()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/tip_deposit_height", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTipDepositHeightRef())) {
            nh.getParam("/cdxbot_defaults/tip_deposit_height", cd.getContainer(i).getTipDepositHeightRef());
            ROS_WARN("No parameter containers:%d:tip_deposit_height found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getTipDepositHeight()));
        }


        /* PARAMETERS SPECIFIC TO WELL-TYPE CONTAINERS */
        /* Only well-type containers require container geometry parameter
         * specification. */
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/volume", i);
        if(!nh.getParam(buf, cd.getContainer(i).getVolumeRef())) {
            nh.getParam("/cdxbot_defaults/volume", cd.getContainer(i).getVolumeRef());
            ROS_WARN("No parameter containers:%d:volume found in configuration file.\
            Initializing cdxbot with default value %f",\
                     i, std::to_string(cd.getContainer(i).getVolumeRef()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/well_geometry", i);
        if(!nh.getParam(buf, cd.getContainer(i).getWellGeometryRef())) {
            nh.getParam("/cdxbot_defaults/well_geometry", cd.getContainer(i).getWellGeometryRef());
            ROS_WARN("No parameter containers:%d:well_geometry found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getWellGeometry());
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/well_diameter", i);
        if(!nh.getParam(buf, cd.getContainer(i).getWellDiameterRef())) {
            nh.getParam("/cdxbot_defaults/well_diameter", cd.getContainer(i).getWellDiameterRef());
            ROS_WARN("No parameter containers:%d:well_diameter found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getWellDiameter()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/well_len_x", i);
        if(!nh.getParam(buf, cd.getContainer(i).getWellLengthRef('x'))) {
            nh.getParam("/cdxbot_defaults/well_len_x", cd.getContainer(i).getWellLengthRef('x'));
            ROS_WARN("No parameter containers:%d:well_len_x found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getWellLength('x')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/well_len_y", i);
        if(!nh.getParam(buf, cd.getContainer(i).getWellLengthRef('y'))) {
            nh.getParam("/cdxbot_defaults/well_len_y", cd.getContainer(i).getWellLengthRef('y'));
            ROS_WARN("No parameter containers:%d:well_len_y found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getWellLength('y')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/well_len_z", i);
        if(!nh.getParam(buf, cd.getContainer(i).getWellLengthRef('z'))) {
            nh.getParam("/cdxbot_defaults/well_len_y", cd.getContainer(i).getWellLengthRef('z'));
            ROS_WARN("No parameter containers:%d:well_len_z found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getWellLength('z')));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/second_section", i);
        if(!nh.getParam(buf, cd.getContainer(i).getSecondSectionRef())) {
            nh.getParam("/cdxbot_defaults/second_section", cd.getContainer(i).getSecondSectionRef());
            ROS_WARN("No parameter containers:%d:second_section found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getSecondSection()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/second_section_height", i);
        if(!nh.getParam(buf, cd.getContainer(i).getSecondSectionHeightRef())) {
            nh.getParam("/cdxbot_defaults/second_section_height", cd.getContainer(i).getSecondSectionHeightRef());
            ROS_WARN("No parameter containers:%d:second_section_height found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getSecondSectionHeight()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/bottom_search_offset", i);
        if(!nh.getParam(buf, cd.getContainer(i).getBottomSearchOffsetRef())) {
            nh.getParam("/cdxbot_defaults/bottom_search_offset", cd.getContainer(i).getBottomSearchOffsetRef());
            ROS_WARN("No parameter containers:%d:bottom_search_offset found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getBottomSearchOffset()));
        }
        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/dispense_offset", i);
        if(!nh.getParam(buf, cd.getContainer(i).getDispensePositionOffsetRef())) {
            nh.getParam("/cdxbot_defaults/dispense_offset", cd.getContainer(i).getDispensePositionOffsetRef());
            ROS_WARN("No parameter containers:%d:dispense_offset found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, std::to_string(cd.getContainer(i).getDispensePositionOffset()));
        }
        cdxbot::pipetterMakeContainerGeometry::Request pmcgreq;
        cdxbot::pipetterMakeContainerGeometry::Response  pmcgresp;
        pmcgreq.index = i;
        pmcgreq.geometry = (cd.getContainer(i).getWellGeometry() == "square") ? 1 : 0;
        pmcgreq.diameter = cd.getContainer(i).getWellDiameter();
        pmcgreq.len_x = cd.getContainer(i).getWellLength('x');
        pmcgreq.len_y = cd.getContainer(i).getWellLength('y');
        pmcgreq.len_z = cd.getContainer(i).getWellLength('z');
        pmcgreq.second_section_height = cd.getContainer(i).getSecondSectionHeight();
        pmcgreq.second_section =  cd.getContainer(i).getSecondSection();
        pmcgreq.max_depth = cd.getContainer(i).getOffset('z') - cd.getContainer(i).getWellLength('z');
        // pmcgreq.max_depth = cd.getContainer(i).getWellLength('z');
        pmcgreq.bottom_search_offset = cd.getContainer(i).getBottomSearchOffset();
        pmcgreq.dispense_offset = cd.getContainer(i).getDispensePositionOffset();
        if(!pipetterMakeContainerGeometryClient.call(pmcgreq, pmcgresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to create container geometry table entry for container " << i << ".");
        } else {
            ROS_DEBUG_STREAM("CDXBotNode: Created container geometry table entry for container " << i << "\n" \
                             << "\tIndex =\t" << pmcgreq.index <<"\n" \
                             << "\tGeometry =\t" << static_cast<unsigned int >(pmcgreq.geometry) << "\n" \
                             << "\tDiameter =\t" << pmcgreq.diameter << "\n" \
                             << "\tLen_X =\t" << pmcgreq.len_x << "\n" \
                             << "\tLen_Y =\t" << pmcgreq.len_y << "\n" \
                             << "\tSecond Section Height =\t" << pmcgreq.second_section_height << "\n" \
                             << "\tSecond Section =\t" << pmcgreq.second_section << "\n" \
                             << "\tMax Immerson Depth =\t" << pmcgreq.max_depth << "\n" \
                             << "\tBottom Search Offset =\t" << pmcgreq.bottom_search_offset << "\n" \
                             << "\tDispense Offset =\t" << pmcgreq.dispense_offset << "\n"\
                             << "\tZ-Length = \t" << pmcgreq.len_z << "\n" \
                             << "\tWell length = \t" << cd.getContainer(i).getWellLength('z') << "\n"\
                             << "\tmin_end_cmd_height = \t" << cd.getContainer(i).getLength('z') - cd.getContainer(i).getWellLength('z'));
        }
        cd.getContainer(i).setContainerGeometryTableIndex(i);
        /* Make deck geometry entry for containers of all types */
        cdxbot::pipetterMakeDeckGeometry::Request pmdgreq;
        cdxbot::pipetterMakeDeckGeometry::Response pmdgresp;
        pmdgreq.index = i;
        // pmdgreq.traverse_height = cd.getContainer(i).getTraverseHeight();
        pmdgreq.traverse_height = cd.getContainer(i).getOffset('z') - cd.getContainer(i).getWellLength('z');
        pmdgreq.min_end_cmd_height = cd.getContainer(i).getOffset('z') - cd.getContainer(i).getWellLength('z');
        pmdgreq.container_offset_z = cd.getContainer(i).getOffset('z');
        pmdgreq.engagement_length = cd.getContainer(i).getTipEngagementLen();
        pmdgreq.tip_deposit_height = cd.getContainer(i).getTipDepositHeight();
        if(!pipetterMakeDeckGeometryClient.call(pmdgreq, pmdgresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to create deck geometry table entry for container " << i << ".");
        } else {
            ROS_DEBUG_STREAM("CDXBotNode: Created deck geometry table entry for container " << i << "\n" \
                             << "\tIndex =\t" << pmdgreq.index <<"\n" \
                             << "\tTraverse height =\t" << pmdgreq.traverse_height << "\n" \
                             << "\tContainer offset Z =\t" << pmdgreq.container_offset_z << "\n" \
                             << "\tEngagement length =\t" << pmdgreq.engagement_length << "\n" \
                             << "\tTip deposit height =\t" << pmdgreq.tip_deposit_height);
        }
        cd.getContainer(i).setDeckGeometryTableIndex(i);
        /* INITIALIZE CONTAINER CELL MATRIX */
        for(unsigned int j =0; j < cd.getContainer(i).getRows(); j++) {
            std::vector<struct container_cell> newRow;
            for(unsigned int k=0; k < cd.getContainer(i).getCols(); k++) {
                struct container_cell c;
                memset(&c, 0, sizeof(struct container_cell));
                if(cd.getContainer(i).getType() == "well") {
                    memset(buf, ' ',64);
                    sprintf(buf,"/cdxbot/containers/c%d/volume", i);
                    if(!nh.getParam(buf, c.vol_max)) {
                        nh.getParam("/cdxbot_defaults/volume", c.vol_max);
                        ROS_WARN("No parameter containers:%d:volume found in configuration file.\
                        Initializing cdxbot with default value %f",\
                                 i, std::to_string(c.vol_max));
                    }
                    memset(buf, ' ',64);
                    sprintf(buf,"/cdxbot/containers/c%d/well_len_x", i);
                    if(!nh.getParam(buf, c.len_x)) {
                        nh.getParam("/cdxbot_defaults/well_len_x", c.len_x);
                        ROS_WARN("No parameter containers:%d:well_len_x found in configuration file.\
                        Initializing cdxbot with default value %s",\
                                 i, std::to_string(c.len_x));
                    }
                    memset(buf, ' ',64);
                    sprintf(buf,"/cdxbot/containers/c%d/well_len_y", i);
                    if(!nh.getParam(buf, c.len_y)) {
                        nh.getParam("/cdxbot_defaults/well_len_y", c.len_y);
                        ROS_WARN("No parameter containers:%d:well_len_y found in configuration file.\
                        Initializing cdxbot with default value %s",\
                                 i, std::to_string(c.len_y));
                    }
                    memset(buf, ' ',64);
                    sprintf(buf,"/cdxbot/containers/c%d/well_len_z", i);
                    if(!nh.getParam(buf, c.len_z)) {
                        nh.getParam("/cdxbot_defaults/well_len_z", c.len_z);
                        ROS_WARN("No parameter containers:%d:well_len_z found in configuration file.\
                        Initializing cdxbot with default value %s",\
                                 i, std::to_string(c.len_z));
                    }

                } else if(cd.getContainer(i).getType() == "tip") {
                    memset(buf, ' ', 64);
                    sprintf(buf,"/cdxbot/containers/c%d/tip_type_index", i);
                    if(!nh.getParam(buf, c.tt_index)) {
                        c.tt_index = 0;
                        ROS_WARN("No parameter containers:%d:tip_type_index found in configuration file.\
                        Initializing cdxbot with default value %s",\
                                 i, std::to_string(c.tt_index));
                    }
                }
                newRow.push_back(c);
            }
            cd.getContainer(i).getCellsVecRef().push_back(newRow);
        }
    }

    /* LOAD LIQUID CLASS PARAMETERS */
    unsigned int num_liquids = 0;
    cdxbot::pipetterMakeLiquidClass::Request pmlcreq;
    cdxbot::pipetterMakeLiquidClass::Response pmlcresp;
    memset(buf, ' ', 64);
    sprintf(buf,"/liquids/l%d/name", num_liquids);
    int tmpint = 0;
    double tmpf64 = 0.0f;
    std::string tmpstr = "";
    bool tmpbool = false;
    if(!nh.hasParam(buf)) {
        ROS_ERROR_STREAM("Liquid configuration file parsing is not working.");
    }
    while(nh.hasParam(buf)) {
        pmlcreq.index = num_liquids;
        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/name", num_liquids);
        if(!nh.getParam(buf, tmpstr)) {
            ROS_WARN("No parameter liquids:l%d:name found in configuration file.", num_liquids);
        }
        pmlcreq.name = tmpstr;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/adc", num_liquids);
        if(!nh.getParam(buf, tmpint)) {
            ROS_WARN("No parameter liquids:l%d:adc found in configuration file.", num_liquids);
        }
        pmlcreq.adc = tmpint;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/aspirate_blowout_volume", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:aspirate_blowout_volume found in configuration file.", num_liquids);
        }
        pmlcreq.aspirate_blowout_volume  = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/aspirate_settling_time", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:aspirate_settling_time found in configuration file.", num_liquids);
        }
        pmlcreq.aspirate_settling_time = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/aspirate_speed", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:aspirate_speed found in configuration file.", num_liquids);
        }
        pmlcreq.aspirate_speed = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/aspirate_swap_speed", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:aspirate_swap_speed found in configuration file.", num_liquids);
        }
        pmlcreq.aspirate_swap_speed = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/aspirate_type", num_liquids);
        if(!nh.getParam(buf, tmpint)) {
            ROS_WARN("No parameter liquids:l%d:aspirate_type found in configuration file.", num_liquids);
        }
        pmlcreq.aspirate_type = tmpint;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/clld_sensitivity", num_liquids);
        if(!nh.getParam(buf, tmpint)) {
            ROS_WARN("No parameter liquids:l%d:clld_sensitivity found in configuration file.", num_liquids);
        }
        pmlcreq.clld_sensitivity = tmpint;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/immersion_depth", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:immersion_depth found in configuration file.", num_liquids);
        }
        pmlcreq.immersion_depth = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/immersion_direction", num_liquids);
        if(!nh.getParam(buf, tmpint)) {
            ROS_WARN("No parameter liquids:l%d:immersion_direction found in configuration file.", num_liquids);
        }
        pmlcreq.immersion_direction = tmpint;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/lld_height_difference", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:lld_height_difference found in configuration file.", num_liquids);
        }
        pmlcreq.lld_height_difference = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/lld_mode", num_liquids);
        if(!nh.getParam(buf, tmpint)) {
            ROS_WARN("No parameter liquids:l%d:lld_mode found in configuration file.", num_liquids);
        }
        pmlcreq.lld_mode = tmpint;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/plld_sensitivity", num_liquids);
        if(!nh.getParam(buf, tmpint)) {
            ROS_WARN("No parameter liquids:l%d:plld_sensitivity found in configuration file.", num_liquids);
        }
        pmlcreq.plld_sensitivity = tmpint;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/prewet_volume", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:prewet_volume found in configuration file.", num_liquids);
        }
        pmlcreq.prewet_volume = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/aspirate_transport_air_volume", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:aspirate_transport_air_volume found in configuration file.", num_liquids);
        }
        pmlcreq.aspirate_transport_air_volume = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/dispense_blowout_volume", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:dispense_blowout_volume found in configuration file.", num_liquids);
        }
        pmlcreq.dispense_blowout_volume = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/cutoff_speed", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:cutoff_speed found in configuration file.", num_liquids);
        }
        pmlcreq.cutoff_speed = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/dispense_height", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:dispense_height found in configuration file.", num_liquids);
        }
        pmlcreq.dispense_height = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/dispense_settling_time", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:dispense_settling_time found in configuration file.", num_liquids);
        }
        pmlcreq.dispense_settling_time = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/dispense_speed", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:dispense_speed found in configuration file.", num_liquids);
        }
        pmlcreq.dispense_speed = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/dispense_swap_speed", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:dispense_swap_speed found in configuration file.", num_liquids);
        }
        pmlcreq.dispense_swap_speed = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/dispense_type", num_liquids);
        if(!nh.getParam(buf, tmpint)) {
            ROS_WARN("No parameter liquids:l%d:dispense_type found in configuration file.", num_liquids);
        }
        pmlcreq.dispense_type = tmpint;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/leaving_height", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:leaving_height found in configuration file.", num_liquids);
        }
        pmlcreq.leaving_height = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/stop_back_volume", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:stop_back_volume found in configuration file.", num_liquids);
        }
        pmlcreq.stop_back_volume = tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/dispense_transport_air_volume", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:dispense_transport_air_volume found in configuration file.", num_liquids);
        }
        pmlcreq.dispense_transport_air_volume =  tmpf64;

        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/transport_speed", num_liquids);
        if(!nh.getParam(buf, tmpf64)) {
            ROS_WARN("No parameter liquids:l%d:transport_speed found in configuration file.", num_liquids);
        }
        pmlcreq.transport_speed = tmpf64;

        if(!pipetterMakeLiquidClassClient.call(pmlcreq, pmlcresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to create liquid class table entry for liquid class " << num_liquids << ".");
        }
        num_liquids++;
        memset(buf, ' ', 64);
        sprintf(buf,"/liquids/l%d/name", num_liquids);

    }

    ROS_INFO_STREAM("CDXBotNode: " << "Loaded configuration parameters.");
    return 0;
}

void guiCmdReceived(const std_msgs::String::ConstPtr &s) {
    ROS_INFO_STREAM("CDXBotNode: Received cmd from gui " << s->data);
    if (s->data == "RUN") {
        cd.setRunStatus(1);
    } else if (s->data == "STOP") {
        cd.setRunStatus(0);
    } else if(s->data == "RESET") {
        cd.setRunStatus(0);
        cd.setActionIndex(0);
        cd.setRunStatus(1);
    }
}

void qtCmdReceived(const qt_startcommand::startcommand::ConstPtr&msg, ros::NodeHandle &nh) {
    unsigned int err = 0;
    /* LOAD FILES IF GANTRY IS NOT ALREADY RUNNING */
    if(!run_flag) {
        run_flag = true;

        /* LOAD RUNFILE FROM SPECIFIED LOCATION */
        ROS_INFO_STREAM("Loading HLMD File...");
        if(!cd.parseHLMDFile(msg->CommandFile.c_str())) {
            ROS_INFO_STREAM("CDXBotNode: " << "Found HLMD file at " << msg->CommandFile <<".");
            ROS_DEBUG_STREAM(ros::this_node::getName() << cd.actionMap.size() << "items pushed into actionmap.");
            for(size_t i = 0; i < cd.actionMap.size(); i++) {
                ROS_DEBUG_STREAM(cd.actionMap[i].cmd);
                for(size_t j = 0; j < cd.actionMap[i].args.size(); j++) {
                    ROS_DEBUG_STREAM("\t" << cd.actionMap[i].args[j]);
                }
            }
            ROS_INFO_STREAM("CDXBotNode: " << "Parsed HLMD file successfully.");

        } else {
            ROS_ERROR_STREAM("CDXBotNode: " << "Error reading HLMD file.");
            err = 1;
        }

        /* LAUNCH ROSPARAM TO LOAD LIQUID CLASSES */
        std::string load_str = std::string("rosparam load -v ") + std::string(msg->LiquidClassFile.c_str());
        int ret = system(load_str.c_str());
        if(!ret) {
            loadConfig(nh, cd);
            ROS_INFO_STREAM("CDXBotNode: Loaded liquid class definitions from file at " << msg->LiquidClassFile << " successfully.");
        } else {
            ROS_ERROR_STREAM("CDXBotNode: Error reading liquid class definitions from file at " << msg->LiquidClassFile);
            err = 1;
        }
        if(!err) {
            cd.setRunStatus(1);
        }{

        }
    }
}

void shutdownCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_WARN_STREAM("CDXBotNode: Received shutdown directive.");
    cd.setRunStatus(0);
    struct action a;
    homeCallback(cd, a);
    ros::shutdown();
}

void gantryStatusCallback(const std_msgs::Bool &msg) {
    if(msg.data > 0) {
        ROS_INFO_STREAM("CDXBotNode: Gantry status updated to 1");
    } else {
        ROS_INFO_STREAM("CDXBotNode: Gantry status updated to 0");
    }
    cd.setGantryStatus(msg.data);
}

void pipetterUpdateZPosCallback(const std_msgs::Float64 &msg) {
    cd.setEEPos(2, msg.data);
}

void aspirateCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("Entered aspirateCallback().");
    if(cd.getRunStatus() != 0) {
        cdxbot::gantryMove::Request gmreq;
        cdxbot::gantryMove::Response gmresp;
        Container c = cd.getContainer(a.args[0]);
        unsigned int row = static_cast<unsigned int>(a.args[1]);
        unsigned int col = static_cast<unsigned int>(a.args[2]);
        double vol = static_cast<double>(a.args[3]);
        unsigned int lc_idx = static_cast<unsigned int>(a.args[4]);
        double curr_vol = cd.getContainer(a.args[0]).getCell(row, col).vol_filled;
        double max_vol  = cd.getContainer(a.args[0]).getCell(row, col).vol_max;

        /* Check to see if current well volume < commanded fill volume */
        // if((curr_vol + a.args[3]) > max_vol) {
        // ROS_ERROR_STREAM("CDXBotNode: Requested aspiration volume exceeds pipette tip capacity.");
        // }
        /* Move z-axis to traverse height for container*/
        // if(cd.getPipetterHasZ()) {
        // cdxbot::pipetterMoveZ::Request pmzreq;
        // cdxbot::pipetterMoveZ::Response pmzresp;
        // pmzreq.pos = c.getTraverseHeight();
        // pmzreq.vel = 1;
        // if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
        // cd.setRunStatus(0);
        // }
        // } else {
        // gmreq.z = c.getTraverseHeight();
        // gmreq.movex = false;
        // gmreq.movey = false;
        // gmreq.movez = true;
        // if(!gantryMoveClient.call(gmreq, gmresp)) {
        // ROS_ERROR_STREAM("CDXBotNode: Unable to move gantry to (z = " << gmreq.z << ").");
        // }
        // }

        /* Move gantry to the well location */
        gmreq.move_mode = 0;
        gmreq.x = c.getGlobalCoords('x', row, col);
        gmreq.y = c.getGlobalCoords('y', row, col);
        gmreq.movex = true;
        gmreq.movey = true;
        gmreq.movez = false;
        if(!gantryMoveClient.call(gmreq, gmresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to move gantry to (x = " << gmreq.x << ", y = " << gmreq.y << ").");
        }
        // Turn off liquid level detection
        cdxbot::pipetterSetLLDActive::Request pslreq;
        cdxbot::pipetterSetLLDActive::Response pslresp;
        pslreq.state = false;
        pipetterSetLLDActiveClient.call(pslreq, pslresp);



        /* Aspirate fluid from well */
        cdxbot::pipetterAspirate::Request pareq;
        cdxbot::pipetterAspirate::Response paresp;
        pareq.vol = vol;
        pareq.gc_idx = c.getContainerGeometryTableIndex();
        pareq.dg_idx = c.getDeckGeometryTableIndex();
        pareq.lc_idx = lc_idx;
        pareq.container_height = c.getLength('z');
        // pareq.check_height = c.getCheckHeight();
        pareq.check_height = 5;
        pareq.liquid_surface = c.getLength('z') - 0.95*(c.getWellLength('z'));
        if(!pipetterAspirateClient.call(pareq, paresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to aspirate");
            cd.setRunStatus(0);
        }
        /* Fast move end effector to traverse height */
        cdxbot::pipetterMoveZ::Request pmzreq;
        cdxbot::pipetterMoveZ::Response pmzresp;
        /* Fast move end effector to traverseHeight */
        pmzreq.pos = c.getTraverseHeight();
        pmzreq.vel = 1.0;
        if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
            // Unable to move gantry. HALT!
            ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
            cd.setRunStatus(0);
        }
    }
    ROS_DEBUG_STREAM("Leaving aspirateCallback().");
}

void dispenseCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("Entered dispenseCallback().");
    if(cd.getRunStatus() != 0) {
        cdxbot::gantryMove::Request gmreq;
        cdxbot::gantryMove::Response gmresp;
        /* Move pipette tip to bottom of well */
        Container c = cd.getContainer(a.args[0]);
        unsigned int row = static_cast<unsigned int>(a.args[1]);
        unsigned int col = static_cast<unsigned int>(a.args[2]);
        double vol = static_cast<double>(a.args[3]);
        unsigned int lc_idx = static_cast<unsigned int>(a.args[4]);
        double curr_vol = cd.getContainer(a.args[0]).getCell(row, col).vol_filled;
        double max_vol  = cd.getContainer(a.args[0]).getCell(row, col).vol_max;

        /* Check to see if current well volume + commanded dispensing
         * volume is greater than maximum well volume */
        if((curr_vol + a.args[3]) > max_vol) {
            ROS_ERROR_STREAM("Dispensing requested amount into cell (" << row \
                             << ", " << col << ") in container " <<  a.args[0]\
                             << " would cause liquid to overflow.");
            cd.setRunStatus(0);
        }

        /* Move gantry to cell location */
        gmreq.move_mode = 0;
        gmreq.x = c.getGlobalCoords('x', row, col);
        gmreq.y = c.getGlobalCoords('y', row, col);
        gmreq.z = cd.getFeedPlaneHeight();
        gmreq.movex = true;
        gmreq.movey = true;
        gmreq.movez = false;
        if(!gantryMoveClient.call(gmreq, gmresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to move gantry to (z = " << gmreq.z << ").");
        }

        /* Dispense commanded volume */
        cdxbot::pipetterDispense::Request pdreq;
        cdxbot::pipetterDispense::Response pdresp;
        pdreq.vol = vol;
        pdreq.gc_idx = c.getContainerGeometryTableIndex();
        pdreq.dg_idx = c.getDeckGeometryTableIndex();
        pdreq.lc_idx = lc_idx;
        pdreq.container_height = c.getLength('z');
        // pdreq.liquid_surface = c.getCell(row, col).liquid_height;
        pdreq.liquid_surface = c.getLength('z') - 0.95*c.getWellLength('z');
        if(!pipetterDispenseClient.call(pdreq, pdresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to dispense");
            cd.setRunStatus(0);
        }

        /* Fast move end effector to traverse height */
        cdxbot::pipetterMoveZ::Request pmzreq;
        cdxbot::pipetterMoveZ::Response pmzresp;
        /* Fast move end effector to traverseHeight */
        pmzreq.pos = c.getTraverseHeight();
        pmzreq.vel = 1.0;
        if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
            // Unable to move gantry. HALT!
            ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
            cd.setRunStatus(0);
        }
    }

    ROS_DEBUG_STREAM("CDXBotNode: Leaving dispenseCallback().");
}

void ejectCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered ejectCallback().");
    /* Move tip to feed plane */
    cdxbot::pipetterMoveZ::Request pmzreq;
    cdxbot::pipetterMoveZ::Response pmzresp;
    pmzreq.pos = cd.getFeedPlaneHeight();
    pmzreq.vel = 1.0; /* Fast move using Zeus pipetter.*/
    ROS_DEBUG_STREAM("CDXBotNode::ejectCallback:: Moving pipetter to: " << pmzreq.pos);
    // if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
    // ROS_ERROR_STREAM("Unable to move z-axis to feed plane during eject command. HALT!");
    // cd.setRunStatus(0);
    // }

    /* Rapid feed to center of eject bin */
    cdxbot::gantryMove::Request gmzreq;
    cdxbot::gantryMove::Response gmzresp;
    gmzreq.move_mode = 0;
    gmzreq.x = cd.getEjectPos(AXIS_X);
    gmzreq.y = cd.getEjectPos(AXIS_Y);
    gmzreq.movex = true;
    gmzreq.movey = true;
    gmzreq.movez = false;
    ROS_WARN_STREAM("Moving gantry to eject position (" << gmzreq.x << ", " << gmzreq.y << ")");
    if(!gantryMoveClient.call(gmzreq, gmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move gantry to specified location. HALT!");
        cd.setRunStatus(0);
    }

    /* Eject tip from pipetter */
    cdxbot::pipetterEjectTip::Request petreq;
    cdxbot::pipetterEjectTip::Response petresp;
    if(!pipetterEjectTipClient.call(petreq, petresp)) {
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not eject tip. HALT!");
        cd.setRunStatus(0);
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving ejectCallback().");
}

void homeCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered homeCallback().");
    cdxbot::gantryHome::Request ghomereq;
    cdxbot::gantryHome::Response ghomeresp;
    cdxbot::pipetterHome::Request phomereq;
    cdxbot::pipetterHome::Response phomeresp;
    // ghomereq.x = ghomereq.y = ghomereq.z = 1;
    ghomereq.x = false;
    ghomereq.y = false;
    ghomereq.z = false;
    ghomereq.all = true;
    phomereq.init_z = true;
    phomereq.init_dosing = true;
    /* Pipetter must be homed before gantry is moved. */
    if(!pipetterHomeClient.call(phomereq, phomeresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Error homing pipetter.");
        cd.setRunStatus(0);
    }
    if(!gantryHomeClient.call(ghomereq, ghomeresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Error homing gantry.");
        cd.setRunStatus(0);
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving homeCallback().");
}


void mixCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("Entered mixCallback().");
    unsigned int container = static_cast<unsigned int>(a.args[0]);
    Container c = cd.getContainer(a.args[0]);
    unsigned int row = static_cast<unsigned int>(a.args[1]);
    unsigned int col = static_cast<unsigned int>(a.args[2]);
    unsigned int cycles = static_cast<unsigned int>(a.args[3]);
    double volume = static_cast<double>(a.args[4]);
    // double depth = static_cast<double>(a.args[5]);
    unsigned int asp_lc_index = static_cast<unsigned int>(a.args[5]);
    unsigned int disp_lc_index = static_cast<unsigned int>(a.args[6]);
    unsigned int cg_index = c.getContainerGeometryTableIndex();
    unsigned int dg_index = c.getDeckGeometryTableIndex();
    cdxbot::pipetterSetLLDActive::Request pslreq;
    cdxbot::pipetterSetLLDActive::Response pslresp;
    cdxbot::gantryMove::Request gmreq;
    cdxbot::gantryMove::Response gmresp;
    cdxbot::pipetterGetContainerVolume::Request pgcvreq;
    cdxbot::pipetterGetContainerVolume::Response pgcvresp;
    cdxbot::pipetterAspirate::Request pareq;
    cdxbot::pipetterAspirate::Response paresp;
    cdxbot::pipetterDispense::Request pdreq;
    cdxbot::pipetterDispense::Response pdresp;
    cdxbot::pipetterMoveZ::Request pmzreq;
    cdxbot::pipetterMoveZ::Response pmzresp;
    /* Volume of solution in well should not exceed 75% of the capacity
     * of the pipetter. Check for this. */

    /* MOVE GANTRY TO DESTINATION */
    gmreq.move_mode = 0;
    gmreq.x = c.getGlobalCoords('x', row, col);
    gmreq.y = c.getGlobalCoords('y', row, col);
    gmreq.z = cd.getFeedPlaneHeight();
    gmreq.movex = true;
    gmreq.movey = true;
    gmreq.movez = false;
    if(!gantryMoveClient.call(gmreq, gmresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Unable to move gantry to (z = " << gmreq.z << ").");
    }

    /* MEASURE CONTAINER VOLUME AND LIQUID LEVEL */
    /* Turn on liquid level detection */
    // pslreq.state = true;
    // pipetterSetLLDActiveClient.call(pslreq, pslresp);
    /* Measure container volume and liquid level*/
    // pgcvreq.cg_index = cg_index;
    // pgcvreq.dg_index = dg_index;
    // pgcvreq.lc_index = lc_index;
    // pgcvreq.lld_search_pos = c.getOffset('z');
    // pgcvreq.liquid_surface = c.getOffset('z') - c.getWellLength('z');
    // if(!pipetterGetContainerVolumeClient.call(pgcvreq, pgcvresp)) {
    /* Failed to measure liquid level. */
    // cd.setRunStatus(0);
    // }
    // ROS_WARN_STREAM("\t Measured Fluid Volume: " << pgcvresp.volume << " uL.");
    // ROS_WARN_STREAM("\t Measured Fluid Height: " << pgcvresp.level << " mm.");

    /* MOVE END EFFECTOR TO TOP OF CONTAINER */
    pmzreq.pos = c.getOffset('z');
    pmzreq.vel = 1.0;
    if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
        cd.setRunStatus(0);
    }

    /* Turn off liquid level detection */
    pslreq.state = false;
    pipetterSetLLDActiveClient.call(pslreq, pslresp);

    /* Move pipette tip to halfway up from the bottom of the well */
    /* Draw up and dispense equal volumes of fluid N times */
    for(unsigned int i = 0; i < cycles; i++) {
        if(cd.getRunStatus() == 0) {
            break;
        }
        /* Aspirate fluid from well */
        pareq.vol = volume;
        pareq.gc_idx = c.getContainerGeometryTableIndex();
        pareq.dg_idx = c.getDeckGeometryTableIndex();
        pareq.lc_idx = asp_lc_index;
        pareq.container_height = c.getOffset('z');
        // pareq.check_height = c.getCheckHeight();
        pareq.check_height = 5;
        pareq.liquid_surface = c.getOffset('z') - 0.95*c.getWellLength('z');
        if(!pipetterAspirateClient.call(pareq, paresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to aspirate");
            cd.setRunStatus(0);
        }
        /* Dispense commanded volume */
        pdreq.vol = volume;
        pdreq.gc_idx = c.getContainerGeometryTableIndex();
        pdreq.dg_idx = c.getDeckGeometryTableIndex();
        pdreq.lc_idx = disp_lc_index;
        pdreq.container_height = c.getOffset('z');
        // pdreq.liquid_surface = c.getCell(row, col).liquid_height;
        pdreq.liquid_surface = c.getOffset('z') - 0.95*c.getWellLength('z');
        if(!pipetterDispenseClient.call(pdreq, pdresp)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to dispense");
            cd.setRunStatus(0);
        }
    }

    /* MOVE END EFFECTOR TO TRAVERSE HEIGHT */
    pmzreq.pos = c.getTraverseHeight();
    pmzreq.vel = 1.0;
    if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
        cd.setRunStatus(0);
    }
    /* Dispense drawn amount of fluid + 1 unit to flush pipette tip */
    ROS_DEBUG_STREAM("Leaving mixCallback().");
}

void moveCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered moveCallback().");
    /* 1.) Check location of end effector height.
     * 2.) If end effector is below feed plane, move end effector to feed
     * plane.
     * 3.) Move gantry in XY plane to destination.
     * 4.) Move end effector to Z destination.
     */
    /* Convert CRC (Container, Row, Column) coords to XYZ Coordinates. */
    // if(a.args[1] > cd.getNumContainers()) {
    // THROW ERROR: CONTAINER INDEX OUT OF RANGE.
    // ROS_ERROR("Container index (%d) out of range [0, %d].\n", a.args[1], cd.getNumContainers());
    // return;
    // }
    unsigned int cidx = (int)a.args[0];
    ROS_DEBUG_STREAM("Calculating coordinates of container " << cidx << ", row " << (int)a.args[1] << ", column " << (int)a.args[2]);
    ROS_DEBUG_STREAM("\t Using x-offset " << cd.getContainer(cidx).getOffset('x'));
    ROS_DEBUG_STREAM("\t Using y-offset " << cd.getContainer(cidx).getOffset('y'));
    ROS_DEBUG_STREAM("\t Using z-offset " << cd.getContainer(cidx).getOffset('z'));
    double x = cd.getContainer(cidx).getGlobalCoords('x', (unsigned int)a.args[1], (unsigned int)a.args[2]);
    double y = cd.getContainer(cidx).getGlobalCoords('y', (unsigned int)a.args[1], (unsigned int)a.args[2]);
    double z = cd.getContainer(cidx).getGlobalCoords('z', (unsigned int)a.args[1], (unsigned int)a.args[2]);
    ROS_DEBUG_STREAM("moving to coordinates (x, y, z) = (" << x << ", " << y << ", " << z << ")");
    /* Move tip to feed plane */
    // gmsg.cmd = "movez";
    // gmsg.z = z;
    // gc_pub.publish(gmsg);
    /* Check location of end effector height and move end effector to feed
     * plane if z-position of end effector is below feed plane. */
    if(cd.getEEPos(2) < cd.getFeedPlaneHeight()) {
        if(cd.getPipetterHasZ()) {
            cdxbot::pipetterMoveZ::Request pmzreq;
            cdxbot::pipetterMoveZ::Response pmzresp;
            // pmzreq.pos = cd.getFeedPlaneHeight();
            pmzreq.pos = cd.getContainer(cidx).getLength('z') + 10; /* Move end effector to 10mm above destination container */
            pmzreq.vel = 0;
            if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
                ROS_ERROR_STREAM("CDXBotNode: Unable to move pipetter to (z = )" << z << ").");
            }
        } else {
            cdxbot::gantryMove::Request gmzreq;
            cdxbot::gantryMove::Response gmzresp;
            /* TODO: nam - Get current move mode dynamically. Mon 08 May 2017 10:25:48 AM MDT */
            gmzreq.move_mode = 0;
            gmzreq.movex = false;
            gmzreq.movey = false;
            gmzreq.movez = true;
            gmzreq.z = cd.getFeedPlaneHeight();
            if(!gantryMoveClient.call(gmzreq, gmzresp)) {
                ROS_ERROR_STREAM("CDXBotNode: Unable to move gantry to (z = " << z << ").");
            }
        }
    }
    cdxbot::gantryMove::Request gmxyreq;
    cdxbot::gantryMove::Response gmxyresp;
    /* Move gantry to destination in XY plane. */
    gmxyreq.move_mode = 0;
    gmxyreq.movex = true;
    gmxyreq.movey = true;
    gmxyreq.movez = false;
    gmxyreq.x = x;
    gmxyreq.y = y;
    if(!gantryMoveClient.call(gmxyreq, gmxyresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Unable to move gantry to (x=" << x << ", y=" << y << ", z = " << z);
    }
    /* TODO: nam - WTF are we doing here? Why issue another z-move to the
     * feed plane?  Tue 29 Aug 2017 02:57:14 PM MDT */

    if(cd.getPipetterHasZ()) {
        cdxbot::pipetterMoveZ::Request pmzreq2;
        cdxbot::pipetterMoveZ::Response pmzresp2;
        pmzreq2.pos = cd.getFeedPlaneHeight();
        pmzreq2.vel = 0;
        if(!pipetterMoveZClient.call(pmzreq2, pmzresp2)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to move pipetter to (z = )" << z << ").");
        }
    } else {
        cdxbot::gantryMove::Request gmzreq2;
        cdxbot::gantryMove::Response gmzresp2;
        /* TODO: nam - Get current move mode dynamically. Mon 08 May 2017 10:25:48 AM MDT */
        gmzreq2.move_mode = 0;
        gmzreq2.x = -1;
        gmzreq2.y = -1;
        gmzreq2.z = cd.getFeedPlaneHeight();
        if(!gantryMoveClient.call(gmzreq2, gmzresp2)) {
            ROS_ERROR_STREAM("CDXBotNode: Unable to move gantry to (z = " << z << ").");
        }
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving moveCallback().");
}

void pauseCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered pauseCallback().");
    /* Pause execution of commands */
    cd.setRunStatus(0);
    /* Publish status message to GUI indicating that execution has halted */
    /* Execution of runfile will halt until GUI publishes resume command */
    ROS_DEBUG_STREAM("CDXBotNode: Leaving pauseCallback().");
}

void pickupCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered pickupCallback().");
    double x = 0, y = 0, z = 0;
    Container c = cd.getContainer(a.args[0]);
    if(c.getType() != "tip") {
        /* ERROR: CANNOT PERFORM PICKUP OPERATION ON WELL-TYPE CONTAINER */
        ROS_ERROR_STREAM("Invalid container [" << a.args[0] << "] selected for pickup operation.");
        return;
    }
    if((c.getCell(a.args[1],a.args[2]).used == true)) {
        /* ERROR: NO TIP PRESENT AT THIS LOCATION */
        ROS_ERROR_STREAM("No consumable item available at specified location [." << a.args[0] << ", " << a.args[1] << ", " << a.args[2]);
        return;
    }
    /* Move to location */
    ROS_DEBUG_STREAM("CDXBotNode: Moving to location.");
    cdxbot::gantryMove::Request gmzreq;
    cdxbot::gantryMove::Response gmzresp;
    gmzreq.move_mode = 0;
    gmzreq.x = cd.getContainer(a.args[0]).getGlobalCoords('x', a.args[1], a.args[2]);
    gmzreq.y = cd.getContainer(a.args[0]).getGlobalCoords('y', a.args[1], a.args[2]);
    gmzreq.z = cd.getContainer(a.args[0]).getGlobalCoords('z', a.args[1], a.args[2]);
    gmzreq.movex = true;
    gmzreq.movey = true;
    gmzreq.movez = true;
    if(!gantryMoveClient.call(gmzreq, gmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move gantry to specified location. HALT!");
        cd.setRunStatus(0);
    }
    ROS_DEBUG_STREAM("CDXBotNode: Picking up tip.");
    /* Pick up Tip */
    cdxbot::pipetterPickUpTip::Request pputreq;
    cdxbot::pipetterPickUpTip::Response pputresp;
    pputreq.tip_type_table_index = c.getTipType();
    pputreq.deck_geometry_table_index = c.getDeckGeometryTableIndex();
    pputreq.tip_pickup_speed = 0; /* Slow tip pick-up */
    if(!pipetterPickUpTipClient.call(pputreq, pputresp)) {
        // Unable to pickup tip from specified location. HALT!
        ROS_ERROR_STREAM("Unable to pick up tip from specified location. HALT!");
        cd.setRunStatus(0);
    }

    cdxbot::pipetterMoveZ::Request pmzreq;
    cdxbot::pipetterMoveZ::Response pmzresp;
    /* Fast move end effector to traverseHeight */
    pmzreq.pos = cd.getContainer(a.args[0]).getTraverseHeight();
    pmzreq.vel = 1.0;
    if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
        cd.setRunStatus(0);
    }

    ROS_DEBUG_STREAM("CDXBotNode: Leaving PickupCallback().");
}

void pierceCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered pierceCallback().");
    /* Check to make sure pipette tip is present on pipetter */
    /* Move end effector to location */
    cdxbot::gantryMove::Request gmreq;
    cdxbot::gantryMove::Response gmresp;
    double cont = a.args[0];
    double row = a.args[1];
    double col = a.args[2];
    double depth = cd.getContainer(cont).getGlobalCoords('z', row, col) - a.args[3];
    gmreq.move_mode = 0;
    gmreq.x = cd.getContainer(cont).getGlobalCoords('x', row, col);
    gmreq.y = cd.getContainer(cont).getGlobalCoords('y', row, col);
    gmreq.z = cd.getContainer(cont).getGlobalCoords('z', row, col);
    gmreq.movex = true;
    gmreq.movey = true;
    gmreq.movez = false;
    if(!gantryMoveClient.call(gmreq, gmresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move gantry to specified location. HALT!");
        cd.setRunStatus(0);
    }

    cdxbot::pipetterMoveZ::Request pmzreq;
    cdxbot::pipetterMoveZ::Response pmzresp;
    /* Fast move end effector to top of container */
    pmzreq.pos = cd.getContainer(cont).getGlobalCoords('z', row, col);
    pmzreq.vel = 1.0;
    if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
        cd.setRunStatus(0);
    }
    /* Cap penetration depth so that end effector does not crash into container
     * floor. */
    double max_depth = cd.getContainer(cont).getGlobalCoords('z', row, col) - cd.getContainer(cont).getWellLength('z');
    pmzreq.pos = (depth < max_depth) ? max_depth : depth;
    /* Slow move end effector to desired penetration depth.*/
    pmzreq.vel = 0.0;
    if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
        cd.setRunStatus(0);
    }
    /* Fast move end effector to traverse height for container */
    pmzreq.pos = cd.getContainer(cont).getTraverseHeight();
    pmzreq.vel = 1.0;
    if(!pipetterMoveZClient.call(pmzreq, pmzresp)) {
        // Unable to move gantry. HALT!
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not move pipetter z-axis. HALT!");
        cd.setRunStatus(0);
    }



    ROS_DEBUG_STREAM("CDXBotNode: Leaving pierceCallback().");
}

void shakerResetCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered shakerResetCallback().");
    cdxbot::shakerReset::Request shaker_reset_request;
    cdxbot::shakerReset::Response shaker_reset_response;
    shaker_reset_request.reset = 1;
    if(!shakerResetClient.call(shaker_reset_request, shaker_reset_response)) {
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not reset shaker.");
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving shakerResetCallback().");
}

void shakerSetFreqCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered shakerSetFreqCallback().");
    cdxbot::shakerSetFreq::Request shaker_setfreq_request;
    cdxbot::shakerSetFreq::Response shaker_setfreq_response;
    shaker_setfreq_request.freq = static_cast<uint32_t>(a.args[0]);
    if(!shakerSetFreqClient.call(shaker_setfreq_request, shaker_setfreq_response)) {
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not set shaker frequency.");
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving shakerSetFreqCallback().");
}

void shakerSetPowerCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered shakerSetPowerCallback().");
    cdxbot::shakerSetPower::Request shaker_setpower_request;
    cdxbot::shakerSetPower::Response shaker_setpower_response;
    shaker_setpower_request.pwr = static_cast<uint8_t>(a.args[0]);
    if(!shakerSetPowerClient.call(shaker_setpower_request, shaker_setpower_response)) {
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not set shaker power.");
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving shakerSetPowerCallback().");
}

void shakerStartCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered shakerStartCallback().");
    cdxbot::shakerStart::Request shaker_start_request;
    cdxbot::shakerStart::Response shaker_start_response;
    shaker_start_request.start = 1;
    shaker_start_request.time = ((a.args.size() > 0) ? static_cast<float>(a.args[0]) : 0);
    if(!shakerStartClient.call(shaker_start_request, shaker_start_response)) {
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not activate shaker.");
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving shakerStartCallback().");
}

void shakerStopCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered shakerStopCallback().");
    cdxbot::shakerStop::Request shaker_stop_request;
    cdxbot::shakerStop::Response shaker_stop_response;
    shaker_stop_request.stop = 1;
    if(!shakerStopClient.call(shaker_stop_request, shaker_stop_response)) {
        ROS_ERROR_STREAM(__FILE__ << ": " << __PRETTY_FUNCTION__ << ": " << "Could not deactivate shaker.");
    }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving shakerStopCallback().");
}

void waitCallback(CDXBot &cd, const struct action a) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered waitCallback().");
    float dur = a.args[0] / 1000.0;
    ROS_INFO_STREAM("sleeping for " << dur << " seconds.");
    ros::Duration(dur).sleep();
    // usleep((a.args[0] * 1000));
    // gmsg.cmd = "wait";
    // gmsg.time = a.args[0];
    // gc_pub.publish(gmsg);
    ROS_DEBUG_STREAM("CDXBotNode: Leaving waitCallback().");
}

void emergencyStopCallback(CDXBot &cd, const struct action a) {
    static bool pipetter_estop_active = false;
    static bool gantry_estop_active = false;
    ROS_DEBUG_STREAM("CDXBotNode: Entered emergencyStopCallback().");
    cdxbot::pipetterEmergencyStop::Request pesreq;
    cdxbot::pipetterEmergencyStop::Response pesresp;
    cdxbot::gantryEmergencyStop::Request gesreq;
    cdxbot::gantryEmergencyStop::Response gesresp;

    pipetter_estop_active ^= pipetter_estop_active;
    gantry_estop_active ^= gantry_estop_active;

    pesreq.state = pipetter_estop_active;
    gesreq.state = gantry_estop_active;

    if(!pipetterEmergencyStopClient.call(pesreq, pesresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Could not activate pipetter emergency stop.");
    }
    if(!gantryEmergencyStopClient.call(gesreq, gesresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Could not activate gantry emergency stop.");
    }

    if((pipetter_estop_active == true) || (gantry_estop_active == true)) {
        cd.setRunStatus(0);
    }

    ROS_DEBUG_STREAM("CDXBotNode: Leaving emergencyStopCallback().");
}

void disposeTip(void) {
    ROS_DEBUG_STREAM("CDXBotNode: Entered disposeTip()");
    struct action b;
    cdxbot::gantryHome::Request ghomereq;
    cdxbot::gantryHome::Response ghomeresp;
    cdxbot::pipetterHome::Request phomereq;
    cdxbot::pipetterHome::Response phomeresp;
    cdxbot::gantryMove::Request gmreq;
    cdxbot::gantryMove::Response gmresp;
    // ghomereq.x = ghomereq.y = ghomereq.z = 1;
    ghomereq.all = true;
    ghomereq.x = false;
    ghomereq.y = false;
    ghomereq.z = false;
    phomereq.init_z = true;
    phomereq.init_dosing = false;
    gmreq.move_mode = 0;
    gmreq.x = 10;
    gmreq.y = 400;
    gmreq.movex = true;
    gmreq.movey = true;
    gmreq.movez = false;
    /* Pipetter must be homed before gantry is moved. */
    if(!pipetterHomeClient.call(phomereq, phomeresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Error homing pipetter.");
        cd.setRunStatus(0);
    }
    if(!gantryHomeClient.call(ghomereq, ghomeresp)) {
        ROS_ERROR_STREAM("CDXBotNode: Error homing gantry.");
        cd.setRunStatus(0);
    }
    ejectCallback(cd, b);
    // if(!gantryMoveClient.call(gmreq, gmresp)) {
    // ROS_ERROR_STREAM("CDXBotNode: Error moving gantry.");
    // cd.setRunStatus(0);
    // }
    ROS_DEBUG_STREAM("CDXBotNode: Leaving disposeTip()");
}

bool initManagedNodes(void) {
    bool ret = true;
    cdxbot::nodeInit::Request req;
    cdxbot::nodeInit::Response resp;
    if(!gantryInitClient.call(req, resp)) {
        ret = false;
    }
    if(!pipetterInitClient.call(req, resp)) {
        ret = false;
    }
    if(!shakerInitClient.call(req, resp)) {
        ret = false;
    }

    return ret;
}

void shutdownManagedNodes(void) {
    cdxbot::nodeShutdown::Request req;
    cdxbot::nodeShutdown::Response resp;

    gantryShutdownClient.call(req, resp);
    pipetterShutdownClient.call(req, resp);
    shakerShutdownClient.call(req, resp);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cdxbot_node");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    /* Subscribe to the shutdown topic */
    ros::Subscriber sd = nh.subscribe("/sd_pub", 100, &shutdownCallback);
    /* Subscribe to GUI command topic */
    // guiSub = nh.subscribe("/gui_cmd", 1000, &guiCmdReceived);
    /* Subscribe to topic published by QT interface */
    ros::Subscriber qtSub = nh.subscribe<qt_startcommand::startcommand>("/gui_cmd", 1000, boost::bind(&qtCmdReceived, _1, boost::ref(nh)));
    ros::Subscriber qtEStopSub = nh.subscribe<std_msgs::String>("/estop", 1000, boost::bind(&emergencyStopCallback, _1, boost::ref(nh)));
    /* Subscribe to the status topics published by other nodes */
    ros::Subscriber gc_status = nh.subscribe("/gantry_status", 1000, &gantryStatusCallback);
    ros::Subscriber pipetter_zpos = nh.subscribe("/pipetter_zpos", 1000, &pipetterUpdateZPosCallback);
    /* Create publishers for the various controller */
    // gc_pub = nh.advertise<cdxbot::gc_cmd>("gc_pub", 100);
    // pc_pub = nh.advertise<cdxbot::pc_cmd>("pc_pub", 100);
    // vc_pub = nh.advertise<cdxbot::vc_cmd>("vc_pub", 100);
    // sc_pub = nh.advertise<cdxbot::sc_cmd>("sc_pub", 100);
    /* Create service clients */
    // gcClient = nh.serviceClient<cdxbot::gc_cmd_s>("gc_cmd_s");
    // pcClient = nh.serviceClient<cdxbot::pc_cmd_s>("pc_cmd_s");
    // vcClient = nh.serviceClient<cdxbot::vc_cmd_s>("vc_cmd_s");
    gantryEmergencyStopClient = nh.serviceClient<cdxbot::gantryEmergencyStop>("gantry_emergency_stop");
    gantryGetCurrentPositionClient = nh.serviceClient<cdxbot::gantryGetCurrentPosition>("gantry_get_current_position");
    gantryHomeClient = nh.serviceClient<cdxbot::gantryHome>("gantry_home");
    gantryInitClient = nh.serviceClient<cdxbot::nodeInit>("gantry_init");
    gantryMotorsToggleClient = nh.serviceClient<cdxbot::gantryMotorsToggle>("gantry_motors_toggle");
    gantryMoveClient = nh.serviceClient<cdxbot::gantryMove>("gantry_move");
    gantrySetAccelerationsClient = nh.serviceClient<cdxbot::gantrySetAccelerations>("gantry_set_accelerations");
    gantrySetAxisStepsPerUnitClient = nh.serviceClient<cdxbot::gantrySetAxisStepsPerUnit>("gantry_set_axis_steps_per_unit");
    gantrySetFeedratesClient = nh.serviceClient<cdxbot::gantrySetFeedrates>("gantry_set_feedrates");
    gantryShutdownClient = nh.serviceClient<cdxbot::nodeShutdown>("gantry_shutdown");
    gantrySetUnitsClient = nh.serviceClient<cdxbot::gantrySetUnits>("gantry_set_units");
    pipetterAspirateClient = nh.serviceClient<cdxbot::pipetterAspirate>("pipetter_aspirate");
    pipetterDispenseClient = nh.serviceClient<cdxbot::pipetterDispense>("pipetter_dispense");
    pipetterEjectTipClient = nh.serviceClient<cdxbot::pipetterEjectTip>("pipetter_eject_tip");
    pipetterEmergencyStopClient = nh.serviceClient<cdxbot::pipetterEmergencyStop>("pipetter_emergency_stop");
    pipetterGetContainerVolumeClient = nh.serviceClient<cdxbot::pipetterGetContainerVolume>("pipetter_get_container_volume");
    pipetterHomeClient = nh.serviceClient<cdxbot::pipetterHome>("pipetter_home");
    pipetterInitClient = nh.serviceClient<cdxbot::nodeInit>("pipetter_init");
    pipetterShutdownClient = nh.serviceClient<cdxbot::nodeShutdown>("pipetter_shutdown");
    pipetterMakeContainerGeometryClient = nh.serviceClient<cdxbot::pipetterMakeContainerGeometry>("pipetter_make_container_geometry");
    pipetterMakeDeckGeometryClient = nh.serviceClient<cdxbot::pipetterMakeDeckGeometry>("pipetter_make_deck_geometry");
    pipetterMakeLiquidClassClient = nh.serviceClient<cdxbot::pipetterMakeLiquidClass>("pipetter_make_liquid_class");
    pipetterMoveZClient = nh.serviceClient<cdxbot::pipetterMoveZ>("pipetter_move_z");;
    pipetterSetLLDActiveClient = nh.serviceClient<cdxbot::pipetterSetLLDActive>("pipetter_set_lld_active");
    pipetterPickUpTipClient = nh.serviceClient<cdxbot::pipetterPickUpTip>("pipetter_pick_up_tip");
    shakerInitClient = nh.serviceClient<cdxbot::nodeInit>("shaker_init");
    shakerShutdownClient = nh.serviceClient<cdxbot::nodeShutdown>("shaker_shutdown");
    shakerResetClient = nh.serviceClient<cdxbot::shakerReset>("shaker_reset");
    shakerSetFreqClient = nh.serviceClient<cdxbot::shakerSetFreq>("shaker_set_freq");
    shakerSetPowerClient = nh.serviceClient<cdxbot::shakerSetPower>("shaker_set_power");
    shakerStartClient = nh.serviceClient<cdxbot::shakerStart>("shaker_start");
    shakerStopClient = nh.serviceClient<cdxbot::shakerStop>("shaker_stop");

    /* Check for and load/parse HLMD file */
    ros::Duration(3).sleep(); // Wait for services to inittialize
    // loadConfig(nh, cd);

    /* TODO: nam - Load HLMDFileLocation from parameter given in CDXBot.conf Fri 31 Mar 2017 10:08:33 AM MDT */

    // if(!cd.parseHLMDFile(cd.HLMDFileLocation)) {
    // ROS_INFO_STREAM("CDXBotNode: " << "Found HLMD file at " << cd.HLMDFileLocation <<".");
    // ROS_DEBUG_STREAM("CDXBotNode: " << cd.actionMap.size() << "items pushed into actionmap.");
    // for(size_t i = 0; i < cd.actionMap.size(); i++) {
    // ROS_DEBUG_STREAM(cd.actionMap[i].cmd);
    // for(size_t j = 0; j < cd.actionMap[i].args.size(); j++) {
    // ROS_DEBUG_STREAM("\t" << cd.actionMap[i].args[j]);
    // }
    // }
    // ROS_INFO_STREAM("CDXBotNode: " << "Parsed HLMD file successfully.");

    // } else {
    // ROS_INFO_STREAM("CDXBotNode: " << "Error reading HLMD file.");
    // }

    /* Dispose of existing tip */
    // disposeTip();
    ROS_INFO_STREAM("CDXBotNode: Initializing managed nodes");
    if(initManagedNodes() == true) {
        ROS_INFO_STREAM("CDXBotNode: Successfully initialized managed nodes");
        /* Process commands in HLMD file. */
        while(ros::ok()) {
            if(cd.getRunStatus() == 1) {
                struct action a;
                if(cd.getNextAction(a) < 0) {
                    ROS_INFO_STREAM("DONE!");
                    cd.setRunStatus(0);
                } else {
                    if(command_table.count(a.cmd)) {
                        command_table[a.cmd](cd, a);
                    } else {
                        ROS_ERROR_STREAM("CDXBotNode: No command \"" << a.cmd << "\" is implemented.");
                    }
                }
            } else {
                ros::spinOnce();
                rate.sleep();
            }
        }
        return 0;
    } else {
        ROS_ERROR_STREAM("CDXBotNode: Failed to initialize managed nodes");
        shutdownManagedNodes();
        ros::shutdown();
        return -1;
    }
}
