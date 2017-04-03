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
#include <ros/ros.h>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <stdbool.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "cdxbot/gc_cmd.h"
#include "cdxbot/gc_cmd_s.h"
#include "cdxbot/pc_cmd.h"
#include "cdxbot/pc_cmd_s.h"
#include "cdxbot/vc_cmd.h"
#include "cdxbot/vc_cmd_s.h"
#include "common.h"
#include "CDXBot.h"


/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
CDXBot cd;

/* Create subscriber for GUI commands */
ros::Subscriber guiSub;

/* Create publishers for the various controller */
ros::Publisher gc_pub; //= nh.advertise<cdx
ros::Publisher pc_pub;
ros::Publisher vc_pub;
ros::Publisher shutdown_pub;
/*******************    FUNCTION IMPLEMENTATIONS    ********************/

int loadConfig(ros::NodeHandle nh, CDXBot &cd) {
    char buf[64];
    memset(buf, ' ',64);
    int ret = 0;
    nh.getParam("/cdxbot/num_containers", ret);
    printf("Read value of %d for num_containers.\n", ret);
    if(!nh.getParam("/cdxbot/num_containers", ret)) {
        nh.getParam("/cdxbot_defaults/num_containers", ret);
        ROS_WARN("No parameter 'num_containers' found in configuration file.\
                        Initializing cdxbot with default value %s",\
                 cd.getNumContainers());
    }
    cd.setNumContainers(ret);
    // if(!nh.getParam("/cdxbot/num_containers", cd.getNumContainersRef())) {
    // nh.getParam("/cdxbot_defaults/num_containers", cd.getNumContainersRef());
    // ROS_WARN("No parameter 'num_containers' found in configuration file.\
    // Initializing cdxbot with default value %s",\
    // cd.getNumContainers());
    // }
    for(unsigned int i=0; i < cd.getNumContainers(); i++) {
        sprintf(buf,"/cdxbot/containers/c%d/type", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTypeRef())) {
            nh.getParam("/cdxbot_defaults/type",cd.getContainer(i).getTypeRef());
            ROS_WARN("No parameter containers:%d:type found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getType());
        }
        printf("Container %d initialized with type %s.\n", i, cd.getContainer(i).getType().c_str());

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/length", i);
        if(!nh.getParam(buf, cd.getContainer(i).getLengthRef())) {
            nh.getParam("/cdxbot_defaults/length", cd.getContainer(i).getLengthRef());
            ROS_WARN("No parameter containers:%d:length found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getLengthRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/width", i);
        if(!nh.getParam(buf, cd.getContainer(i).getWidthRef())) {
            nh.getParam("/cdxbot_defaults/width", cd.getContainer(i).getWidthRef());
            ROS_WARN("No parameter containers:%d:width found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getWidthRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/height", i);
        if(!nh.getParam(buf, cd.getContainer(i).getHeightRef())) {
            nh.getParam("/cdxbot_defaults/height", cd.getContainer(i).getHeightRef());
            ROS_WARN("No parameter containers:%d:height found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getHeightRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/rows", i);
        if(!nh.getParam(buf, cd.getContainer(i).getRowsRef())) {
            nh.getParam("/cdxbot_defaults/rows", cd.getContainer(i).getRowsRef());
            ROS_WARN("No parameter containers:%d:rows found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getRowsRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/cols", i);
        if(!nh.getParam(buf, cd.getContainer(i).getColsRef())) {
            nh.getParam("/cdxbot_defaults/cols", cd.getContainer(i).getColsRef());
            ROS_WARN("No parameter containers:%d:cols found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getColsRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/row_spacing", i);
        if(!nh.getParam(buf, cd.getContainer(i).getRowSpacingRef())) {
            nh.getParam("/cdxbot_defaults/row_spacing", cd.getContainer(i).getRowSpacingRef());
            ROS_WARN("No parameter containers:%d:row_spacing found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getRowSpacingRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/col_spacing", i);
        if(!nh.getParam(buf, cd.getContainer(i).getColSpacingRef())) {
            nh.getParam("/cdxbot_defaults/col_spacing", cd.getContainer(i).getColSpacingRef());
            ROS_WARN("No parameter containers:%d:col_spacing found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getColSpacingRef());
        }


        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/offset_x", i);
        if(!nh.getParam(buf, cd.getContainer(i).getOffsetXRef())) {
            nh.getParam("/cdxbot_defaults/offset_x", cd.getContainer(i).getOffsetXRef());
            ROS_WARN("No parameter containers:%d:offset_x found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getOffsetXRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/offset_y", i);
        if(!nh.getParam(buf, cd.getContainer(i).getOffsetYRef())) {
            nh.getParam("/cdxbot_defaults/offset_y", cd.getContainer(i).getOffsetYRef());
            ROS_WARN("No parameter containers:%d:offset_y found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getOffsetYRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/offset_z", i);
        if(!nh.getParam(buf, cd.getContainer(i).getOffsetZRef())) {
            nh.getParam("/cdxbot_defaults/offset_z", cd.getContainer(i).getOffsetZRef());
            ROS_WARN("No parameter containers:%d:offset_z found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getOffsetZRef());
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/tray_offset_x", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTrayOffsetRef(AXIS_X))) {
            nh.getParam("/cdxbot_defaults/tray_offset_x", cd.getContainer(i).getTrayOffsetRef(AXIS_X));
            ROS_WARN("No parameter containers:%d:tray_offset_x found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getTrayOffset(AXIS_X));
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/tray_offset_y", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTrayOffsetRef(AXIS_Y))) {
            nh.getParam("/cdxbot_defaults/tray_offset_y", cd.getContainer(i).getTrayOffsetRef(AXIS_Y));
            ROS_WARN("No parameter containers:%d:tray_offset_y found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getTrayOffset(AXIS_Y));
        }

        memset(buf, ' ',64);
        sprintf(buf,"/cdxbot/containers/c%d/tray_offset_z", i);
        if(!nh.getParam(buf, cd.getContainer(i).getTrayOffsetRef(AXIS_Z))) {
            nh.getParam("/cdxbot_defaults/tray_offset_z", cd.getContainer(i).getTrayOffsetRef(AXIS_Z));
            ROS_WARN("No parameter containers:%d:tray_offset_z found in configuration file.\
                        Initializing cdxbot with default value %s",\
                     i, cd.getContainer(i).getTrayOffset(AXIS_Z));
        }
        /* INITIALIZE CONTAINER CELL MATRIX */
        for(unsigned int j =0; j < cd.getContainer(i).getRows(); j++) {
            std::vector<struct container_cell> newRow;
            cd.getContainer(i).getCellsVecRef().push_back(newRow);
            for(unsigned int k=0; k < cd.getContainer(i).getCols(); k++) {
                struct container_cell c;
                c.test_type = " ";
                c.used = 0;
                if(cd.getContainer(i).getType() == "well") {
                    memset(buf, ' ',64);
                    sprintf(buf,"/cdxbot/containers/c%d/container_vol", i);
                    if(!nh.getParam(buf, c.vol)) {
                        nh.getParam("/cdxbot_defaults/container_vol", c.vol);
                        ROS_WARN("No parameter containers:%d:container_vol found in configuration file.\
                        Initializing cdxbot with default value %f",\
                                 i, c.vol);
                    }

                    memset(buf, ' ',64);
                    sprintf(buf,"/cdxbot/containers/c%d/well_depth", i);
                    if(!nh.getParam(buf, c.depth)) {
                        nh.getParam("/cdxbot_defaults/well_depth", c.depth);
                        ROS_WARN("No parameter containers:%d:well_depth found in configuration file.\
                        Initializing cdxbot with default value %s",\
                                 i, c.depth);
                    }
                }
                cd.getContainer(i).getCellsVecRef()[j].push_back(c);
            }
        }
    }

    ROS_INFO_STREAM(ros::this_node::getName() << "Loaded configuration parameters.");
    return 0;
}

void guiCmdReceived(const std_msgs::String::ConstPtr &s) {
    ROS_INFO_STREAM("CDXBotNode: Received cmd from gui " << s->data);
    if (s->data == "RUN") {
        cd.setRunStatus(1);
    } else if (s->data == "STOP") {
        cd.setRunStatus(0);
    }
    else if(s->data == "SHUTDOWN") {
        cd.setRunStatus(0);
        std_msgs::String msg;
        std::stringstream ss;
        ss << "SHUT THE HELL DOWN";
        msg.data = ss.str();
        shutdown_pub.publish(msg);
        ros::shutdown();
    }
    else if(s->data == "RESET"){
        cd.setRunStatus(0);
        cd.setActionIndex(0);
        cd.setRunStatus(1);
    } 
}

void parseAction(CDXBot &cd, const struct action a) {
    cdxbot::gc_cmd gmsg;
    cdxbot::pc_cmd pmsg;
    cdxbot::vc_cmd vmsg;

    if(a.cmd == "move") {
        /* Convert CRC (Container, Row, Column) coords to XYZ Coordinates. */
        // if(a.args[1] > cd.getNumContainers()) {
        // THROW ERROR: CONTAINER INDEX OUT OF RANGE.
        // ROS_ERROR("Container index (%d) out of range [0, %d].\n", a.args[1], cd.getNumContainers());
        // return;
        // }
        printf("CDXBotNode: Parsing move command.\n");
        unsigned int cidx = (int)a.args[0];
        printf("Calculating coordinates of container... %d\n", cidx);
        double x = cd.getContainer(cidx).getGlobalCoords('x', a.args[1], a.args[2]);
        double y = cd.getContainer(cidx).getGlobalCoords('y', a.args[1], a.args[2]);
        double z = cd.getContainer(cidx).getGlobalCoords('z', a.args[1], a.args[2]);
        std::cout << "moving to coordinates (x, y, z) = (" << x << ", " << y << ", " << z << ")" << std::endl;
        /* Move tip to feed plane */
        gmsg.cmd = "movez";
        gmsg.z = z;
        gc_pub.publish(gmsg);

        /* Rapid feed to calculated coordinates in feed plane */
        gmsg.cmd = "setvel";
        gmsg.vel = -1;   //  [> Feed at maximum velocity <]
        gc_pub.publish(gmsg);

        gmsg.cmd = "movexy";
        gmsg.x = x;
        gmsg.y = y;
        gc_pub.publish(gmsg);

        /* Set velocity to plunge velocity */
        gmsg.cmd = "setvel";
        gmsg.vel = -1;    // [> Feed at maximum velocity <]
        gc_pub.publish(gmsg);

        /* Move to top of container cell */
        gmsg.cmd = "movez";
        gmsg.z = z;
        gc_pub.publish(gmsg);

    } else if(a.cmd == "fill") {
        /* FILL PIPETTE TIP FROM WELL; TO FILL WELL, USE DISPENSE COMMAND*/
        /* Check to see if current well volume < commanded fill volume */
        unsigned int row = static_cast<unsigned int>(a.args[1]);
        unsigned int col = static_cast<unsigned int>(a.args[2]);
        double curr_vol = cd.getContainer(a.args[0]).getCell(row, col).filled_vol;
        double max_vol  = cd.getContainer(a.args[0]).getCell(row, col).vol;
        if((curr_vol + a.args[3]) > max_vol) {
            //Throw some error
            //return;
        }
        /* Move pipette tip to bottom of well */
        /* Set velocity to plunge velocity */
        gmsg.cmd = "setvel";
        gmsg.vel = -1;     /* Feed at maximum velocity */
        gc_pub.publish(gmsg);

        /* Move to bottom of container cell */
        gmsg.cmd = "movez";
        gmsg.z =cd.getContainer(a.args[0]).getCell(a.args[1],a.args[2]).depth;
        gc_pub.publish(gmsg);

        /* Aspirate */

    } else if(a.cmd == "dispense") {
        /* Check to see if current well volume + commanded dispensing
         * volume is greater than maximum well volume */
        /* Move pipette tip to bottom of well */
        /* Dispense commanded volume */

    } else if(a.cmd == "mix") {
        /* Volume of solution in well should not exceed 75% of the capacity
         * of the pipetter. Check for this. */
        /* Move pipette tip to halfway up from the bottom of the well */
        /* Draw up and dispense equal volumes of fluid N times */
        /* Dispense drawn amount of fluid + 1 unit to flush pipette tip */

    } else if(a.cmd == "eject") {
        /* Move tip to feed plane */

        /* Rapid feed to center of eject bin */

        /* Eject tip from pipetter */

        /* Move to home position? */
    } else if(a.cmd == "wait") {
        gmsg.cmd = "wait";
        gmsg.time = a.args[0];
        gc_pub.publish(gmsg);
    } else if(a.cmd == "home") {
        gmsg.cmd = "home";
        gc_pub.publish(gmsg);
    } else if (a.cmd == "estop") {
        gmsg.cmd = "estop";
        gc_pub.publish(gmsg);
    } else if(a.cmd == "estoprst") {
        gmsg.cmd = "estoprst";
        gc_pub.publish(gmsg);
    }
}

void shutdownCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("CDXBotNode: Received shutdown directive.");
    ros::shutdown();
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "cdxbot_node");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    loadConfig(nh, cd);
    /* Subscribe to GUI topic */
    guiSub = nh.subscribe("/gui_cmd", 1000, &guiCmdReceived);
    ros::Subscriber sd = nh.subscribe("/sd_pub", 1000, &shutdownCallback);

    /* Create publishers for the various controller */
    gc_pub = nh.advertise<cdxbot::gc_cmd>("gc_pub", 100);
    pc_pub = nh.advertise<cdxbot::pc_cmd>("pc_pub", 100);
    vc_pub = nh.advertise<cdxbot::vc_cmd>("vc_pub", 100);
    
    ros::ServiceClient gcClient = nh.serviceClient<cdxbot::gc_cmd_s>("gc_cmd_s");
    ros::ServiceClient pcClient = nh.serviceClient<cdxbot::pc_cmd_s>("pc_cmd_s");
    ros::ServiceClient vcClient = nh.serviceClient<cdxbot::vc_cmd_s>("vc_cmd_s");
    //shutdown_pub = nh.advertise<std_msgs::String>("sd_pub", 100);

    /* Check for and load/parse HLMD file */

    /* TODO: nam - Load HLMDFileLocation from parameter given in CDXBot.conf Fri 31 Mar 2017 10:08:33 AM MDT */

    if(!cd.parseHLMDFile(cd.HLMDFileLocation)) {
        ROS_INFO_STREAM(ros::this_node::getName() << "Found HLMD file at " << cd.HLMDFileLocation <<".");
        ROS_DEBUG_STREAM(ros::this_node::getName() << cd.actionMap.size() << "items pushed into actionmap.");
        for(size_t i = 0; i < cd.actionMap.size(); i++) {
            ROS_DEBUG_STREAM(cd.actionMap[i].cmd);
            for(size_t j = 0; j < cd.actionMap[i].args.size(); j++) {
                ROS_DEBUG_STREAM("\t" << cd.actionMap[i].args[j]);
            }
        }
        ROS_INFO_STREAM(ros::this_node::getName() << "Parsed HLMD file successfully.");

    } else {
        ROS_INFO_STREAM(ros::this_node::getName() << "Error reading HLMD file.");
    }
    /* Process commands in HLMD file. */
    while(ros::ok()) {
        if(cd.getRunStatus() == 1) {
            struct action a;

            if(cd.getNextAction(a) < 0) {
                ROS_INFO_STREAM("DONE!");
                cd.setRunStatus(0);
            } else {
                parseAction(cd, a);
            }
        } else {
            ros::spinOnce();
            rate.sleep();
        }
    }
    return 0;
}
