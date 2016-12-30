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
#include <cstdio>
#include <iostream>
#include <stdbool.h>
#include "std_msgs/String.h"
#include "cdxbot/gc_cmd.h"
#include "CDXBot.h"
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
std::string defaultConfigFilePath = ".";
CDXBot cd;

/* Create subscriber for GUI commands */
ros::Subscriber guiSub;

/* Create publishers for the various controller */
ros::Publisher gc_pub; //= nh.advertise<cdx
ros::Publisher pc_pub;
ros::Publisher vc_pub;

/*******************    FUNCTION IMPLEMENTATIONS    ********************/

int loadConfig(const std::string file) {
    ROS_INFO_STREAM("Loaded configuration parameters.");
    return 0;
}

void guiCmdReceived(const std_msgs::String::ConstPtr &s) {
    ROS_INFO_STREAM("Received cmd from gui " << s->data);
    if (s->data == "RUN") {
        cd.setRunStatus(1);
    } else if (s->data == "STOP") {
        cd.setRunStatus(0);
    }
}

void parseAction(CDXBot cd, const struct action a) {
    cdxbot::gc_cmd gmsg;
    cdxbot::pc_cmd pmsg;
    cdxbot::vc_cmd vmsg;

    if(a.cmd == "move") {
        /* Convert CRC (Container, Row, Column) coords to XYZ Coordinates. */
        if(a.args[1] > cd.getNumContainers()) {
            // THROW ERROR: CONTAINER INDEX OUT OF RANGE.
            return;
        }
        double x = cd.getContainer(a.args[1]).getGlobalCoords('x', a.args[2], a.args[3]);
        double y = cd.getContainer(a.args[1]).getGlobalCoords('y', a.args[2], a.args[3]);
        double z = cd.getContainer(a.args[1]).getGlobalCoords('z', a.args[2], a.args[3]);

        /* Move tip to feed plane */
        gmsg.cmd = "movez";
        gmsg.z = z;
        gc_pub.publish(gmsg);

        /* Rapid feed to calculated coordinates in feed plane */
        gmsg.cmd = "setvel";
        gmsg.vel = -1;     /* Feed at maximum velocity */
        gc_pub.publish(gmsg);

        gmsg.cmd = "movexy";
        gmsg.x = x;
        gmsg.y = y;
        gc_pub.publish(gmsg);

        /* Set velocity to plunge velocity */
        gmsg.cmd = "setvel";
        gmsg.vel = -1;     /* Feed at maximum velocity */
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
            return;
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
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cdxbot_node");
    ros::NodeHandle nh;

    /* Subscribe to GUI topic */
    guiSub = nh.subscribe("/gui_cmd", 1000, &guiCmdReceived);

    /* Create publishers for the various controller */
    //gc_pub = nh.advertise<cdxbot::gc_cmd.msg>();
    //pc_pub = ;
    //vc_pub = ;
    /* Check for and load/parse HLMD file */
    if(!cd.parseHLMDFile(cd.HLMDFileLocation)) {
        ROS_INFO_STREAM("Found HLMD file at " << cd.HLMDFileLocation <<".");
        ROS_DEBUG_STREAM(cd.actionMap.size() << "items pushed into actionmap.");
        ROS_DEBUG_STREAM("items pushed into actionmap");
        for(size_t i = 0; i < cd.actionMap.size(); i++) {
            ROS_DEBUG_STREAM(cd.actionMap[i].cmd);
            for(size_t j = 0; j < cd.actionMap[i].args.size(); j++) {
                ROS_DEBUG_STREAM("\t" << cd.actionMap[i].args[j]);
            }
        }
        ROS_INFO_STREAM("Parsed HLMD file successfully.");
    } else {
        ROS_INFO_STREAM("Error reading HLMD file.");
    }

    /* Process commands in HLMD file. */
    while(ros::ok()) {
        if(cd.getRunStatus() == 1) {
            struct action a;

            if(cd.getNextAction(a) < 0) {
                ROS_INFO_STREAM("DONE!");
                cd.setRunStatus(0);
            } else {
                ROS_INFO_STREAM("Got action: " << a.cmd);
                parseAction(cd, a);
            }
        } else {
            ros::spinOnce();
        }
    }
    return 0;
}
