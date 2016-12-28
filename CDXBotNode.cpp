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
#include "CDXBot.h"
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
std::string defaultConfigFilePath = ".";
CDXBot cd;

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "cdxbot_node");
    ros::NodeHandle nh;

    /* Subscribe to GUI topic */
    ros::Subscriber guiSub = nh.subscribe("/gui_cmd", 1000, &guiCmdReceived);

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
            }
        } else {
            ros::spinOnce();
        }
    }
    return 0;
}
