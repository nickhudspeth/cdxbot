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
#include "CDXBot.h"
#include "std_msgs/String.h"
/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
std::string defaultConfigFilePath = ".";
CDXBot * cd;

/*******************    FUNCTION IMPLEMENTATIONS    ********************/





int main(int argc, char **argv) {
    ros::init(argc, argv, "cdxbot");
    ros::NodeHandle nh;

    /* Subscribe to GUI topic */
    ros::Subscriber guiSub = nh.subscribe("gui/cmd", 1000, &guiCmdReceived);
    cd->parseHLMDFile(cd->HLMDFileLocation);
    while(ros::ok()) {
        if(cd->getRunStatus()) {


        }
    }
    return 0;
}

int loadConfig(const std::string file) {
    ROS_INFO_STREAM("Loaded configuration parameters.");
    return 0;
}

void guiCmdReceived(const std_msgs::String::ConstPtr &s) {
    ROS_INFO_STREAM("Received cmd from gui" << s);
    if (s == "RUN") {
        cd->setRunStatus(1);
    }
}
