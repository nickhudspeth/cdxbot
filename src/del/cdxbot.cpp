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
#include <confuse.h>

/*********************    CONSTANTS AND MACROS    **********************/


/***********************    GLOBAL VARIABLES    ************************/
std::string defaultConfigFilePath = ".";

/*******************    FUNCTION IMPLEMENTATIONS    ********************/



int loadConfig(const std::string file)
{
    cfg_t *cfg;
    cfg_opt_t opts[] = {
        CFG_INT((char*)"id", 1, CFGF_NONE),
        CFG_STR((char*)"pipetter_type", (char*)"NONE", CFGF_NONE),
        CFG_STR((char*)"controller_type", (char*)"NONE", CFGF_NONE),
        CFG_END(),
    };
    cfg = cfg_init(opts, CFGF_NOCASE);
    if(cfg_parse(cfg, file.c_str()) == CFG_PARSE_ERROR) {
        ROS_ERROR_STREAM("Error parsing config file.");
        return -1;
    }
    /* Load default settings from configuration file, if available.*/
    if(file.empty()) {
        std::string cfgPath = defaultConfigFilePath + "/" + file + std::string(".conf");
        std::cout << "Loading configuration from file at " << cfgPath << std::endl;
    }
    else{
        ROS_WARN_STREAM("No configuration file found at " + defaultConfigFilePath\
                + ". Loading default values for all parameters in " + file + ".conf!");

    }

}




int main(int argc, char **argv) {
    ros::init(argc, argv, "cdxbot");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Hello, ROS!");
    // while(ros::ok()){

    // }
    return 0;
}

