/*************************************************************************
Title:    cdxbot.h -
Author:   Nicholas Morrow <nickhudspeth@gmail.com> http://www.nickhudspeth.com
File:     cdxbot.h
Software: C Standard Library
Hardware: Platform Independent
License:  The MIT License (MIT)

DESCRIPTION:
    What does this module do?

USAGE:


NOTES:


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

*************************************************************************/

/**********************    INCLUDE DIRECTIVES    ***********************/
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <boost/tokenizer.hpp>
#include <map>
#include "Container.h"

/**************    CONSTANTS, MACROS, & DATA STRUCTURES    ***************/
class CDXBot {
  public:
    CDXBot () {};
    virtual ~CDXBot () {};
    int parseHLMDFile(const char *fname);
    void setRunStatus(unsigned int status);
    unsigned int getRunStatus(void) {
        return _runStatus;
    }
    const char *HLMDFileLocation = "/home/cdx/catkin_ws/src/cdxbot/run.hlmd";
    std::vector<struct action> actionMap;
    int getNextAction(struct action &a);
    int getNumContainers(void) {
        return _containers.size();
    }
    void setNumContainers(unsigned int i) {
        _num_containers = i;
    }

    int& getNumContainersRef(void) {
        return _num_containers;
    }

    // unsigned int getNumContainers(void) {
    // return _num_containers;
    // }

    Container& getContainer(unsigned int index) {
        return _containers[index];
    };
    double getFeedPlaneHeight(void) {
        return _feed_plane;
    }
    void setFeedPlaneHeight(double z);

    std::vector<class Container> &getContainersRef(void) {
        return _containers;
    }

  private:
    double _feed_plane;  /* Feed plane height (mm) */
    const char *d = ","; /*HLMD File delimiter */
    unsigned int _runStatus = 0;
    int _num_containers;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_z = 0.0;
    double extents_x = 0.0;
    double extents_y = 0.0;
    double extents_z = 0.0;

    //    std::map<std::string, struct action> actionMap;
    // std::vector<struct action> actionMap;
    void (*getActionPointer(std::string s))(std::vector<float>);
    /* data */
    // std::map<unsigned int, class Container> _containers; // List of deck containers.
    std::vector<class Container> _containers;
};

struct action {
    void (*fp)(std::vector<float>);
    std::string cmd;
    std::vector<float> args;
};

/***********************    FUNCTION PROTOTYPES    ***********************/
