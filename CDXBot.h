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
    CDXBot () {
        /* Watch out for this. Make sure to update the z-position of the end
         * effector after homing. */
        _eepos[0] = 0;
        _eepos[1] = 0;
        _eepos[2] = 0;
    };
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
        if(_containers.size() < i) {
            for(int j = 0; j < (_containers.size() - i); j++) {
                _containers.push_back(Container());
            }
        }
    }

    Container& getContainer(unsigned int index) {
        if(index > _containers.size()) {
            printf("Requested container index %d is out of range [%d, %zu ]!", index, 0, _containers.size());
        }
        return _containers[index];
    }

    double getFeedPlaneHeight(void) {
        return _feed_plane;
    }

    double &getFeedPlaneHeightRef(void) {
        return _feed_plane;
    }

    void setFeedPlaneHeight(double z) {
        _feed_plane = z;
    };

    std::vector<class Container> &getContainersRef(void) {
        return _containers;
    }

    void setActionIndex(unsigned int index) {
        _action_index = index;
    }

    unsigned int getActionIndex(void) {
        return _action_index;
    }

    bool setGantryStatus(bool s) {
        _gantry_status = s;
    }

    bool getGantryStatus(void) {
        return _gantry_status;
    }
    void setPipetterHasZ(bool s) {
        _pipetter_has_z = s;
    }
    bool getPipetterHasZ(void) {
        return _pipetter_has_z;
    }
    bool &getPipetterHasZRef(void) {
        return _pipetter_has_z;
    }
    double getEEPos(unsigned int i) {
        if(i > 2) {
            std::cout << "ERROR " << __PRETTY_FUNCTION__ << "Requested axis is invalid. " << std::endl;
            return 0;
        }
        return _eepos[i];
    }
    void setEEPos(unsigned int i, double pos) {
        if(i > 2) {
            std::cout << "ERROR " << __PRETTY_FUNCTION__ << "Requested axis is invalid. " << std::endl;
        }
        _eepos[i] = pos;
    }

    void setEjectPos(unsigned int axis, double pos) {
        switch(axis) {
        case AXIS_X:
            _eject_x = pos;
            break;
        case AXIS_Y:
            _eject_y = pos;
            break;
        case AXIS_Z:
            _eject_z = pos;
            break;
        }
    }

    double getEjectPos(unsigned int axis) {
        double ret = 0.0f;
        switch(axis) {
        case AXIS_X:
            ret = _eject_x;
            break;
        case AXIS_Y:
            ret = _eject_y;
            break;
        case AXIS_Z:
            ret = _eject_z;
            break;
        }
        return ret;
    }

    double &getEjectPosRef(unsigned int axis) {
        double ret = 0.0f;
        switch(axis) {
        case AXIS_X:
            ret = _eject_x;
            break;
        case AXIS_Y:
            ret = _eject_y;
            break;
        case AXIS_Z:
            ret = _eject_z;
            break;
        }
        return ret;
    }

  private:
    double _feed_plane;  /* Feed plane height (mm) */
    const char *d = ","; /*HLMD File delimiter */
    unsigned int _runStatus = 0;
    unsigned int _action_index = 0;
    double _origin_x = 0.0;
    double _origin_y = 0.0;
    double _origin_z = 0.0;
    double _extents_x = 0.0;
    double _extents_y = 0.0;
    double _extents_z = 0.0;
    double _eject_x -0;
    double _eject_y = 0;
    bool _gantry_status = 0;
    bool _pipetter_has_z;
    double _eepos[3];
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
